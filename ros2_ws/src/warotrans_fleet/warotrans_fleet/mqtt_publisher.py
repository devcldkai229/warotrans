"""Non-blocking MQTT client with reconnect (paho-mqtt): publish + optional subscribe."""

from __future__ import annotations

from warotrans_fleet import _vendor_path  # noqa: F401 — vendored paho for WSL without pip

import json
import logging
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol, Sequence

logger = logging.getLogger(__name__)

MessageCallback = Callable[[str, dict[str, Any]], None]


class MqttClientLike(Protocol):
    def connect(self, host: str, port: int, keepalive: int = 60) -> int: ...
    def loop_start(self) -> None: ...
    def loop_stop(self) -> None: ...
    def disconnect(self) -> None: ...
    def publish(self, topic: str, payload: str, qos: int = 0, retain: bool = False) -> Any: ...
    def subscribe(self, topic: str, qos: int = 0) -> Any: ...
    def is_connected(self) -> bool: ...

    on_connect: Any
    on_disconnect: Any
    on_message: Any


ClientFactory = Callable[[str], MqttClientLike]


@dataclass(frozen=True)
class OutboundMessage:
    topic: str
    payload: dict[str, Any]
    qos: int = 0


class ReconnectingMqttPublisher:
    """Queue publishes on a worker thread; optional subscribe with inbound callback."""

    def __init__(
        self,
        *,
        host: str,
        port: int,
        client_id: str,
        client_factory: ClientFactory,
        subscribe_topic: Optional[str | Sequence[str]] = None,
        on_message: Optional[MessageCallback] = None,
        reconnect_backoff_sec: float = 1.0,
        max_backoff_sec: float = 10.0,
        queue_size: int = 200,
    ) -> None:
        self._host = host
        self._port = port
        self._client_id = client_id
        self._client_factory = client_factory
        # One topic (str) or several (command + teleop), all re-subscribed on every reconnect.
        if subscribe_topic is None:
            self._subscribe_topics: tuple[str, ...] = ()
        elif isinstance(subscribe_topic, str):
            self._subscribe_topics = (subscribe_topic,)
        else:
            self._subscribe_topics = tuple(subscribe_topic)
        self._on_message = on_message
        self._reconnect_backoff_sec = reconnect_backoff_sec
        self._max_backoff_sec = max_backoff_sec
        self._queue: queue.Queue[Optional[OutboundMessage]] = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._connected = threading.Event()
        self._client: Optional[MqttClientLike] = None
        # Fault Injection Lab (IOT-01): off the broker until this time.monotonic() value.
        self._cut_until = 0.0

    @property
    def is_connected(self) -> bool:
        return self._connected.is_set()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="mqtt-client", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        try:
            self._queue.put_nowait(None)
        except queue.Full:
            pass
        if self._thread:
            self._thread.join(timeout=timeout)
        self._thread = None

    def cut_until(self, monotonic_deadline: float) -> None:
        """Lab only: drop the connection now and stay off the broker until the deadline (then reconnect)."""
        self._cut_until = float(monotonic_deadline)

    def _is_cut(self) -> bool:
        return time.monotonic() < self._cut_until

    def publish(self, topic: str, payload: dict[str, Any], qos: int = 0) -> bool:
        """Non-blocking enqueue. Returns False if the queue is full (drop)."""
        try:
            self._queue.put_nowait(OutboundMessage(topic=topic, payload=payload, qos=qos))
            return True
        except queue.Full:
            logger.warning("MQTT outbound queue full; dropping message topic=%s", topic)
            return False

    def _run(self) -> None:
        backoff = self._reconnect_backoff_sec
        while not self._stop.is_set():
            if self._is_cut():
                self._stop.wait(0.5)
                continue
            try:
                client = self._client_factory(self._client_id)
                self._client = client

                def _on_connect(_c, _u, _f, rc, *_a):
                    if rc == 0 or getattr(rc, "value", rc) == 0:
                        self._connected.set()
                        logger.info("MQTT connected %s:%s", self._host, self._port)
                        for topic in self._subscribe_topics:
                            try:
                                client.subscribe(topic, qos=1)
                                logger.info("MQTT subscribed %s", topic)
                            except Exception as exc:  # noqa: BLE001
                                logger.warning("MQTT subscribe failed %s: %s", topic, exc)
                    else:
                        self._connected.clear()
                        logger.warning("MQTT connect failed rc=%s", rc)

                def _on_disconnect(_c, _u, rc, *_a):
                    self._connected.clear()
                    logger.warning("MQTT disconnected rc=%s", rc)

                def _on_message(_c, _u, msg):
                    if self._on_message is None:
                        return
                    try:
                        body = json.loads(msg.payload.decode("utf-8"))
                    except Exception:  # noqa: BLE001
                        logger.warning("Invalid MQTT JSON on %s", msg.topic)
                        return
                    try:
                        self._on_message(msg.topic, body)
                    except Exception as exc:  # noqa: BLE001
                        logger.exception("MQTT on_message handler error: %s", exc)

                client.on_connect = _on_connect
                client.on_disconnect = _on_disconnect
                client.on_message = _on_message
                rc = client.connect(self._host, self._port, keepalive=30)
                if rc != 0:
                    raise ConnectionError(f"MQTT connect returned rc={rc}")
                client.loop_start()

                for _ in range(40):
                    if self._stop.is_set() or self._connected.is_set():
                        break
                    time.sleep(0.05)

                if not self._connected.is_set():
                    raise ConnectionError("MQTT connect timeout")

                backoff = self._reconnect_backoff_sec
                self._drain_loop(client)
            except Exception as exc:  # noqa: BLE001 — reconnect loop
                logger.warning("MQTT client error: %s; retry in %.1fs", exc, backoff)
                self._teardown_client()
                self._stop.wait(backoff)
                backoff = min(backoff * 2.0, self._max_backoff_sec)
            finally:
                self._teardown_client()

    def _drain_loop(self, client: MqttClientLike) -> None:
        while not self._stop.is_set():
            # Prefer our on_connect/on_disconnect latch. paho Client.is_connected() can
            # return False under CallbackAPIVersion.VERSION2 even after a successful
            # connect — that caused endless "MQTT connection lost" and zero grants
            # (authority_ok stuck False → gate blocks all cmd_vel).
            if not self._connected.is_set():
                raise ConnectionError("MQTT connection lost")
            if self._is_cut():
                raise ConnectionError("MQTT link cut by the Fault Injection Lab")
            try:
                item = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                return
            body = json.dumps(item.payload, separators=(",", ":"))
            client.publish(item.topic, body, qos=item.qos, retain=False)

    def _teardown_client(self) -> None:
        self._connected.clear()
        client = self._client
        self._client = None
        if client is None:
            return
        try:
            client.loop_stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            client.disconnect()
        except Exception:  # noqa: BLE001
            pass


@dataclass(frozen=True)
class MqttTlsConfig:
    """MQTT over TLS with an X.509 client certificate (AWS IoT Core: port 8883, thing certificate).

    ``ca_file`` empty: the system trust store (it holds Amazon Root CA 1). Files are read by paho at connect time;
    the key file must stay readable by the robot user only (0600).
    """

    cert_file: str
    key_file: str
    ca_file: str = ""

    def validate(self) -> None:
        import os

        for label, path in (("mqtt_cert_file", self.cert_file), ("mqtt_key_file", self.key_file)):
            if not path or not os.path.isfile(path):
                raise ValueError(f"MQTT TLS: {label} {path!r} is not a file")
        if self.ca_file and not os.path.isfile(self.ca_file):
            raise ValueError(f"MQTT TLS: mqtt_ca_file {self.ca_file!r} is not a file")


def with_tls(base: ClientFactory, tls: MqttTlsConfig) -> ClientFactory:
    """Wrap a client factory so every new client speaks TLS 1.2+ and presents the client certificate."""
    import ssl

    tls.validate()

    def factory(client_id: str) -> MqttClientLike:
        client = base(client_id)
        client.tls_set(  # type: ignore[attr-defined]
            ca_certs=tls.ca_file or None,
            certfile=tls.cert_file,
            keyfile=tls.key_file,
            cert_reqs=ssl.CERT_REQUIRED,
            tls_version=ssl.PROTOCOL_TLS_CLIENT,
        )
        return client

    return factory


def default_paho_factory(client_id: str) -> MqttClientLike:
    import paho.mqtt.client as mqtt

    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:
        return mqtt.Client(client_id=client_id)
