"""Non-blocking MQTT publisher with reconnect (paho-mqtt)."""

from __future__ import annotations

import json
import logging
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol

logger = logging.getLogger(__name__)


class MqttClientLike(Protocol):
    def connect(self, host: str, port: int, keepalive: int = 60) -> int: ...
    def loop_start(self) -> None: ...
    def loop_stop(self) -> None: ...
    def disconnect(self) -> None: ...
    def publish(self, topic: str, payload: str, qos: int = 0, retain: bool = False) -> Any: ...
    def is_connected(self) -> bool: ...

    # paho callbacks
    on_connect: Any
    on_disconnect: Any


ClientFactory = Callable[[str], MqttClientLike]


@dataclass(frozen=True)
class OutboundMessage:
    topic: str
    payload: dict[str, Any]


class ReconnectingMqttPublisher:
    """Queue publishes on a worker thread; ROS callbacks must only call publish()."""

    def __init__(
        self,
        *,
        host: str,
        port: int,
        client_id: str,
        client_factory: ClientFactory,
        reconnect_backoff_sec: float = 1.0,
        max_backoff_sec: float = 10.0,
        queue_size: int = 200,
    ) -> None:
        self._host = host
        self._port = port
        self._client_id = client_id
        self._client_factory = client_factory
        self._reconnect_backoff_sec = reconnect_backoff_sec
        self._max_backoff_sec = max_backoff_sec
        self._queue: queue.Queue[Optional[OutboundMessage]] = queue.Queue(maxsize=queue_size)
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._connected = threading.Event()
        self._client: Optional[MqttClientLike] = None

    @property
    def is_connected(self) -> bool:
        return self._connected.is_set()

    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="mqtt-publisher", daemon=True)
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

    def publish(self, topic: str, payload: dict[str, Any]) -> bool:
        """Non-blocking enqueue. Returns False if the queue is full (drop)."""
        try:
            self._queue.put_nowait(OutboundMessage(topic=topic, payload=payload))
            return True
        except queue.Full:
            logger.warning("MQTT outbound queue full; dropping message topic=%s", topic)
            return False

    def _run(self) -> None:
        backoff = self._reconnect_backoff_sec
        while not self._stop.is_set():
            try:
                client = self._client_factory(self._client_id)
                self._client = client

                def _on_connect(_c, _u, _f, rc, *_a):
                    if rc == 0:
                        self._connected.set()
                        logger.info("MQTT connected %s:%s", self._host, self._port)
                    else:
                        self._connected.clear()
                        logger.warning("MQTT connect failed rc=%s", rc)

                def _on_disconnect(_c, _u, rc, *_a):
                    self._connected.clear()
                    logger.warning("MQTT disconnected rc=%s", rc)

                client.on_connect = _on_connect
                client.on_disconnect = _on_disconnect
                rc = client.connect(self._host, self._port, keepalive=30)
                if rc != 0:
                    raise ConnectionError(f"MQTT connect returned rc={rc}")
                client.loop_start()

                # Wait briefly for connect ack
                for _ in range(40):
                    if self._stop.is_set() or self._connected.is_set():
                        break
                    time.sleep(0.05)

                if not self._connected.is_set():
                    raise ConnectionError("MQTT connect timeout")

                backoff = self._reconnect_backoff_sec
                self._drain_loop(client)
            except Exception as exc:  # noqa: BLE001 — reconnect loop
                logger.warning("MQTT publisher error: %s; retry in %.1fs", exc, backoff)
                self._teardown_client()
                self._stop.wait(backoff)
                backoff = min(backoff * 2.0, self._max_backoff_sec)
            finally:
                self._teardown_client()

    def _drain_loop(self, client: MqttClientLike) -> None:
        while not self._stop.is_set():
            if not client.is_connected():
                raise ConnectionError("MQTT connection lost")
            try:
                item = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if item is None:
                return
            body = json.dumps(item.payload, separators=(",", ":"))
            client.publish(item.topic, body, qos=0, retain=False)

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


def default_paho_factory(client_id: str) -> MqttClientLike:
    import paho.mqtt.client as mqtt

    # Callback API v2 if available; fall back for older paho.
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:
        return mqtt.Client(client_id=client_id)
