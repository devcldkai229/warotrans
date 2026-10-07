import time
from typing import Any, List, Tuple

from warotrans_fleet.mqtt_publisher import ReconnectingMqttPublisher


class FakeMqttClient:
    def __init__(self, client_id: str, fail_connects: int = 0) -> None:
        self.client_id = client_id
        self.fail_connects = fail_connects
        self._connect_attempts = 0
        self._connected = False
        self.published: List[Tuple[str, str]] = []
        self.on_connect = None
        self.on_disconnect = None
        self.loop_started = False

    def connect(self, host: str, port: int, keepalive: int = 60) -> int:
        self._connect_attempts += 1
        if self._connect_attempts <= self.fail_connects:
            if self.on_connect:
                self.on_connect(self, None, None, 1)
            return 1
        self._connected = True
        if self.on_connect:
            self.on_connect(self, None, None, 0)
        return 0

    def loop_start(self) -> None:
        self.loop_started = True

    def loop_stop(self) -> None:
        self.loop_started = False

    def disconnect(self) -> None:
        was = self._connected
        self._connected = False
        if was and self.on_disconnect:
            self.on_disconnect(self, None, 0)

    def publish(self, topic: str, payload: str, qos: int = 0, retain: bool = False) -> Any:
        assert qos == 0
        assert retain is False
        self.published.append((topic, payload))
        return None

    def is_connected(self) -> bool:
        return self._connected


def test_publisher_reconnects_after_failed_connect():
    clients: List[FakeMqttClient] = []

    def factory(client_id: str) -> FakeMqttClient:
        # First client fails once, second succeeds — simulate reconnect cycle via fail_connects on shared counter
        client = FakeMqttClient(client_id, fail_connects=0)
        if len(clients) == 0:
            client = FakeMqttClient(client_id, fail_connects=1)
        clients.append(client)
        return client

    pub = ReconnectingMqttPublisher(
        host="localhost",
        port=1883,
        client_id="test",
        client_factory=factory,
        reconnect_backoff_sec=0.05,
        max_backoff_sec=0.1,
    )
    pub.start()
    deadline = time.time() + 3.0
    while time.time() < deadline and not pub.is_connected:
        time.sleep(0.05)
    assert pub.is_connected
    assert pub.publish("warotrans/v1/robots/RBT-001/heartbeat", {"schemaVersion": 1})
    deadline = time.time() + 2.0
    while time.time() < deadline and not any(clients[-1].published):
        time.sleep(0.05)
    assert clients[-1].published
    topic, body = clients[-1].published[0]
    assert topic.endswith("/heartbeat")
    assert "schemaVersion" in body
    pub.stop()
