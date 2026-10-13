"""Integration: RobotGatewayNode thật + mock Nav2 NavigateToPose action server + MQTT client giả.

Cần ROS 2 Jazzy + nav2_msgs (chạy trên Pi / container ros:jazzy). Tự skip nếu thiếu.

  source /opt/ros/jazzy/setup.bash
  cd ros2_ws/src/warotrans_fleet && PYTHONPATH=.:$PYTHONPATH python3 -m pytest test/test_gateway_ros_integration.py -v
"""

import json
import math
import threading
import time
from types import SimpleNamespace

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("nav2_msgs")

from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionServer, CancelResponse, GoalResponse  # noqa: E402
from rclpy.callback_groups import ReentrantCallbackGroup  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from tf2_ros import TransformBroadcaster  # noqa: E402

from warotrans_fleet import robot_gateway_node  # noqa: E402

ROBOT = "RBT-IT"
PREFIX = f"warotrans/v1/robots/{ROBOT}"


class FakePaho:
    """Thay paho client: ghi lại publish, cho test bơm message vào như broker."""

    instances: list["FakePaho"] = []

    def __init__(self, client_id: str) -> None:
        self.client_id = client_id
        self.on_connect = None
        self.on_disconnect = None
        self.on_message = None
        self.subscriptions: list[str] = []
        self.published: list[tuple[str, dict, int, float]] = []
        self._lock = threading.Lock()
        self._connected = False
        FakePaho.instances.append(self)

    def connect(self, host, port, keepalive=60):
        self._connected = True
        self.on_connect(self, None, None, 0)
        return 0

    def loop_start(self):
        pass

    def loop_stop(self):
        pass

    def disconnect(self):
        self._connected = False

    def is_connected(self):
        return self._connected

    def subscribe(self, topic, qos=0):
        self.subscriptions.append(topic)

    def publish(self, topic, payload, qos=0, retain=False):
        with self._lock:
            self.published.append((topic, json.loads(payload), qos, time.monotonic()))

    def deliver(self, topic: str, body: dict) -> None:
        # Gọi trên thread của test — mô phỏng paho network thread.
        self.on_message(self, None, SimpleNamespace(topic=topic, payload=json.dumps(body).encode()))

    def messages(self, leaf: str) -> list[tuple[dict, int, float]]:
        with self._lock:
            return [(p, q, t) for (tp, p, q, t) in self.published if tp == f"{PREFIX}/{leaf}"]


class MockNav2(Node):
    """NavigateToPose server giả: chạy `duration` giây, feedback 10 Hz, tôn trọng cancel."""

    def __init__(self) -> None:
        super().__init__("mock_nav2")
        self.duration = 1.5
        self.mode = "succeed"  # succeed | abort | reject
        self.goals: list = []
        self.cancel_requests = 0
        self._server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    def _on_goal(self, goal_request):
        self.goals.append(goal_request)
        return GoalResponse.REJECT if self.mode == "reject" else GoalResponse.ACCEPT

    def _on_cancel(self, _goal_handle):
        self.cancel_requests += 1
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        start = time.monotonic()
        while time.monotonic() - start < self.duration:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return NavigateToPose.Result()
            fb = NavigateToPose.Feedback()
            fb.distance_remaining = float(self.duration - (time.monotonic() - start))
            goal_handle.publish_feedback(fb)
            time.sleep(0.1)
        if self.mode == "abort":
            goal_handle.abort()
        else:
            goal_handle.succeed()
        return NavigateToPose.Result()


class Localizer(Node):
    """Phát TF map→base_link có stamp hiện tại để gateway báo LOCALIZED."""

    def __init__(self) -> None:
        super().__init__("fake_localizer")
        self._tf = TransformBroadcaster(self)
        self.create_timer(0.05, self._publish)

    def _publish(self) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "base_link"
        t.transform.rotation.w = 1.0
        self._tf.sendTransform(t)


@pytest.fixture
def system(monkeypatch):
    FakePaho.instances.clear()
    monkeypatch.setattr(robot_gateway_node, "default_paho_factory", FakePaho)
    rclpy.init(
        args=[
            "--ros-args",
            "-p", f"robot_code:={ROBOT}",
            "-p", "heartbeat_hz:=5.0",
            "-p", "telemetry_hz:=5.0",
            "-p", "max_transform_age_sec:=1.0",
            "-p", "terminal_status_hold_sec:=0.5",
            "-p", "command_tick_hz:=10.0",
        ]
    )
    gateway = robot_gateway_node.RobotGatewayNode()
    nav2 = MockNav2()
    localizer = Localizer()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (gateway, nav2, localizer):
        executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    mqtt = _wait_for(lambda: FakePaho.instances[-1] if FakePaho.instances and FakePaho.instances[-1].subscriptions else None)
    _wait_for(lambda: _last_telemetry(mqtt, "localizationStatus") == "LOCALIZED" or None)
    _wait_for(lambda: _server_ready(gateway) or None)
    yield SimpleNamespace(gateway=gateway, nav2=nav2, mqtt=mqtt)

    executor.shutdown()
    for node in (gateway, nav2, localizer):
        node.destroy_node()
    rclpy.shutdown()
    spinner.join(timeout=2.0)


def _server_ready(gateway) -> bool:
    return gateway._commands.move._navigator.is_ready()


def _wait_for(predicate, timeout: float = 10.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.05)
    raise AssertionError("timed out waiting for condition")


def _last_telemetry(mqtt: FakePaho, key: str):
    telemetry = mqtt.messages("telemetry")
    return telemetry[-1][0].get(key) if telemetry else None


def _navigate(command_id: str, **payload) -> dict:
    body = {
        "schemaVersion": 1,
        "messageId": "m-" + command_id,
        "commandId": command_id,
        "robotCode": ROBOT,
        "type": "NAVIGATE_TO_POSE",
        "issuedAt": "2026-10-13T08:00:00.000Z",
        "jobStepId": "step-1",
        "payload": {"frameId": "map", "x": 1.0, "y": 2.0, "yaw": math.pi / 2},
    }
    body["payload"].update(payload)
    return body


def _result_for(mqtt: FakePaho, command_id: str):
    for payload, qos, _t in mqtt.messages("command_result"):
        if payload["commandId"] == command_id:
            assert qos == 1
            return payload
    return None


def test_navigate_success_end_to_end_with_heartbeat_during_motion(system):
    mqtt, nav2 = system.mqtt, system.nav2
    mqtt.deliver(f"{PREFIX}/command", _navigate("it-1"))

    ack, qos, ack_t = _wait_for(lambda: mqtt.messages("command_ack") and mqtt.messages("command_ack")[0])
    assert qos == 1 and ack["accepted"] is True and ack["commandId"] == "it-1"

    _wait_for(lambda: _last_telemetry(mqtt, "navigationStatus") == "NAVIGATING" or None)
    assert _last_telemetry(mqtt, "currentCommandId") == "it-1"

    result = _wait_for(lambda: _result_for(mqtt, "it-1"))
    result_t = next(t for p, _q, t in mqtt.messages("command_result") if p["commandId"] == "it-1")
    assert result["outcome"] == "SUCCEEDED"
    assert result["jobStepId"] == "step-1"

    goal = nav2.goals[0].pose
    assert goal.header.frame_id == "map"
    assert (goal.pose.position.x, goal.pose.position.y) == (1.0, 2.0)
    assert abs(goal.pose.orientation.z - math.sin(math.pi / 4)) < 1e-9

    # Heartbeat + telemetry vẫn đều trong lúc robot đang chạy (5 Hz × ~1.5 s).
    hb_during = [t for _p, _q, t in mqtt.messages("heartbeat") if ack_t <= t <= result_t]
    tel_during = [t for _p, _q, t in mqtt.messages("telemetry") if ack_t <= t <= result_t]
    assert len(hb_during) >= 5, hb_during
    assert len(tel_during) >= 5, tel_during

    _wait_for(lambda: _last_telemetry(mqtt, "navigationStatus") == "IDLE" or None)
    assert _last_telemetry(mqtt, "currentCommandId") is None


def test_backend_cancel_stops_nav2_goal(system):
    mqtt, nav2 = system.mqtt, system.nav2
    nav2.duration = 10.0
    mqtt.deliver(f"{PREFIX}/command", _navigate("it-2"))
    _wait_for(lambda: _last_telemetry(mqtt, "navigationStatus") == "NAVIGATING" or None)

    mqtt.deliver(
        f"{PREFIX}/command",
        {
            "schemaVersion": 1,
            "messageId": "m-c",
            "commandId": "it-2-cancel",
            "robotCode": ROBOT,
            "type": "CANCEL",
            "payload": {"targetCommandId": "it-2"},
        },
    )
    nav_result = _wait_for(lambda: _result_for(mqtt, "it-2"), timeout=5.0)
    cancel_result = _wait_for(lambda: _result_for(mqtt, "it-2-cancel"), timeout=5.0)
    assert nav2.cancel_requests == 1
    assert nav_result["outcome"] == "CANCELED"
    assert cancel_result["outcome"] == "SUCCEEDED"


def test_timeout_cancels_goal_and_reports_navigation_timeout(system):
    mqtt, nav2 = system.mqtt, system.nav2
    nav2.duration = 10.0
    mqtt.deliver(f"{PREFIX}/command", _navigate("it-3", timeoutSeconds=1.0))
    result = _wait_for(lambda: _result_for(mqtt, "it-3"), timeout=6.0)
    assert nav2.cancel_requests == 1
    assert result["outcome"] == "FAILED"
    assert result["errorCode"] == "NAVIGATION_TIMEOUT"


def test_goal_rejected_and_duplicate_and_busy(system):
    mqtt, nav2 = system.mqtt, system.nav2
    nav2.mode = "reject"
    mqtt.deliver(f"{PREFIX}/command", _navigate("it-4"))
    result = _wait_for(lambda: _result_for(mqtt, "it-4"))
    assert result["outcome"] == "FAILED" and result["errorCode"] == "GOAL_REJECTED"

    nav2.mode = "succeed"
    nav2.duration = 3.0
    mqtt.deliver(f"{PREFIX}/command", _navigate("it-5"))
    mqtt.deliver(f"{PREFIX}/command", _navigate("it-5"))  # QoS1 duplicate
    mqtt.deliver(f"{PREFIX}/command", _navigate("it-6"))  # MOVE mới khi đang chạy
    def acks_5_6():
        acks = [a for a, _q, _t in mqtt.messages("command_ack") if a["commandId"] in ("it-5", "it-6")]
        return acks if len(acks) >= 3 else None

    acks = _wait_for(acks_5_6)
    by_id = [(a["commandId"], a["accepted"], a["reasonCode"]) for a in acks]
    assert by_id == [("it-5", True, None), ("it-5", True, "ALREADY_ACCEPTED"), ("it-6", False, "BUSY")]
    _wait_for(lambda: _result_for(mqtt, "it-5"), timeout=6.0)
    assert len(nav2.goals) == 2  # it-4 (rejected) + it-5; không có goal thứ hai cho it-5, không có it-6
