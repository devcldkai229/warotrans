"""ROS2 ↔ MQTT Robot Gateway: heartbeat/telemetry uplink + command downlink.

Threading:
- MQTT network thread (paho) chỉ parse JSON rồi đẩy vào inbox + trigger guard condition.
  Không gọi rclpy / Nav2 từ thread đó, không block.
- ROS executor thread: CommandRouter, MoveCommandHandler, Nav2 action futures, timers
  (heartbeat / telemetry / command tick). Không callback nào chờ Nav2 đồng bộ, nên
  heartbeat + telemetry vẫn chạy đều trong lúc robot đang MOVE.
"""

from __future__ import annotations

import os
import queue
from typing import Any, Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from warotrans_fleet import navigation_state as nav_state
from warotrans_fleet.command_stack import build_command_stack
from warotrans_fleet.mqtt_publisher import ReconnectingMqttPublisher, default_paho_factory
from warotrans_fleet.nav2_status_tracker import Nav2StatusTracker
from warotrans_fleet.navigator import NavigatorPort, UnavailableNavigator
from warotrans_fleet.payload_builder import build_heartbeat, build_telemetry
from warotrans_fleet.sequence import SequenceClock
from warotrans_fleet.transforms import quaternion_to_yaw


def _env_or(param_value: str, env_name: str) -> str:
    env = os.environ.get(env_name)
    return env if env is not None and env != "" else param_value


class _MqttCommandEvents:
    """CommandEventPublisher → MQTT QoS1 trên command_ack / command_result."""

    def __init__(self, node: "RobotGatewayNode") -> None:
        self._node = node

    def publish_ack(self, payload: dict[str, Any]) -> None:
        self._publish(self._node.topic("command_ack"), payload)

    def publish_result(self, payload: dict[str, Any]) -> None:
        self._publish(self._node.topic("command_result"), payload)

    def _publish(self, topic: str, payload: dict[str, Any]) -> None:
        if not self._node.mqtt.publish(topic, payload, qos=1):
            self._node.get_logger().error(
                f"[MqttPublishDropped] topic={topic} commandId={payload.get('commandId')} (outbound queue full)"
            )


class RobotGatewayNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_gateway")

        self.declare_parameter("robot_code", "RBT-001")
        self.declare_parameter("mqtt_host", "localhost")
        self.declare_parameter("mqtt_port", 1883)
        self.declare_parameter("mqtt_client_id", "warotrans-robot-gateway")
        self.declare_parameter("topic_prefix", "warotrans/v1")
        self.declare_parameter("map_version_code", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")
        self.declare_parameter("navigate_action", "navigate_to_pose")
        self.declare_parameter("heartbeat_hz", 1.0)
        self.declare_parameter("telemetry_hz", 5.0)
        self.declare_parameter("max_transform_age_sec", 0.5)
        # MOVE command
        self.declare_parameter("default_navigate_timeout_sec", 0.0)  # 0 = không timeout nếu command không gửi
        self.declare_parameter("goal_response_timeout_sec", 10.0)
        self.declare_parameter("cancel_confirm_timeout_sec", 10.0)
        self.declare_parameter("terminal_status_hold_sec", 3.0)
        self.declare_parameter("command_tick_hz", 5.0)
        self.declare_parameter("command_ledger_size", 500)

        self._robot_code = _env_or(self._str_param("robot_code"), "WARO_ROBOT_CODE")
        self._mqtt_host = _env_or(self._str_param("mqtt_host"), "WARO_MQTT_HOST")
        mqtt_port_param = int(self.get_parameter("mqtt_port").get_parameter_value().integer_value)
        port_env = os.environ.get("WARO_MQTT_PORT")
        self._mqtt_port = int(port_env) if port_env else mqtt_port_param
        self._topic_prefix = self._str_param("topic_prefix").strip().rstrip("/")
        self._map_version_code = _env_or(self._str_param("map_version_code"), "WARO_MAP_VERSION_CODE") or None
        self._map_frame = self._str_param("map_frame")
        self._base_frame = self._str_param("base_frame")
        self._max_transform_age = self._float_param("max_transform_age_sec")
        client_id = self._str_param("mqtt_client_id")

        default_timeout = self._float_param("default_navigate_timeout_sec")
        goal_response_timeout = self._float_param("goal_response_timeout_sec")
        cancel_confirm_timeout = self._float_param("cancel_confirm_timeout_sec")
        terminal_hold = self._float_param("terminal_status_hold_sec")
        tick_hz = self._float_param("command_tick_hz")
        ledger_size = int(self.get_parameter("command_ledger_size").get_parameter_value().integer_value)
        if default_timeout < 0 or goal_response_timeout <= 0 or cancel_confirm_timeout <= 0:
            raise ValueError("navigate timeouts must be > 0 (default_navigate_timeout_sec may be 0 = disabled)")
        if terminal_hold < 0 or tick_hz <= 0 or ledger_size < 1:
            raise ValueError("terminal_status_hold_sec >= 0, command_tick_hz > 0, command_ledger_size >= 1")

        self._seq = SequenceClock()
        self._nav_tracker = Nav2StatusTracker()
        self._ever_localized = False
        self._last_localization = "UNKNOWN"
        self._lin_vel = 0.0
        self._ang_vel = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        odom_topic = self._str_param("odom_topic")
        amcl_topic = self._str_param("amcl_pose_topic")
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            self._on_amcl_pose,
            qos_profile_sensor_data,
        )

        navigator = self._setup_nav2()

        self.mqtt = ReconnectingMqttPublisher(
            host=self._mqtt_host,
            port=self._mqtt_port,
            client_id=f"{client_id}-{self._robot_code}",
            client_factory=default_paho_factory,
            subscribe_topic=self.topic("command"),
            on_message=self._on_mqtt_message,
        )

        self._commands = build_command_stack(
            robot_code=self._robot_code,
            publisher=_MqttCommandEvents(self),
            navigator=navigator,
            localization_provider=lambda: self._last_localization,
            log=self.get_logger(),
            map_frame=self._map_frame,
            robot_map_version_code=self._map_version_code,
            default_timeout_sec=default_timeout or None,
            goal_response_timeout_sec=goal_response_timeout,
            cancel_confirm_timeout_sec=cancel_confirm_timeout,
            terminal_status_hold_sec=terminal_hold,
            ledger_capacity=ledger_size,
            external_goal_active=lambda: self._nav_tracker.has_active_goal,
        )

        # Inbox: MQTT thread → executor thread.
        self._inbox: "queue.SimpleQueue[dict[str, Any]]" = queue.SimpleQueue()
        self._inbox_signal = self.create_guard_condition(self._drain_inbox)
        self.create_timer(1.0 / tick_hz, self._commands.router.tick)

        self.mqtt.start()

        hb_hz = max(0.1, self._float_param("heartbeat_hz"))
        tel_hz = max(0.1, self._float_param("telemetry_hz"))
        self.create_timer(1.0 / hb_hz, self._on_heartbeat_timer)
        self.create_timer(1.0 / tel_hz, self._on_telemetry_timer)

        self.get_logger().info(
            f"Robot gateway started robot_code={self._robot_code} "
            f"mqtt={self._mqtt_host}:{self._mqtt_port} "
            f"frames={self._map_frame}->{self._base_frame} bootId={self._seq.boot_id} "
            f"subscribe={self.topic('command')} commands={list(self._commands.router.supported_types)} "
            f"mapVersion={self._map_version_code}"
        )

    # ------------------------------------------------------------------ setup helpers

    def _str_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).get_parameter_value().double_value)

    def _setup_nav2(self) -> NavigatorPort:
        action_name = self._str_param("navigate_action")
        try:
            from action_msgs.msg import GoalStatusArray

            from warotrans_fleet.nav2_navigator import Nav2ActionNavigator
        except ImportError as exc:
            self.get_logger().error(f"Nav2 unavailable ({exc}); NAVIGATE_TO_POSE will be rejected NAV2_UNAVAILABLE")
            return UnavailableNavigator()

        navigator = Nav2ActionNavigator(self, action_name)
        self.create_subscription(GoalStatusArray, f"{action_name}/_action/status", self._on_goal_status, 10)
        self.get_logger().info(f"Nav2 ActionClient on {action_name}")
        return navigator

    def topic(self, leaf: str) -> str:
        return f"{self._topic_prefix}/robots/{self._robot_code}/{leaf}"

    def destroy_node(self) -> bool:
        self.mqtt.stop()
        return super().destroy_node()

    # ------------------------------------------------------------------ command downlink

    def _on_mqtt_message(self, topic: str, body: dict) -> None:
        # MQTT network thread: chỉ enqueue, không xử lý.
        if topic != self.topic("command"):
            return
        self._inbox.put(body)
        self._inbox_signal.trigger()

    def _drain_inbox(self) -> None:
        while True:
            try:
                body = self._inbox.get_nowait()
            except queue.Empty:
                return
            self._commands.router.handle_message(body)

    # ------------------------------------------------------------------ ROS inputs

    def _on_odom(self, msg: Odometry) -> None:
        self._lin_vel = float(msg.twist.twist.linear.x)
        self._ang_vel = float(msg.twist.twist.angular.z)

    def _on_amcl_pose(self, _msg: PoseWithCovarianceStamped) -> None:
        pass

    def _on_goal_status(self, msg) -> None:
        self._nav_tracker.update_from_status_list([int(s.status) for s in msg.status_list])

    # ------------------------------------------------------------------ uplink

    def _on_heartbeat_timer(self) -> None:
        seq = self._seq.next_heartbeat()
        payload = build_heartbeat(
            robot_code=self._robot_code,
            boot_id=self._seq.boot_id,
            sequence=seq,
        )
        self.mqtt.publish(self.topic("heartbeat"), payload)

    def _lookup_map_pose(self) -> tuple[Optional[dict[str, float]], str]:
        try:
            when = Time()
            transform = self._tf_buffer.lookup_transform(
                self._map_frame,
                self._base_frame,
                when,
                timeout=Duration(seconds=0.05),
            )
            stamp = transform.header.stamp
            age = (self.get_clock().now() - Time.from_msg(stamp)).nanoseconds / 1e9
            if age > self._max_transform_age:
                status = "LOST" if self._ever_localized else "UNKNOWN"
                return None, status

            t = transform.transform.translation
            q = transform.transform.rotation
            yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)
            self._ever_localized = True
            return {"x": float(t.x), "y": float(t.y), "yaw": float(yaw)}, "LOCALIZED"
        except TransformException:
            status = "LOST" if self._ever_localized else "UNKNOWN"
            return None, status

    def _navigation_status(self) -> str:
        status = self._commands.move.navigation_status
        if status == nav_state.IDLE and self._nav_tracker.has_active_goal:
            # Goal không do fleet gửi (RViz / CLI) — vẫn phản ánh robot đang chạy.
            return nav_state.NAVIGATING
        return status

    def _on_telemetry_timer(self) -> None:
        pose, localization_status = self._lookup_map_pose()
        self._last_localization = localization_status
        if localization_status != "LOCALIZED":
            pose = None

        seq = self._seq.next_telemetry()
        payload = build_telemetry(
            robot_code=self._robot_code,
            boot_id=self._seq.boot_id,
            sequence=seq,
            map_version_code=self._map_version_code,
            pose=pose,
            battery_percent=None,
            navigation_status=self._navigation_status(),
            localization_status=localization_status,
            linear_velocity=self._lin_vel,
            angular_velocity=self._ang_vel,
            current_command_id=self._commands.move.active_command_id,
            error_code=None,
        )
        self.mqtt.publish(self.topic("telemetry"), payload)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotGatewayNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        # Jazzy tự shutdown context khi nhận SIGINT — shutdown lần hai sẽ raise RCLError.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
