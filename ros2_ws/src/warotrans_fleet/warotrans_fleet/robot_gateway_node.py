"""ROS2 ↔ MQTT Robot Gateway: heartbeat (1 Hz) + telemetry (~5 Hz)."""

from __future__ import annotations

import os
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from warotrans_fleet.mqtt_publisher import ReconnectingMqttPublisher, default_paho_factory
from warotrans_fleet.nav2_status_tracker import Nav2StatusTracker
from warotrans_fleet.payload_builder import build_heartbeat, build_telemetry
from warotrans_fleet.sequence import SequenceClock
from warotrans_fleet.transforms import quaternion_to_yaw


def _env_or(param_value: str, env_name: str) -> str:
    env = os.environ.get(env_name)
    return env if env is not None and env != "" else param_value


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

        self._robot_code = _env_or(self.get_parameter("robot_code").get_parameter_value().string_value, "WARO_ROBOT_CODE")
        self._mqtt_host = _env_or(self.get_parameter("mqtt_host").get_parameter_value().string_value, "WARO_MQTT_HOST")
        mqtt_port_param = int(self.get_parameter("mqtt_port").get_parameter_value().integer_value)
        port_env = os.environ.get("WARO_MQTT_PORT")
        self._mqtt_port = int(port_env) if port_env else mqtt_port_param
        self._topic_prefix = self.get_parameter("topic_prefix").get_parameter_value().string_value.strip().rstrip("/")
        self._map_version_code = _env_or(
            self.get_parameter("map_version_code").get_parameter_value().string_value,
            "WARO_MAP_VERSION_CODE",
        ) or None
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self._max_transform_age = float(
            self.get_parameter("max_transform_age_sec").get_parameter_value().double_value
        )
        client_id = self.get_parameter("mqtt_client_id").get_parameter_value().string_value

        # Do not name this `_clock` — that shadows rclpy Node._clock used by create_timer.
        self._seq = SequenceClock()
        self._nav = Nav2StatusTracker()
        self._ever_localized = False
        self._lin_vel = 0.0
        self._ang_vel = 0.0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        amcl_topic = self.get_parameter("amcl_pose_topic").get_parameter_value().string_value
        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_topic,
            self._on_amcl_pose,
            qos_profile_sensor_data,
        )

        self._action_client = None
        self._setup_nav2_action()

        self._mqtt = ReconnectingMqttPublisher(
            host=self._mqtt_host,
            port=self._mqtt_port,
            client_id=f"{client_id}-{self._robot_code}",
            client_factory=default_paho_factory,
        )
        self._mqtt.start()

        hb_hz = max(0.1, float(self.get_parameter("heartbeat_hz").get_parameter_value().double_value))
        tel_hz = max(0.1, float(self.get_parameter("telemetry_hz").get_parameter_value().double_value))
        self.create_timer(1.0 / hb_hz, self._on_heartbeat_timer)
        self.create_timer(1.0 / tel_hz, self._on_telemetry_timer)

        self.get_logger().info(
            f"Robot gateway started robot_code={self._robot_code} "
            f"mqtt={self._mqtt_host}:{self._mqtt_port} "
            f"frames={self._map_frame}->{self._base_frame} bootId={self._seq.boot_id}"
        )

    def _setup_nav2_action(self) -> None:
        try:
            from action_msgs.msg import GoalStatusArray
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient

            action_name = self.get_parameter("navigate_action").get_parameter_value().string_value
            self._action_client = ActionClient(self, NavigateToPose, action_name)
            # Status topic published by action server: <action>/_action/status
            status_topic = f"{action_name}/_action/status"
            self.create_subscription(GoalStatusArray, status_topic, self._on_goal_status, 10)
            self.get_logger().info(f"Tracking Nav2 action status on {status_topic}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"Nav2 action tracking unavailable: {exc}; navigationStatus stays IDLE")

    def destroy_node(self) -> bool:
        self._mqtt.stop()
        return super().destroy_node()

    def _on_odom(self, msg: Odometry) -> None:
        self._lin_vel = float(msg.twist.twist.linear.x)
        self._ang_vel = float(msg.twist.twist.angular.z)

    def _on_amcl_pose(self, _msg: PoseWithCovarianceStamped) -> None:
        # Presence of AMCL pose is an assist signal only; TF remains SoT for pose.
        pass

    def _on_goal_status(self, msg) -> None:
        if not msg.status_list:
            return
        # Use the most recent status entry.
        latest = msg.status_list[-1]
        self._nav.update_from_goal_status(int(latest.status))

    def _heartbeat_topic(self) -> str:
        return f"{self._topic_prefix}/robots/{self._robot_code}/heartbeat"

    def _telemetry_topic(self) -> str:
        return f"{self._topic_prefix}/robots/{self._robot_code}/telemetry"

    def _on_heartbeat_timer(self) -> None:
        seq = self._seq.next_heartbeat()
        payload = build_heartbeat(
            robot_code=self._robot_code,
            boot_id=self._seq.boot_id,
            sequence=seq,
        )
        self._mqtt.publish(self._heartbeat_topic(), payload)

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

    def _on_telemetry_timer(self) -> None:
        pose, localization_status = self._lookup_map_pose()
        # Never publish a stale pose as valid when not localized.
        if localization_status != "LOCALIZED":
            pose = None

        seq = self._seq.next_telemetry()
        payload = build_telemetry(
            robot_code=self._robot_code,
            boot_id=self._seq.boot_id,
            sequence=seq,
            map_version_code=self._map_version_code,
            pose=pose,
            battery_percent=None,  # no battery sensor in current hardware stack
            navigation_status=self._nav.navigation_status,
            localization_status=localization_status,
            linear_velocity=self._lin_vel,
            angular_velocity=self._ang_vel,
            current_command_id=None,
            error_code=None,
        )
        self._mqtt.publish(self._telemetry_topic(), payload)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotGatewayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
