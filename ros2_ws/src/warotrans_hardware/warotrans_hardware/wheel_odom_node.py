"""Wheel odometry node for cumulative ESP32 encoder ticks."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from warotrans_msgs.msg import WheelTicks


class WheelOdomNode(Node):
    """Integrate differential-drive wheel distances into planar odometry."""

    def __init__(self) -> None:
        super().__init__("wheel_odom_node")

        self.declare_parameter("ticks_per_rev_left", 0.0)
        self.declare_parameter("ticks_per_rev_right", 0.0)
        self.declare_parameter("wheel_radius_left", 0.0)
        self.declare_parameter("wheel_radius_right", 0.0)
        self.declare_parameter("wheel_separation", 0.0)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        # When EKF owns odom→base_footprint, set publish_tf:=false (topic /odom stays).
        self.declare_parameter("publish_tf", True)
        # Diagonal covariances (x, y, z, roll, pitch, yaw) for pose / twist.
        # Zero covariance makes robot_localization over-trust the wheels.
        self.declare_parameter(
            "pose_covariance_diagonal", [0.01, 0.01, 1e6, 1e6, 1e6, 0.05]
        )
        self.declare_parameter(
            "twist_covariance_diagonal", [0.01, 0.001, 1e6, 1e6, 1e6, 0.1]
        )

        self._ticks_per_rev_left = float(
            self.get_parameter("ticks_per_rev_left").value
        )
        self._ticks_per_rev_right = float(
            self.get_parameter("ticks_per_rev_right").value
        )
        self._wheel_radius_left = float(
            self.get_parameter("wheel_radius_left").value
        )
        self._wheel_radius_right = float(
            self.get_parameter("wheel_radius_right").value
        )
        self._wheel_separation = float(
            self.get_parameter("wheel_separation").value
        )
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)
        self._pose_cov = self._covariance_matrix("pose_covariance_diagonal")
        self._twist_cov = self._covariance_matrix("twist_covariance_diagonal")
        self._validate_parameters()

        self._odom_publisher = self.create_publisher(Odometry, "/odom", 10)
        self._tf_broadcaster = (
            TransformBroadcaster(self) if self._publish_tf else None
        )
        if not self._publish_tf:
            self.get_logger().info(
                "publish_tf=false — /odom topic only; EKF must own "
                f"{self._odom_frame} → {self._base_frame}"
            )
        self._subscription = self.create_subscription(
            WheelTicks,
            "/wheel_ticks",
            self._on_wheel_ticks,
            10,
        )

        self._last_ticks: Optional[tuple[int, int]] = None
        self._last_mcu_millis: Optional[int] = None
        self._last_stamp = None
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0

    def _covariance_matrix(self, name: str) -> list[float]:
        diagonal = [float(v) for v in self.get_parameter(name).value]
        if len(diagonal) != 6 or any(
            not math.isfinite(v) or v < 0.0 for v in diagonal
        ):
            raise ValueError(f"{name} must be 6 finite non-negative values")
        matrix = [0.0] * 36
        for i, value in enumerate(diagonal):
            matrix[i * 7] = value
        return matrix

    def _validate_parameters(self) -> None:
        physical_values = {
            "ticks_per_rev_left": self._ticks_per_rev_left,
            "ticks_per_rev_right": self._ticks_per_rev_right,
            "wheel_radius_left": self._wheel_radius_left,
            "wheel_radius_right": self._wheel_radius_right,
            "wheel_separation": self._wheel_separation,
        }
        invalid = [
            name
            for name, value in physical_values.items()
            if not math.isfinite(value) or value <= 0.0
        ]
        if invalid:
            joined = ", ".join(invalid)
            raise ValueError(
                f"Measured odometry parameters required; invalid: {joined}"
            )

        if not self._odom_frame:
            raise ValueError("odom_frame must not be empty")
        if self._base_frame != "base_footprint":
            raise ValueError(
                "base_frame must be base_footprint; "
                "wheel odometry must not publish base_link"
            )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _on_wheel_ticks(self, message: WheelTicks) -> None:
        now = self.get_clock().now()
        current_ticks = (int(message.left_ticks), int(message.right_ticks))

        if self._last_ticks is None:
            self._last_ticks = current_ticks
            self._last_mcu_millis = int(message.mcu_millis)
            self._last_stamp = now
            self._publish_odometry(now, 0.0, 0.0)
            return

        # A backwards MCU clock normally means the ESP32 restarted. Rebaseline
        # instead of integrating the tick reset as a large robot movement.
        if self._last_mcu_millis is not None:
            mcu_delta = (
                int(message.mcu_millis) - self._last_mcu_millis
            ) & 0xFFFFFFFF
            if mcu_delta > 0x80000000:
                self.get_logger().warning(
                    "MCU timestamp moved backwards; resetting odometry baseline"
                )
                self._last_ticks = current_ticks
                self._last_mcu_millis = int(message.mcu_millis)
                self._last_stamp = now
                self._publish_odometry(now, 0.0, 0.0)
                return

        previous_left, previous_right = self._last_ticks
        delta_left_ticks = current_ticks[0] - previous_left
        delta_right_ticks = current_ticks[1] - previous_right

        distance_per_left_tick = (
            2.0 * math.pi * self._wheel_radius_left
            / self._ticks_per_rev_left
        )
        distance_per_right_tick = (
            2.0 * math.pi * self._wheel_radius_right
            / self._ticks_per_rev_right
        )
        d_left = delta_left_ticks * distance_per_left_tick
        d_right = delta_right_ticks * distance_per_right_tick
        d_s = (d_right + d_left) / 2.0
        d_theta = (d_right - d_left) / self._wheel_separation

        midpoint_theta = self._theta + d_theta / 2.0
        self._x += d_s * math.cos(midpoint_theta)
        self._y += d_s * math.sin(midpoint_theta)
        self._theta = self._normalize_angle(self._theta + d_theta)

        dt = 0.0
        if self._last_stamp is not None:
            dt = (now.nanoseconds - self._last_stamp.nanoseconds) / 1e9
        linear_velocity = d_s / dt if dt > 0.0 else 0.0
        angular_velocity = d_theta / dt if dt > 0.0 else 0.0

        self._last_ticks = current_ticks
        self._last_mcu_millis = int(message.mcu_millis)
        self._last_stamp = now
        self._publish_odometry(
            now,
            linear_velocity,
            angular_velocity,
        )

    def _publish_odometry(
        self,
        stamp,
        linear_velocity: float,
        angular_velocity: float,
    ) -> None:
        stamp_message = stamp.to_msg()

        odometry = Odometry()
        odometry.header.stamp = stamp_message
        odometry.header.frame_id = self._odom_frame
        odometry.child_frame_id = self._base_frame
        odometry.pose.pose.position.x = self._x
        odometry.pose.pose.position.y = self._y
        odometry.pose.pose.position.z = 0.0
        odometry.pose.pose.orientation.z = math.sin(self._theta / 2.0)
        odometry.pose.pose.orientation.w = math.cos(self._theta / 2.0)
        odometry.twist.twist.linear.x = linear_velocity
        odometry.twist.twist.angular.z = angular_velocity
        odometry.pose.covariance = self._pose_cov
        odometry.twist.covariance = self._twist_cov
        self._odom_publisher.publish(odometry)

        if self._tf_broadcaster is None:
            return

        transform = TransformStamped()
        transform.header.stamp = stamp_message
        transform.header.frame_id = self._odom_frame
        transform.child_frame_id = self._base_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.translation.z = 0.0
        transform.transform.rotation.z = math.sin(self._theta / 2.0)
        transform.transform.rotation.w = math.cos(self._theta / 2.0)
        self._tf_broadcaster.sendTransform(transform)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[WheelOdomNode] = None
    try:
        node = WheelOdomNode()
        rclpy.spin(node)
    except (ValueError, KeyboardInterrupt) as exc:
        if node is None:
            print(f"wheel_odom_node configuration error: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
