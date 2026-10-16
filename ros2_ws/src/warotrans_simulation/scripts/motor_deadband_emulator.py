#!/usr/bin/env python3
"""Emulate the real WaroTrans motor deadband in Gazebo (simulation ONLY).

The real skid-steer stalls when a wheel is commanded below the static-friction PWM
(open-loop firmware, no min_start_duty): slow in-place turns and the inner wheel of a
curve do not move. Gazebo's DiffDrive turns at any speed, so the T-junction bug never
showed in simulation. This node sits between Nav2/teleop and Gazebo:

    /cmd_vel  ->  mix to wheel speeds  ->  deadband  ->  remix  ->  /cmd_vel_motor (Gazebo)

compensate:=false   wheels with |v| < deadband_mps are stopped        (old firmware)
compensate:=true    wheels with 0 < |v| < deadband_mps run at deadband (firmware min_start_duty)

Same mixing as warotrans_hardware/motor_mixer.py (v_l = v - w*s/2, v_r = v + w*s/2).
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

WHEEL_MOVING_EPSILON_MPS = 0.005


def apply_deadband(wheel_speed: float, deadband: float, compensate: bool) -> float:
    magnitude = abs(wheel_speed)
    if magnitude <= WHEEL_MOVING_EPSILON_MPS:
        return 0.0
    if magnitude >= deadband:
        return wheel_speed
    return math.copysign(deadband, wheel_speed) if compensate else 0.0


class MotorDeadbandEmulator(Node):
    def __init__(self) -> None:
        super().__init__("motor_deadband_emulator")
        self._deadband = float(self.declare_parameter("deadband_mps", 0.08).value)
        self._compensate = bool(self.declare_parameter("compensate", False).value)
        self._separation = float(self.declare_parameter("wheel_separation", 0.4535).value)
        self._publisher = self.create_publisher(Twist, "/cmd_vel_motor", 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.get_logger().info(
            f"motor deadband {self._deadband:.3f} m/s, compensate={self._compensate}, "
            f"separation={self._separation:.4f} m: /cmd_vel -> /cmd_vel_motor"
        )

    def _on_cmd_vel(self, message: Twist) -> None:
        half = self._separation / 2.0
        left = message.linear.x - message.angular.z * half
        right = message.linear.x + message.angular.z * half
        left = apply_deadband(left, self._deadband, self._compensate)
        right = apply_deadband(right, self._deadband, self._compensate)
        out = Twist()
        out.linear.x = (left + right) / 2.0
        out.angular.z = (right - left) / self._separation
        self._publisher.publish(out)


def main() -> None:
    rclpy.init()
    node = MotorDeadbandEmulator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
