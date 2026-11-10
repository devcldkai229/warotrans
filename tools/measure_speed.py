#!/usr/bin/env python3
"""Compare /odom linear.x to /cmd_vel_motor for lab feedforward tuning.

Usage (on Pi or WSL with ROS):
  ros2 run ... or: python3 tools/measure_speed.py --seconds 20

Prints mean |odom.v| / |cmd.v| when |cmd| > threshold, plus peak odom speed.
"""

from __future__ import annotations

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class SpeedProbe(Node):
    def __init__(self) -> None:
        super().__init__("measure_speed")
        self.cmd_v = 0.0
        self.odom_v = 0.0
        self.samples: list[tuple[float, float]] = []
        self.create_subscription(Twist, "/cmd_vel_motor", self._on_cmd, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos_profile_sensor_data)

    def _on_cmd(self, msg: Twist) -> None:
        self.cmd_v = float(msg.linear.x)

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_v = float(msg.twist.twist.linear.x)
        self.samples.append((self.cmd_v, self.odom_v))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--cmd-min", type=float, default=0.02, help="ignore samples with |cmd| below this")
    args = ap.parse_args()
    rclpy.init()
    node = SpeedProbe()
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    pairs = [(c, o) for c, o in node.samples if abs(c) >= args.cmd_min]
    if not pairs:
        print("no samples with |cmd| >= threshold — drive the robot while measuring")
        return 1
    ratios = [o / c for c, o in pairs if abs(c) > 1e-6]
    peak = max(abs(o) for _, o in node.samples) if node.samples else 0.0
    mean_r = sum(ratios) / len(ratios) if ratios else float("nan")
    print(f"samples={len(node.samples)} driven={len(pairs)} mean_odom/cmd={mean_r:.3f} peak_odom={peak:.3f} m/s")
    print("If mean_odom/cmd >> 1, open-loop motors run faster than commanded — tune firmware feedforward.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
