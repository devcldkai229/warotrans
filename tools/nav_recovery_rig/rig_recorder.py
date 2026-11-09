#!/usr/bin/env python3
"""Record /plan, /cmd_vel_motor, /odom (and optional last_block_reason) to a JSONL trace."""

from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, path: str) -> None:
        super().__init__("rig_recorder")
        self._fp = open(path, "a", encoding="utf-8", buffering=1)
        self.create_subscription(Path, "/plan", self._on_plan, 10)
        self.create_subscription(Twist, "/cmd_vel_motor", self._on_cmd, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(String, "/traffic/last_block_reason", self._on_reason, 10)

    def _write(self, kind: str, body: dict) -> None:
        self._fp.write(json.dumps({"wall": round(time.time(), 3), "t": kind, **body}) + "\n")

    def _on_plan(self, msg: Path) -> None:
        poses = [
            {
                "x": round(float(p.pose.position.x), 4),
                "y": round(float(p.pose.position.y), 4),
                "yaw": round(_yaw(p.pose.orientation), 4),
            }
            for p in msg.poses
        ]
        self._write("plan", {"n": len(poses), "poses": poses})

    def _on_cmd(self, msg: Twist) -> None:
        self._write("cmd", {"v": round(float(msg.linear.x), 4), "w": round(float(msg.angular.z), 4)})

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._write(
            "pose",
            {
                "x": round(float(p.x), 4),
                "y": round(float(p.y), 4),
                "yaw": round(_yaw(msg.pose.pose.orientation), 4),
            },
        )

    def _on_reason(self, msg: String) -> None:
        self._write("block_reason", {"reason": msg.data})


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit("usage: rig_recorder.py <trace.jsonl>")
    rclpy.init()
    node = Recorder(sys.argv[1])
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
