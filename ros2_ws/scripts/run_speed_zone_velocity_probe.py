#!/usr/bin/env python3
"""Log /odom linear speed while NavigateToPose crosses SPEED mask cells (SIM evidence)."""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener


def yaw_quat(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Probe(Node):
    def __init__(self) -> None:
        super().__init__(
            "speed_zone_velocity_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.ac = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)
        self._v = 0.0
        self._mask = None
        self._samples: list[dict] = []
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, "/speed_filter_mask", self._on_mask, qos)

    def _on_odom(self, msg: Odometry) -> None:
        self._v = float(msg.twist.twist.linear.x)

    def _on_mask(self, msg: OccupancyGrid) -> None:
        self._mask = msg

    def _mask_at(self, x: float, y: float) -> int | None:
        m = self._mask
        if m is None:
            return None
        info = m.info
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return int(m.data[my * info.width + mx])

    def pose(self) -> tuple[float, float] | None:
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:  # noqa: BLE001
            return None

    def navigate(self, x: float, y: float, yaw: float, timeout: float) -> str:
        if not self.ac.wait_for_server(timeout_sec=15.0):
            return "NO_SERVER"
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        z, w = yaw_quat(yaw)
        goal.pose.pose.orientation.z = z
        goal.pose.pose.orientation.w = w
        t0 = time.time()
        send = self.ac.send_goal_async(goal)
        while not send.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is not None:
                self._samples.append(
                    {
                        "t_wall": f"{time.time() - t0:.2f}",
                        "x": f"{p[0]:.3f}",
                        "y": f"{p[1]:.3f}",
                        "v_odom": f"{self._v:.4f}",
                        "speed_mask": "" if self._mask_at(*p) is None else str(self._mask_at(*p)),
                    }
                )
        if not send.done() or send.result() is None or not send.result().accepted:
            return "REJECTED"
        res_fut = send.result().get_result_async()
        while not res_fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.pose()
            if p is not None:
                self._samples.append(
                    {
                        "t_wall": f"{time.time() - t0:.2f}",
                        "x": f"{p[0]:.3f}",
                        "y": f"{p[1]:.3f}",
                        "v_odom": f"{self._v:.4f}",
                        "speed_mask": "" if self._mask_at(*p) is None else str(self._mask_at(*p)),
                    }
                )
        if not res_fut.done():
            return "TIMEOUT"
        return "PASS" if int(res_fut.result().status) == 4 else "FAIL"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args()
    rclpy.init()
    node = Probe()
    t0 = time.time()
    while time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    # OUTBOUND -> WAIT crosses SPEED mask cells (mask≈30 → 0.30 m/s absolute).
    status = node.navigate(1.292, 0.83, 0.0, args.timeout)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["t_wall", "x", "y", "v_odom", "speed_mask"])
        w.writeheader()
        w.writerows(node._samples)
    in_zone = [s for s in node._samples if s["speed_mask"] not in ("", "0", "-1")]
    vmax = max((abs(float(s["v_odom"])) for s in in_zone), default=0.0)
    print(
        {
            "status": status,
            "samples": len(node._samples),
            "in_speed_zone": len(in_zone),
            "max_abs_v_in_zone": round(vmax, 4),
            "out": str(out),
        }
    )
    node.destroy_node()
    rclpy.shutdown()
    return 0 if status == "PASS" and in_zone else 1


if __name__ == "__main__":
    raise SystemExit(main())
