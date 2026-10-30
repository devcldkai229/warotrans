#!/usr/bin/env python3
"""Record ~20 Hz Nav2 / TF / Gazebo evidence for traffic-hardening Phase B.

Writes CSV under --out-dir (durable path; not /tmp).
Topics: /cmd_vel_nav, /cmd_vel_smoothed (if present), /cmd_vel,
TF map->base_link and map->odom (via tf2), optional Gazebo GT pose topic.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import time
from dataclasses import dataclass, field

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class Sample:
    t: float
    cmd_nav_vx: float = float("nan")
    cmd_nav_wz: float = float("nan")
    cmd_sm_vx: float = float("nan")
    cmd_sm_wz: float = float("nan")
    cmd_vx: float = float("nan")
    cmd_wz: float = float("nan")
    bl_x: float = float("nan")
    bl_y: float = float("nan")
    bl_yaw: float = float("nan")
    odom_x: float = float("nan")
    odom_y: float = float("nan")
    odom_yaw: float = float("nan")
    gt_x: float = float("nan")
    gt_y: float = float("nan")
    gt_yaw: float = float("nan")


def yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


class Recorder(Node):
    def __init__(self, duration_s: float, hz: float, gt_topic: str) -> None:
        super().__init__("phase_b_corridor_recorder")
        self.duration_s = duration_s
        self.period = 1.0 / hz
        self.t0 = time.monotonic()
        self.latest: Sample = Sample(t=0.0)
        self.rows: list[Sample] = field(default_factory=list)  # type: ignore[misc]
        self.rows = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Twist, "/cmd_vel_nav", self._on_nav, 20)
        self.create_subscription(Twist, "/cmd_vel_smoothed", self._on_sm, 20)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 20)
        if gt_topic:
            self.create_subscription(Odometry, gt_topic, self._on_gt, 20)
        self.create_timer(self.period, self._tick)

    def _on_nav(self, msg: Twist) -> None:
        self.latest.cmd_nav_vx = msg.linear.x
        self.latest.cmd_nav_wz = msg.angular.z

    def _on_sm(self, msg: Twist) -> None:
        self.latest.cmd_sm_vx = msg.linear.x
        self.latest.cmd_sm_wz = msg.angular.z

    def _on_cmd(self, msg: Twist) -> None:
        self.latest.cmd_vx = msg.linear.x
        self.latest.cmd_wz = msg.angular.z

    def _on_gt(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.latest.gt_x = p.x
        self.latest.gt_y = p.y
        self.latest.gt_yaw = yaw_from_quat(q.z, q.w)

    def _lookup(self, parent: str, child: str) -> tuple[float, float, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
            t = tf.transform.translation
            q = tf.transform.rotation
            return t.x, t.y, yaw_from_quat(q.z, q.w)
        except TransformException:
            return None

    def _tick(self) -> None:
        now = time.monotonic() - self.t0
        if now > self.duration_s:
            raise SystemExit(0)
        row = Sample(t=now)
        row.cmd_nav_vx = self.latest.cmd_nav_vx
        row.cmd_nav_wz = self.latest.cmd_nav_wz
        row.cmd_sm_vx = self.latest.cmd_sm_vx
        row.cmd_sm_wz = self.latest.cmd_sm_wz
        row.cmd_vx = self.latest.cmd_vx
        row.cmd_wz = self.latest.cmd_wz
        row.gt_x = self.latest.gt_x
        row.gt_y = self.latest.gt_y
        row.gt_yaw = self.latest.gt_yaw
        bl = self._lookup("map", "base_link")
        if bl:
            row.bl_x, row.bl_y, row.bl_yaw = bl
        od = self._lookup("map", "odom")
        if od:
            row.odom_x, row.odom_y, row.odom_yaw = od
        self.rows.append(row)


def summarize(rows: list[Sample]) -> dict[str, float]:
    wz = [abs(r.cmd_vx) for r in rows if not math.isnan(r.cmd_vx)]
    ang = [abs(r.cmd_wz) for r in rows if not math.isnan(r.cmd_wz)]
    # lateral deviation vs GT when both present
    lat = []
    for r in rows:
        if math.isnan(r.bl_x) or math.isnan(r.gt_x):
            continue
        lat.append(math.hypot(r.bl_x - r.gt_x, r.bl_y - r.gt_y))
    def rms(xs: list[float]) -> float:
        if not xs:
            return float("nan")
        return math.sqrt(sum(x * x for x in xs) / len(xs))
    return {
        "n": float(len(rows)),
        "cmd_vx_rms": rms(wz),
        "cmd_wz_peak": max(ang) if ang else float("nan"),
        "cmd_wz_rms": rms(ang),
        "amcl_vs_gt_rms_m": rms(lat),
        "amcl_vs_gt_peak_m": max(lat) if lat else float("nan"),
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", required=True)
    ap.add_argument("--duration", type=float, default=40.0)
    ap.add_argument("--hz", type=float, default=20.0)
    ap.add_argument("--gt-topic", default="/gazebo/ground_truth/odom")
    args = ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    rclpy.init()
    node = Recorder(args.duration, args.hz, args.gt_topic)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    rows = node.rows
    node.destroy_node()
    rclpy.shutdown()

    csv_path = os.path.join(args.out_dir, "corridor_20hz.csv")
    fields = [f.name for f in Sample.__dataclass_fields__.values()]  # type: ignore[attr-defined]
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(Sample.__dataclass_fields__.keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r.__dict__)

    stats = summarize(rows)
    summary_path = os.path.join(args.out_dir, "corridor_summary.txt")
    with open(summary_path, "w", encoding="utf-8") as f:
        for k, v in stats.items():
            f.write(f"{k}={v}\n")
    print(f"wrote {csv_path} n={len(rows)}")
    for k, v in stats.items():
        print(f"{k}={v}")


if __name__ == "__main__":
    main()
