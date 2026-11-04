#!/usr/bin/env python3
"""Synchronized Stage-3 motion trace: grants, authority_ok, cmd_vel, pose, planRevision."""

from __future__ import annotations

import argparse
import csv
import json
import math
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def yaw_from_q(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class RobotTrace:
    def __init__(self, ns: str) -> None:
        self.ns = ns
        self.lock = threading.Lock()
        self.authority_ok: bool | None = None
        self.authority_ok_t: float | None = None
        self.cmd_vel = (0.0, 0.0)
        self.cmd_vel_safe = (0.0, 0.0)
        self.plan_n = 0
        self.plan_rev_hint = 0
        self.pose: tuple[float, float, float] | None = None
        self.rows: list[dict] = []

    def sample(self, wall: float, sim_t: float) -> dict:
        with self.lock:
            return {
                "wall_t": f"{wall:.3f}",
                "sim_t": f"{sim_t:.3f}",
                "ns": self.ns,
                "authority_ok": self.authority_ok,
                "authority_ok_age": (
                    None
                    if self.authority_ok_t is None
                    else round(time.monotonic() - self.authority_ok_t, 3)
                ),
                "cmd_vx": round(self.cmd_vel[0], 4),
                "cmd_wz": round(self.cmd_vel[1], 4),
                "safe_vx": round(self.cmd_vel_safe[0], 4),
                "safe_wz": round(self.cmd_vel_safe[1], 4),
                "plan_n": self.plan_n,
                "pose_x": None if self.pose is None else round(self.pose[0], 4),
                "pose_y": None if self.pose is None else round(self.pose[1], 4),
                "pose_yaw": None if self.pose is None else round(self.pose[2], 4),
            }


class SyncTraceNode(Node):
    def __init__(self, namespaces: list[str], out_csv: Path, hz: float) -> None:
        super().__init__(
            "sync_motion_trace",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            cli_args=["--ros-args", "-r", "/tf:=/sim01/tf", "-r", "/tf_static:=/sim01/tf_static"],
        )
        self.out_csv = out_csv
        self.robots = {ns: RobotTrace(ns) for ns in namespaces}
        self.tf = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tfl = TransformListener(self.tf, self, spin_thread=True)
        for ns in namespaces:
            rt = self.robots[ns]
            self.create_subscription(
                Bool, f"/{ns}/traffic/authority_ok", self._mk_auth_cb(rt), 10
            )
            self.create_subscription(Twist, f"/{ns}/cmd_vel", self._mk_cmd_cb(rt, False), 10)
            self.create_subscription(
                Twist, f"/{ns}/cmd_vel_safe", self._mk_cmd_cb(rt, True), 10
            )
            self.create_subscription(NavPath, f"/{ns}/plan", self._mk_plan_cb(rt), 10)
        self.create_timer(1.0 / max(1.0, hz), self._on_tick)
        self._wall0 = time.time()
        self.out_csv.parent.mkdir(parents=True, exist_ok=True)
        self._fieldnames = [
            "wall_t",
            "sim_t",
            "ns",
            "authority_ok",
            "authority_ok_age",
            "cmd_vx",
            "cmd_wz",
            "safe_vx",
            "safe_wz",
            "plan_n",
            "pose_x",
            "pose_y",
            "pose_yaw",
        ]
        with self.out_csv.open("w", newline="", encoding="utf-8") as f:
            csv.DictWriter(f, fieldnames=self._fieldnames).writeheader()
        self.get_logger().info(f"sync_motion_trace → {out_csv} ns={namespaces} hz={hz}")

    def _mk_auth_cb(self, rt: RobotTrace):
        def _cb(msg: Bool) -> None:
            with rt.lock:
                rt.authority_ok = bool(msg.data)
                rt.authority_ok_t = time.monotonic()

        return _cb

    def _mk_cmd_cb(self, rt: RobotTrace, safe: bool):
        def _cb(msg: Twist) -> None:
            with rt.lock:
                pair = (float(msg.linear.x), float(msg.angular.z))
                if safe:
                    rt.cmd_vel_safe = pair
                else:
                    rt.cmd_vel = pair

        return _cb

    def _mk_plan_cb(self, rt: RobotTrace):
        def _cb(msg: NavPath) -> None:
            with rt.lock:
                rt.plan_n = len(msg.poses)
                rt.plan_rev_hint += 1

        return _cb

    def _lookup_pose(self, ns: str) -> tuple[float, float, float] | None:
        try:
            # Prefer namespaced TF trees.
            for prefix in (f"/{ns}", ""):
                try:
                    t = self.tf.lookup_transform(
                        "map", f"{ns}/base_footprint", rclpy.time.Time()
                    )
                    tr = t.transform.translation
                    yaw = yaw_from_q(t.transform.rotation)
                    return float(tr.x), float(tr.y), float(yaw)
                except Exception:  # noqa: BLE001
                    continue
            return None
        except Exception:  # noqa: BLE001
            return None

    def _on_tick(self) -> None:
        wall = time.time() - self._wall0
        sim_t = self.get_clock().now().nanoseconds / 1e9
        rows = []
        for ns, rt in self.robots.items():
            pose = self._lookup_pose(ns)
            with rt.lock:
                rt.pose = pose
            rows.append(rt.sample(wall, sim_t))
        with self.out_csv.open("a", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=self._fieldnames)
            for row in rows:
                w.writerow(row)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--ns", default="sim01,sim02")
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--duration", type=float, default=0.0, help="0 = run until killed")
    args = ap.parse_args()
    ns_list = [x.strip() for x in args.ns.split(",") if x.strip()]
    rclpy.init()
    node = SyncTraceNode(ns_list, Path(args.out), args.hz)
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if args.duration > 0 and time.time() - t0 >= args.duration:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    meta = Path(args.out).with_suffix(".meta.json")
    meta.write_text(
        json.dumps({"ns": ns_list, "hz": args.hz, "out": str(args.out)}, indent=2),
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
