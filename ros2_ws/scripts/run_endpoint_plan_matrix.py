#!/usr/bin/env python3
"""Live ComputePathToPose matrix for all directed endpoint pairs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter


def yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Matrix(Node):
    def __init__(self) -> None:
        super().__init__(
            "endpoint_plan_matrix",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.ac = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")

    def plan(self, sx, sy, syaw, gx, gy, gyaw, timeout: float) -> tuple[str, int, float]:
        if not self.ac.wait_for_server(timeout_sec=10.0):
            return "NO_SERVER", 0, 0.0
        goal = ComputePathToPose.Goal()
        goal.planner_id = "GridBased"
        goal.use_start = True
        goal.start = PoseStamped()
        goal.start.header.frame_id = "map"
        goal.start.header.stamp = self.get_clock().now().to_msg()
        goal.start.pose.position.x = sx
        goal.start.pose.position.y = sy
        _, _, z, w = yaw_quat(syaw)
        goal.start.pose.orientation.z = z
        goal.start.pose.orientation.w = w
        goal.goal = PoseStamped()
        goal.goal.header = goal.start.header
        goal.goal.pose.position.x = gx
        goal.goal.pose.position.y = gy
        _, _, z, w = yaw_quat(gyaw)
        goal.goal.pose.orientation.z = z
        goal.goal.pose.orientation.w = w
        t0 = time.time()
        fut = self.ac.send_goal_async(goal)
        while not fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not fut.done() or fut.result() is None or not fut.result().accepted:
            return "REJECTED", 0, time.time() - t0
        res_fut = fut.result().get_result_async()
        while not res_fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        dt = time.time() - t0
        if not res_fut.done():
            return "TIMEOUT", 0, dt
        result = res_fut.result().result
        n = len(result.path.poses)
        if n >= 2:
            return "PASS", n, dt
        return "EMPTY", 0, dt


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoints", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args()
    eps = json.loads(Path(args.endpoints).read_text(encoding="utf-8"))["endpoints"]
    rclpy.init()
    node = Matrix()
    rows = []
    ok = fail = 0
    try:
        for a in eps:
            for b in eps:
                if a["code"] == b["code"]:
                    continue
                status, n, dt = node.plan(
                    a["x"], a["y"], float(a.get("yaw") or 0.0),
                    b["x"], b["y"], float(b.get("yaw") or 0.0),
                    args.timeout,
                )
                cat = "VALID" if status == "PASS" else "UNEXPECTED_FAIL"
                if status == "PASS":
                    ok += 1
                else:
                    fail += 1
                rows.append(
                    {
                        "src": a["code"],
                        "dst": b["code"],
                        "status": status,
                        "category": cat,
                        "poses": n,
                        "latency_s": f"{dt:.3f}",
                    }
                )
                print(f"{a['code']}->{b['code']} {status} n={n} t={dt:.2f}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["src", "dst", "status", "category", "poses", "latency_s"])
        w.writeheader()
        w.writerows(rows)
    print(json.dumps({"pairs": len(rows), "pass": ok, "fail": fail, "out": str(out)}))
    return 0 if fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
