#!/usr/bin/env python3
"""SIM-01 NavigateToPose tour across selected endpoints (continues from actual pose)."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener


def yaw_quat(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Tour(Node):
    def __init__(self) -> None:
        super().__init__(
            "sim01_motion_tour",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.ac = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.tf = Buffer()
        self._tfl = TransformListener(self.tf, self)

    def pose(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:  # noqa: BLE001
            return None

    def navigate(self, x: float, y: float, yaw: float, timeout: float) -> tuple[str, int]:
        if not self.ac.wait_for_server(timeout_sec=15.0):
            return "NO_SERVER", -1
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
        if not send.done() or send.result() is None or not send.result().accepted:
            return "REJECTED", -1
        res_fut = send.result().get_result_async()
        while not res_fut.done() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not res_fut.done():
            # Cancel stuck goal so later legs can run.
            try:
                send.result().cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            return "TIMEOUT", -1
        wrap = res_fut.result()
        status = int(wrap.status)
        code = int(getattr(wrap.result, "error_code", 0))
        if status == 4:  # STATUS_SUCCEEDED
            return "PASS", code
        return f"FAIL_status_{status}", code


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoints", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--timeout", type=float, default=150.0)
    ap.add_argument("--codes", default="")
    args = ap.parse_args()
    eps = {e["code"]: e for e in json.loads(Path(args.endpoints).read_text(encoding="utf-8"))["endpoints"]}
    codes = [c.strip() for c in args.codes.split(",") if c.strip()] or list(eps.keys())
    rclpy.init()
    node = Tour()
    rows = []
    # warm TF
    t0 = time.time()
    while time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    try:
        for code in codes:
            e = eps[code]
            start = node.pose()
            status, err = node.navigate(e["x"], e["y"], float(e.get("yaw") or 0.0), args.timeout)
            end = node.pose()
            err_xy = None
            if end is not None:
                err_xy = math.hypot(end[0] - e["x"], end[1] - e["y"])
            rows.append(
                {
                    "robot": "SIM-01",
                    "goal": code,
                    "status": status,
                    "error_code": err,
                    "start_xy": "" if start is None else f"{start[0]:.3f},{start[1]:.3f}",
                    "end_xy": "" if end is None else f"{end[0]:.3f},{end[1]:.3f}",
                    "goal_error_m": "" if err_xy is None else f"{err_xy:.3f}",
                }
            )
            print(rows[-1], flush=True)
            # Continue remaining legs for coverage evidence; orchestrator scores PASS count.
    finally:
        node.destroy_node()
        rclpy.shutdown()
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=["robot", "goal", "status", "error_code", "start_xy", "end_xy", "goal_error_m"],
        )
        w.writeheader()
        w.writerows(rows)
    passed = sum(1 for r in rows if r["status"] == "PASS")
    print(json.dumps({"legs": len(rows), "pass": passed, "out": str(out)}))
    return 0 if passed == len(rows) and rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
