#!/usr/bin/env python3
"""Dual-robot standalone NavigateToPose scenarios under semantic_active (no Jobs)."""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
import urllib.request
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


class NsNav(Node):
    def __init__(self, ns: str) -> None:
        # Remap /tf → namespaced tf (multi_sim publishes /sim0N/tf, not /tf).
        super().__init__(
            f"dual_nav_{ns}",
            namespace=ns,
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"],
        )
        self.ns = ns
        # Action under namespace: /sim01/navigate_to_pose
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tfl = TransformListener(self.tf, self, spin_thread=True)
        self.map_frame = "map"
        self.base_frame = f"{ns}/base_footprint"

    def pose(self) -> tuple[float, float] | None:
        try:
            t = self.tf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:  # noqa: BLE001
            return None

    def navigate(self, x: float, y: float, yaw: float, timeout: float) -> tuple[str, int]:
        if not self.ac.wait_for_server(timeout_sec=20.0):
            return "NO_SERVER", -1
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.map_frame
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
            try:
                send.result().cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            return "TIMEOUT", -1
        wrap = res_fut.result()
        status = int(wrap.status)
        code = int(getattr(wrap.result, "error_code", 0))
        if status == 4:
            return "PASS", code
        return f"FAIL_status_{status}", code


def fetch_json(url: str) -> dict | None:
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            return json.loads(r.read().decode("utf-8"))
    except Exception:  # noqa: BLE001
        return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoints", required=True)
    ap.add_argument("--out-csv", required=True)
    ap.add_argument("--out-md", required=True)
    ap.add_argument("--api", default="")
    ap.add_argument("--timeout", type=float, default=180.0)
    args = ap.parse_args()
    eps = {e["code"]: e for e in json.loads(Path(args.endpoints).read_text(encoding="utf-8"))["endpoints"]}

    rclpy.init()
    sim01 = NsNav("sim01")
    sim02 = NsNav("sim02")
    # warm TF / clock (spin_thread on TransformListener; still pump executors)
    t0 = time.time()
    while time.time() - t0 < 15.0:
        rclpy.spin_once(sim01, timeout_sec=0.05)
        rclpy.spin_once(sim02, timeout_sec=0.05)
        if sim01.pose() is not None and sim02.pose() is not None:
            break
    print(
        f"TF_WARM sim01={sim01.pose()} sim02={sim02.pose()}",
        flush=True,
    )

    rows: list[dict] = []
    md_lines = [
        "# Dual-robot results (Stage 3)",
        "",
        f"**Updated:** {time.strftime('%Y-%m-%d %H:%M:%S')}",
        "**Mode:** semantic_active · no stage1_open_grant · no Transport Jobs",
        "",
    ]

    def snap(tag: str) -> None:
        if not args.api:
            return
        data = fetch_json(f"{args.api.rstrip('/')}/api/fleet/traffic/snapshot")
        path = Path(args.out_csv).parent / "runtime" / f"traffic_snap_{tag}.json"
        path.parent.mkdir(parents=True, exist_ok=True)
        if data is not None:
            path.write_text(json.dumps(data, indent=2)[:50000], encoding="utf-8")
            md_lines.append(f"- snapshot `{tag}`: saved `{path.name}`")
        else:
            md_lines.append(f"- snapshot `{tag}`: FAILED fetch")

    scenarios = [
        # 1) Independent non-conflicting: SIM-01 → WAIT, SIM-02 → INBOUND (approx separate)
        {
            "id": "independent_paths",
            "legs": [
                ("SIM-01", "sim01", "EP-WAIT-POINT"),
                ("SIM-02", "sim02", "EP-INBOUND"),
            ],
            "parallel": True,
        },
        # 2) Return hubs then shelf conflict zone approach
        {
            "id": "sim01_to_outbound",
            "legs": [("SIM-01", "sim01", "EP-OUTBOUND")],
            "parallel": False,
        },
        {
            "id": "sim02_to_outbound_area",
            "legs": [("SIM-02", "sim02", "EP-WAIT-POINT")],
            "parallel": False,
        },
    ]

    snap("before")

    for sc in scenarios:
        md_lines.append(f"## Scenario `{sc['id']}`")
        md_lines.append("")
        if sc.get("parallel") and len(sc["legs"]) == 2:
            (r1, ns1, c1), (r2, ns2, c2) = sc["legs"]
            e1, e2 = eps[c1], eps[c2]
            n1 = sim01 if ns1 == "sim01" else sim02
            n2 = sim02 if ns2 == "sim02" else sim01
            # launch both
            # simplistic parallel: alternate spin
            # Sequential NavigateToPose (rclpy single-threaded); still covers both robots.
            for robot, ns, code in sc["legs"]:
                node = sim01 if ns == "sim01" else sim02
                ep = eps[code]
                start = node.pose()
                status, err = node.navigate(ep["x"], ep["y"], float(ep.get("yaw") or 0.0), args.timeout)
                end = node.pose()
                err_xy = None if end is None else math.hypot(end[0] - ep["x"], end[1] - ep["y"])
                row = {
                    "scenario": sc["id"],
                    "robot": robot,
                    "goal": code,
                    "status": status,
                    "error_code": err,
                    "start_xy": "" if start is None else f"{start[0]:.3f},{start[1]:.3f}",
                    "end_xy": "" if end is None else f"{end[0]:.3f},{end[1]:.3f}",
                    "goal_error_m": "" if err_xy is None else f"{err_xy:.3f}",
                }
                rows.append(row)
                print(row, flush=True)
                md_lines.append(f"- {robot} → {code}: **{status}** err={err} goal_error={row['goal_error_m']}")
        else:
            for robot, ns, code in sc["legs"]:
                node = sim01 if ns == "sim01" else sim02
                ep = eps[code]
                start = node.pose()
                status, err = node.navigate(ep["x"], ep["y"], float(ep.get("yaw") or 0.0), args.timeout)
                end = node.pose()
                err_xy = None if end is None else math.hypot(end[0] - ep["x"], end[1] - ep["y"])
                row = {
                    "scenario": sc["id"],
                    "robot": robot,
                    "goal": code,
                    "status": status,
                    "error_code": err,
                    "start_xy": "" if start is None else f"{start[0]:.3f},{start[1]:.3f}",
                    "end_xy": "" if end is None else f"{end[0]:.3f},{end[1]:.3f}",
                    "goal_error_m": "" if err_xy is None else f"{err_xy:.3f}",
                }
                rows.append(row)
                print(row, flush=True)
                md_lines.append(f"- {robot} → {code}: **{status}** err={err} goal_error={row['goal_error_m']}")
        snap(sc["id"])
        md_lines.append("")

    snap("after")
    passed = sum(1 for r in rows if r["status"] == "PASS")
    md_lines.extend(
        [
            "## Summary",
            "",
            f"- legs: {len(rows)}",
            f"- pass: {passed}",
            f"- fail/timeout: {len(rows) - passed}",
            "",
            "CSV: `DUAL_MOTION_RESULTS.csv`",
            "",
        ]
    )

    out_csv = Path(args.out_csv)
    out_csv.parent.mkdir(parents=True, exist_ok=True)
    with out_csv.open("w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=["scenario", "robot", "goal", "status", "error_code", "start_xy", "end_xy", "goal_error_m"],
        )
        w.writeheader()
        w.writerows(rows)
    Path(args.out_md).write_text("\n".join(md_lines) + "\n", encoding="utf-8")
    print(json.dumps({"legs": len(rows), "pass": passed, "csv": str(out_csv)}))
    sim01.destroy_node()
    sim02.destroy_node()
    rclpy.shutdown()
    return 0 if passed == len(rows) and rows else 1


if __name__ == "__main__":
    raise SystemExit(main())
