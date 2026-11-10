#!/usr/bin/env python3
"""Ask the running Nav2 planner_server (LaneAwareNavfn) for real paths between map endpoints. Planning only: this calls
ComputePathToPose, never NavigateToPose, so nothing moves. Output is labelled source=nav2 (the fixture used by the
geometry checks; synthetic harness paths are straight lines and are never mixed with it).

    python3 generate_nav2_paths.py --endpoints endpoints.json --out real_nav2_paths.json
"""
import argparse
import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient


def pose(node, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--endpoints", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    eps = json.load(open(a.endpoints, encoding="utf-8"))
    rclpy.init()
    node = rclpy.create_node("generate_nav2_paths")
    client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not client.wait_for_server(timeout_sec=10):
        raise SystemExit("planner_server not available")
    results = []
    for s in eps:
        for g in eps:
            if s["code"] == g["code"]:
                continue
            goal = ComputePathToPose.Goal()
            goal.use_start = True
            goal.start = pose(node, s["x"], s["y"], s["yaw"])
            goal.goal = pose(node, g["x"], g["y"], g["yaw"])
            goal.planner_id = "GridBased"
            f = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, f, timeout_sec=5)
            h = f.result()
            entry = {"from": s["code"], "to": g["code"], "source": "nav2", "ok": False, "poses": []}
            if h is not None and h.accepted:
                rf = h.get_result_async()
                rclpy.spin_until_future_complete(node, rf, timeout_sec=10)
                r = rf.result()
                if r is not None and r.result.path.poses:
                    entry["ok"] = True
                    entry["poses"] = [
                        [round(p.pose.position.x, 4), round(p.pose.position.y, 4),
                         round(2 * math.atan2(p.pose.orientation.z, p.pose.orientation.w), 4)]
                        for p in r.result.path.poses]
                elif r is not None:
                    entry["error"] = getattr(r.result, "error_code", None)
            results.append(entry)
    json.dump({"generatedAt": time.strftime("%FT%TZ", time.gmtime()), "source": "nav2", "planner": "LaneAwareNavfn",
               "paths": results}, open(a.out, "w"))
    print("paths", len(results), "ok", sum(1 for r in results if r["ok"]))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
