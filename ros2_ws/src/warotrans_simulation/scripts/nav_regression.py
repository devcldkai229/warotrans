#!/usr/bin/env python3
"""NavigateToPose regression on the demo warehouse (T-junctions, other side of a rack, obstacle, unreachable).

Sends the scenarios one after another (each starts where the previous ended) and prints a table:
result, duration, recoveries, final error vs goal (map -> base_footprint TF).

    ros2 run warotrans_simulation nav_regression.py                 # all scenarios
    ros2 run warotrans_simulation nav_regression.py --only straight left_t
    ros2 run warotrans_simulation nav_regression.py --no-obstacle   # e.g. on the real robot

The obstacle scenario spawns a box in Gazebo (gz service) — simulation only.
"""

from __future__ import annotations

import argparse
import math
import subprocess
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

# name, x, y, yaw, expect_success, what it exercises
SCENARIOS = [
    ("straight", 5.75, 2.0, math.pi / 2, True, "straight 1.1 m north from PARK"),
    ("left_t", 4.875, 3.75, 0.0, True, "T-junction, turn LEFT into aisle A|B (Rack B dock)"),
    ("right_t", 6.625, 3.75, 0.0, True, "out of aisle, turn RIGHT into aisle B|C (Rack C dock)"),
    ("other_side", 5.75, 6.75, math.pi / 2, True, "around the racks to the charger (other side)"),
    ("obstacle", 5.75, 0.9, math.pi / 2, True, "back to PARK with an unmapped box on the route"),
    ("unreachable", 5.75, 3.75, 0.0, False, "goal inside Rack B -> must ABORT, not hang"),
]
OBSTACLE = (5.75, 5.75)  # between the charger and the racks, on the way south
STATUS = {
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
}


def spawn_box(world: str, x: float, y: float) -> bool:
    sdf = (
        "<sdf version='1.9'><model name='regression_box'><static>true</static><pose>"
        f"{x} {y} 0.25 0 0 0</pose><link name='l'><collision name='c'><geometry><box><size>0.4 0.4 0.5</size>"
        "</box></geometry></collision><visual name='v'><geometry><box><size>0.4 0.4 0.5</size></box></geometry>"
        "<material><diffuse>0.8 0.2 0.2 1</diffuse></material></visual></link></model></sdf>"
    )
    req = 'sdf: "' + sdf.replace('"', '\\"') + '"'
    cmd = ["gz", "service", "-s", f"/world/{world}/create", "--reqtype", "gz.msgs.EntityFactory",
           "--reptype", "gz.msgs.Boolean", "--timeout", "3000", "--req", req]
    out = subprocess.run(cmd, capture_output=True, text=True, check=False)
    return "true" in out.stdout


class Regression(Node):
    def __init__(self) -> None:
        super().__init__("nav_regression")
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def pose(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf_buffer.lookup_transform("map", "base_footprint", rclpy.time.Time())
        except Exception:  # noqa: BLE001
            return None
        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def run(self, x: float, y: float, yaw: float, timeout: float) -> tuple[str, float, int]:
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        recoveries = [0]

        def on_feedback(fb) -> None:
            recoveries[0] = fb.feedback.number_of_recoveries

        start = time.monotonic()
        send = self.client.send_goal_async(goal, feedback_callback=on_feedback)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return "REJECTED", time.monotonic() - start, 0
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=timeout)
        if not result.done():
            handle.cancel_goal_async()
            return "TIMEOUT(hang)", time.monotonic() - start, recoveries[0]
        return STATUS.get(result.result().status, str(result.result().status)), time.monotonic() - start, recoveries[0]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--only", nargs="*", default=None)
    parser.add_argument("--no-obstacle", action="store_true")
    parser.add_argument("--timeout", type=float, default=300.0, help="wall seconds per goal before 'hang'")
    parser.add_argument("--world", default="sim_warehouse")
    args = parser.parse_args()

    rclpy.init()
    node = Regression()
    if not node.client.wait_for_server(timeout_sec=60.0):
        print("navigate_to_pose action server not available", file=sys.stderr)
        return 2
    rows = []
    failures = 0
    for name, x, y, yaw, expect_ok, what in SCENARIOS:
        if args.only and name not in args.only:
            continue
        if name == "obstacle":
            if args.no_obstacle:
                continue
            print(f"spawn box at {OBSTACLE}: {spawn_box(args.world, *OBSTACLE)}", flush=True)
        print(f"--> {name}: ({x}, {y}, {yaw:.2f}) {what}", flush=True)
        status, duration, recoveries = node.run(x, y, yaw, args.timeout)
        for _ in range(10):  # let TF catch up
            rclpy.spin_once(node, timeout_sec=0.1)
        pose = node.pose()
        err = math.hypot(pose[0] - x, pose[1] - y) if pose else float("nan")
        ok = (status == "SUCCEEDED") if expect_ok else (status == "ABORTED" and duration <= 180.0)
        failures += 0 if ok else 1
        rows.append((name, status, duration, recoveries, err, "PASS" if ok else "FAIL"))
        print(f"<-- {name}: {status} {duration:.0f}s recoveries={recoveries} err={err:.2f}m", flush=True)

    print("\nscenario      result          time  recov  err[m]  verdict")
    for name, status, duration, recoveries, err, verdict in rows:
        print(f"{name:<13} {status:<15} {duration:5.0f}s {recoveries:5d}  {err:6.2f}  {verdict}")
    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
