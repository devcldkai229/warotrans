#!/usr/bin/env python3
"""Sample cmd_vel pipeline + authority while a short NavigateToPose runs."""

from __future__ import annotations

import json
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class Probe(Node):
    def __init__(self) -> None:
        super().__init__(
            "motion_cmd_vel_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.samples = {k: [] for k in ("cmd_vel", "cmd_vel_safe", "cmd_vel_nav", "cmd_vel_smoothed", "authority_ok")}
        self.create_subscription(Twist, "/cmd_vel", lambda m: self._t("cmd_vel", m), 20)
        self.create_subscription(Twist, "/cmd_vel_safe", lambda m: self._t("cmd_vel_safe", m), 20)
        self.create_subscription(Twist, "/cmd_vel_nav", lambda m: self._t("cmd_vel_nav", m), 20)
        self.create_subscription(Twist, "/cmd_vel_smoothed", lambda m: self._t("cmd_vel_smoothed", m), 20)
        self.create_subscription(Bool, "/traffic/authority_ok", lambda m: self.samples["authority_ok"].append(bool(m.data)), 20)
        self._ac = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def _t(self, key: str, msg: Twist) -> None:
        self.samples[key].append((msg.linear.x, msg.angular.z))


def main() -> int:
    rclpy.init()
    node = Probe()
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.05)
    print("pre_authority", node.samples["authority_ok"][-5:], "n", len(node.samples["authority_ok"]))

    if not node._ac.wait_for_server(timeout_sec=10.0):
        print("no navigate_to_pose server")
        return 1
    goal = NavigateToPose.Goal()
    goal.pose = PoseStamped()
    goal.pose.header.frame_id = "map"
    goal.pose.header.stamp = node.get_clock().now().to_msg()
    # short goal near parking toward center
    goal.pose.pose.position.x = -1.02
    goal.pose.pose.position.y = -0.529
    z = math.sin(-math.pi / 4)
    w = math.cos(-math.pi / 4)
    goal.pose.pose.orientation.z = z
    goal.pose.pose.orientation.w = w
    send = node._ac.send_goal_async(goal)
    t0 = time.time()
    while not send.done() and time.time() - t0 < 10:
        rclpy.spin_once(node, timeout_sec=0.05)
    gh = send.result()
    print("goal_accepted", gh.accepted if gh else None)
    t0 = time.time()
    while time.time() - t0 < 25.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    def summarize(key: str):
        vals = node.samples[key]
        if not vals:
            return {"n": 0}
        if key == "authority_ok":
            return {"n": len(vals), "last": vals[-1], "true_frac": sum(1 for v in vals if v) / len(vals)}
        lx = [v[0] for v in vals]
        az = [v[1] for v in vals]
        return {
            "n": len(vals),
            "lx_max": max(lx),
            "lx_absmax": max(abs(x) for x in lx),
            "az_absmax": max(abs(x) for x in az),
            "nonzero": sum(1 for x, a in vals if abs(x) > 1e-4 or abs(a) > 1e-4),
        }

    out = {k: summarize(k) for k in node.samples}
    Path("/tmp/nav2_live_artifacts").mkdir(parents=True, exist_ok=True)
    Path("/tmp/nav2_live_artifacts/motion_cmd_vel_probe.json").write_text(json.dumps(out, indent=2) + "\n")
    print(json.dumps(out, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
