#!/usr/bin/env python3
"""Cancel any active NavigateToPose goals (cleanup between acceptance legs)."""

from __future__ import annotations

import time

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node
from rclpy.parameter import Parameter


def main() -> int:
    rclpy.init()
    node = Node(
        "cancel_nav2_goals",
        parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
    )
    cli = node.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")
    if not cli.wait_for_service(timeout_sec=5.0):
        print("cancel_nav2_goals: service not ready")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    fut = cli.call_async(CancelGoal.Request())
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    print("cancel_nav2_goals: done", fut.done())
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
