#!/usr/bin/env python3
"""Tell AMCL where the robot stands: publish /initialpose at a map endpoint of the semantic bundle's snapshot.

Run on the robot, ONLY after a person has placed it at that endpoint; it publishes a pose estimate and nothing else
(no goal, no velocity). The gateway reports LOCALIZED only after an operator pose + a converged AMCL.

    python3 set_initial_pose.py --x 1.244 --y -0.997 --yaw 1.571 --confirm-placed
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--x", type=float, required=True)
    ap.add_argument("--y", type=float, required=True)
    ap.add_argument("--yaw", type=float, required=True)
    ap.add_argument("--xy-std", type=float, default=0.15)
    ap.add_argument("--yaw-std", type=float, default=0.2)
    ap.add_argument("--confirm-placed", action="store_true", help="a person confirms the robot stands at this pose")
    args = ap.parse_args()
    if not args.confirm_placed:
        print("Refusing: pass --confirm-placed once the robot physically stands at this pose.", file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("set_initial_pose")
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pub = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = args.x
    msg.pose.pose.position.y = args.y
    msg.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
    msg.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
    msg.pose.covariance[0] = msg.pose.covariance[7] = args.xy_std ** 2
    msg.pose.covariance[35] = args.yaw_std ** 2
    # AMCL itself must be matched: other subscribers (the gateway) discover this publisher earlier, and a volatile
    # subscriber never receives what was published before it matched.
    deadline = time.monotonic() + 15.0
    amcl = False
    while not amcl and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        amcl = any(i.node_name == "amcl" for i in node.get_subscriptions_info_by_topic("/initialpose"))
    if amcl:
        time.sleep(1.0)  # matched in the graph; give the data path a moment
        msg.header.stamp = node.get_clock().now().to_msg()
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.5)
    subs = pub.get_subscription_count()
    node.destroy_node()
    rclpy.shutdown()
    print(f"/initialpose ({args.x}, {args.y}, {args.yaw}) amcl_matched={amcl} subscribers={subs}")
    return 0 if amcl else 1


if __name__ == "__main__":
    raise SystemExit(main())
