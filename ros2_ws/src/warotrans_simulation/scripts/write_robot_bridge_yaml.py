#!/usr/bin/env python3
"""Write a ros_gz_bridge YAML for one namespaced Stage-3 sim robot."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--ros-ns", required=True, help="ROS namespace without leading slash, e.g. sim01")
    p.add_argument("--gz-prefix", required=True, help="Gazebo topic prefix, e.g. sim01/")
    p.add_argument("--out", required=True, type=Path)
    args = p.parse_args()
    ns = args.ros_ns.strip("/")
    gp = args.gz_prefix
    if gp and not gp.endswith("/"):
        gp += "/"

    # Shared /clock once per world — dual launch uses a separate clock-only bridge.
    entries = [
        ("cmd_vel", f"/{ns}/cmd_vel", f"{gp}cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "ROS_TO_GZ"),
        ("odom", f"/{ns}/odom", f"{gp}odom", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
        ("tf", f"/{ns}/tf", f"{gp}tf", "tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V", "GZ_TO_ROS"),
        ("scan", f"/{ns}/scan", f"{gp}scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan", "GZ_TO_ROS"),
        ("imu", f"/{ns}/imu/data", f"{gp}imu/data", "sensor_msgs/msg/Imu", "gz.msgs.IMU", "GZ_TO_ROS"),
        (
            "js",
            f"/{ns}/joint_states",
            f"{gp}joint_states",
            "sensor_msgs/msg/JointState",
            "gz.msgs.Model",
            "GZ_TO_ROS",
        ),
    ]
    lines = ["# GENERATED — do not edit by hand\n"]
    for _, ros, gz, rt, gt, direction in entries:
        lines.append(f"- ros_topic_name: \"{ros}\"\n")
        lines.append(f"  gz_topic_name: \"{gz}\"\n")
        lines.append(f"  ros_type_name: \"{rt}\"\n")
        lines.append(f"  gz_type_name: \"{gt}\"\n")
        lines.append(f"  direction: {direction}\n")
        lines.append("\n")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text("".join(lines), encoding="utf-8")
    print(args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
