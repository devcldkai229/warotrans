"""Launch robot_localization EKF for odom + IMU yaw fusion.

TF OWNERSHIP
    When this launch runs, ekf_filter_node owns odom → base_footprint.
    Pass publish_tf:=false to warotrans_hardware/hardware.launch.py.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg = get_package_share_directory("warotrans_localization")
    default_params = os.path.join(pkg, "config", "ekf.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="EKF parameter YAML",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
            ),
        ]
    )
