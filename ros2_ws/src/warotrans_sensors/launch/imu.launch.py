"""Standalone launch for GY-BNO055 → /imu/data.

Include from bringup when enable_ekf:=true (with EKF TF ownership).
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    default_params = str(
        Path(get_package_share_directory("warotrans_sensors"))
        / "config"
        / "bno055.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML with BNO055 I2C and publish parameters",
            ),
            Node(
                package="warotrans_sensors",
                executable="bno055_node",
                name="bno055_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
