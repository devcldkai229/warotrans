"""Standalone launch for four MKE-S01 ultrasonics on Pi GPIO.

Do not include in full robot bringup until electrical/software validation PASS.
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
        / "ultrasonic.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML with ultrasonic GPIO and timing parameters",
            ),
            Node(
                package="warotrans_sensors",
                executable="ultrasonic_node",
                name="ultrasonic_node",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
