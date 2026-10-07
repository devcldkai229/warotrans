"""Launch the WaroTrans Robot Gateway (ROS2 → MQTT heartbeat/telemetry)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("warotrans_fleet")
    default_params = os.path.join(pkg_share, "config", "robot_gateway.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("robot_code", default_value=""),
            DeclareLaunchArgument("mqtt_host", default_value=""),
            DeclareLaunchArgument("mqtt_port", default_value=""),
            DeclareLaunchArgument("map_version_code", default_value=""),
            Node(
                package="warotrans_fleet",
                executable="robot_gateway_node",
                name="robot_gateway",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    # Empty launch args are ignored by ROS when not overridden via env;
                    # prefer env WARO_* or edit YAML for production.
                ],
            ),
        ]
    )
