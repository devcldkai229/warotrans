"""WaroTrans localization launch — Map Server + AMCL.

Minimal launch for Phase 0: map_server + amcl + lifecycle_manager.

USAGE
    ros2 launch warotrans_localization localization.launch.py map:=/path/to/map.yaml

TF OWNERSHIP
    When this launch is running, AMCL owns map -> odom.
    Do NOT run slam_toolbox simultaneously (it also publishes map -> odom).

ARGUMENTS
    map           — Full path to map YAML file (required)
    use_sim_time  — Use simulation clock (default: false)
    autostart     — Auto-transition lifecycle nodes (default: true)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    pkg_localization = get_package_share_directory("warotrans_localization")

    # Launch configurations
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    # Named amcl_params_file to avoid colliding with lidar/slam params_file
    # in the same bringup LaunchContext.
    amcl_params_file = LaunchConfiguration("amcl_params_file")

    # Default params file
    default_params = os.path.join(pkg_localization, "config", "amcl.yaml")

    # Lifecycle nodes to manage
    lifecycle_nodes = ["map_server", "amcl"]

    # Remap TF topics
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    return LaunchDescription(
        [
            # --- Launch arguments ---
            DeclareLaunchArgument(
                "map",
                description="Full path to map YAML file",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Auto-transition lifecycle nodes to active",
            ),
            DeclareLaunchArgument(
                "amcl_params_file",
                default_value=default_params,
                description="AMCL/map_server params YAML",
            ),
            # --- Map Server ---
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    amcl_params_file,
                    {"use_sim_time": use_sim_time},
                    {"yaml_filename": map_yaml_file},
                ],
                remappings=remappings,
            ),
            # --- AMCL ---
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    amcl_params_file,
                    {"use_sim_time": use_sim_time},
                ],
                remappings=remappings,
            ),
            # --- Lifecycle Manager ---
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"autostart": autostart},
                    {"node_names": lifecycle_nodes},
                ],
            ),
        ]
    )
