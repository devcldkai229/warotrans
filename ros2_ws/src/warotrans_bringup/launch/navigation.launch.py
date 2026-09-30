"""Launch the full WaroTrans navigation stack (Phase 9 — Nav2).

Thành phần:

    description   -> robot_state_publisher, TF tĩnh
    hardware      -> /wheel_ticks, /odom, TF odom -> base_footprint
    lidar         -> /scan
    localization  -> map_server + AMCL, TF map -> odom
    navigation    -> Nav2 (planner, controller, BT navigator, costmaps)
    foxglove      -> ws://<robot>:8765 cho laptop (optional)

KHÔNG BAO GỒM:
    - teleop: xung đột /cmd_vel với Nav2 controller
    - slam: xung đột map -> odom với AMCL

TF ownership khi navigation:
    map -> odom:           AMCL
    odom -> base_footprint: wheel_odom_node
    static geometry:        robot_state_publisher (URDF)

CẢNH BÁO: không chạy đồng thời với mapping.launch.py, demo.launch.py,
hoặc service warotrans-mapping (trùng esp32_bridge_node /dev/warotrans
và xung đột TF ownership).

USAGE
    ros2 launch warotrans_bringup navigation.launch.py map:=$HOME/maps/warotrans.yaml

    Sau đó trên WSL2/laptop chạy RViz:
    rviz2 -d ~/ros2_ws/src/warotrans_navigation/rviz/nav2_warotrans.rviz

    Dùng "2D Pose Estimate" đặt pose ban đầu, sau đó "Nav2 Goal" để điều hướng.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _subsystem_launch(package: str, launch_file: str):
    return PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])


def generate_launch_description() -> LaunchDescription:
    # Launch arguments
    map_file = LaunchConfiguration("map")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    foxglove_port = LaunchConfiguration("foxglove_port")
    autostart = LaunchConfiguration("autostart")

    # Default map path
    default_map = os.path.expanduser("~/maps/warotrans.yaml")

    navigation_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            _subsystem_launch("warotrans_navigation", "navigation.launch.py")
        ),
        launch_arguments={
            "autostart": autostart,
            "nav2_params_file": LaunchConfiguration("nav2_params_file"),
            "enable_costmap_filters": LaunchConfiguration("enable_costmap_filters"),
            "keepout_mask": LaunchConfiguration("keepout_mask"),
            "speed_mask": LaunchConfiguration("speed_mask"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_navigation")),
    )

    return LaunchDescription(
        [
            # --- Launch arguments ---
            DeclareLaunchArgument(
                "map",
                default_value=default_map,
                description="Full path to map YAML file",
            ),
            DeclareLaunchArgument(
                "start_hardware",
                default_value="true",
                description="Start ESP32 bridge and wheel odometry",
            ),
            DeclareLaunchArgument(
                "start_lidar",
                default_value="true",
                description="Start the RPLIDAR driver",
            ),
            DeclareLaunchArgument(
                "start_localization",
                default_value="true",
                description="Start map_server and AMCL",
            ),
            DeclareLaunchArgument(
                "start_navigation",
                default_value="true",
                description="Start Nav2 stack",
            ),
            DeclareLaunchArgument(
                "start_foxglove",
                default_value="true",
                description="Start foxglove_bridge for remote visualization",
            ),
            DeclareLaunchArgument(
                "lidar_serial_port",
                default_value="",
                description="Override the LiDAR serial device (udev symlink)",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="8765",
                description="foxglove_bridge websocket port",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Auto-transition Nav2 lifecycle nodes to active",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(
                    get_package_share_directory("warotrans_navigation"),
                    "config",
                    "nav2_params.yaml",
                ),
                description="Base Nav2 params YAML",
            ),
            DeclareLaunchArgument(
                "enable_costmap_filters",
                default_value="false",
                description="Phase 3 keepout/speed filters (off for outdoor Nav2)",
            ),
            DeclareLaunchArgument(
                "keepout_mask",
                default_value=os.path.expanduser("~/maps/warotrans_keepout.yaml"),
                description="Phase 3 KEEP_OUT filter mask YAML (when filters on)",
            ),
            DeclareLaunchArgument(
                "speed_mask",
                default_value=os.path.expanduser("~/maps/warotrans_speed.yaml"),
                description="Phase 3 SPEED_LIMIT filter mask YAML (when filters on)",
            ),
            DeclareLaunchArgument(
                "start_ultrasonic",
                default_value="false",
                description=(
                    "Start ultrasonic_node (Range topics). "
                    "Nav2 Collision Monitor still OFF until FRONT/REAR mounts measured "
                    "— see docs/ultrasonic-front-rear-nav2.md"
                ),
            ),
            DeclareLaunchArgument(
                "nav_start_delay_sec",
                default_value="12.0",
                description="Delay Nav2 start so /scan and AMCL map->odom exist first",
            ),

            # --- robot_state_publisher: always required for TF ---
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _subsystem_launch("warotrans_description", "description.launch.py")
                )
            ),

            # --- Hardware: ESP32 bridge + wheel odometry ---
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _subsystem_launch("warotrans_hardware", "hardware.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("start_hardware")),
            ),

            # --- LiDAR ---
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _subsystem_launch("warotrans_sensors", "lidar.launch.py")
                ),
                launch_arguments={"serial_port": lidar_serial_port}.items(),
                condition=IfCondition(LaunchConfiguration("start_lidar")),
            ),

            # --- Ultrasonic (monitoring; FRONT/REAR for future Nav2 safety) ---
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _subsystem_launch("warotrans_sensors", "ultrasonic.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("start_ultrasonic")),
            ),

            # --- Localization: map_server + AMCL ---
            # AMCL owns map -> odom transform
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _subsystem_launch("warotrans_localization", "localization.launch.py")
                ),
                launch_arguments={
                    "map": map_file,
                    "autostart": autostart,
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_localization")),
            ),

            # --- Navigation: delayed so LiDAR + AMCL TF are ready ---
            # Planner activate needs map->odom; AMCL publishes it after /scan.
            TimerAction(
                period=12.0,
                actions=[navigation_include],
            ),

            # --- Foxglove bridge: remote visualization ---
            Node(
                package="foxglove_bridge",
                executable="foxglove_bridge",
                name="foxglove_bridge",
                output="screen",
                parameters=[
                    {
                        "address": "0.0.0.0",
                        "port": ParameterValue(foxglove_port, value_type=int),
                        "use_sim_time": False,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("start_foxglove")),
            ),
        ]
    )
