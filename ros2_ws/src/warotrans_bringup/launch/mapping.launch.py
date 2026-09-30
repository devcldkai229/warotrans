"""Launch the full WaroTrans mapping stack (Phase 8).

Thành phần:

    description  -> robot_state_publisher, TF tĩnh
    hardware     -> /wheel_ticks, /odom, TF odom -> base_footprint
    sensors      -> /scan
    teleop       -> /cmd_vel (web UI cổng 8080)
    slam         -> /map, TF map -> odom
    foxglove     -> ws://<robot>:8765 cho laptop

File này chỉ compose các launch của từng subsystem, không định nghĩa lại node.

CẢNH BÁO: không chạy đồng thời với mobile_base.launch.py, demo.launch.py,
hoặc service warotrans-demo (trùng esp32_bridge_node /dev/warotrans).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _subsystem_launch(package: str, launch_file: str):
    return PathJoinSubstitution([FindPackageShare(package), "launch", launch_file])


def generate_launch_description() -> LaunchDescription:
    slam_params_file = LaunchConfiguration("slam_params_file")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    foxglove_port = LaunchConfiguration("foxglove_port")

    default_slam_params = PathJoinSubstitution(
        [FindPackageShare("warotrans_slam"), "config", "slam_toolbox.yaml"]
    )

    return LaunchDescription(
        [
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
                "start_teleop",
                default_value="true",
                description="Start the mobile web teleop server",
            ),
            DeclareLaunchArgument(
                "start_slam",
                default_value="true",
                description="Start slam_toolbox asynchronous mapping",
            ),
            DeclareLaunchArgument(
                "start_foxglove",
                default_value="true",
                description="Start foxglove_bridge for remote visualization",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=default_slam_params,
                description="Override the slam_toolbox parameter file",
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
            # robot_state_publisher luôn chạy: mọi thứ còn lại phụ thuộc TF tĩnh.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _subsystem_launch("warotrans_description", "description.launch.py")
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _subsystem_launch("warotrans_hardware", "hardware.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("start_hardware")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _subsystem_launch("warotrans_sensors", "lidar.launch.py")
                ),
                launch_arguments={"serial_port": lidar_serial_port}.items(),
                condition=IfCondition(LaunchConfiguration("start_lidar")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _subsystem_launch("warotrans_teleop", "teleop.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("start_teleop")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _subsystem_launch("warotrans_slam", "slam.launch.py")
                ),
                launch_arguments={"params_file": slam_params_file}.items(),
                condition=IfCondition(LaunchConfiguration("start_slam")),
            ),
            # Pi chạy Ubuntu Server: visualization nằm trên laptop, Pi chỉ mở
            # websocket bridge chứ không cần desktop environment.
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
