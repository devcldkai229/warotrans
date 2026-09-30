"""Launch WaroTrans operator demo stack (boot autostart).

Thành phần (mỗi node chỉ một lần — không chạy kèm mobile_base/mapping):

    description  -> TF tĩnh
    hardware     -> /wheel_ticks, /odom, TF odom -> base_footprint
    teleop       -> /cmd_vel, web UI :8080
    lidar        -> /scan
    foxglove     -> ws://0.0.0.0:8765

Không gồm slam_toolbox (tránh CPU nặng và TF map->odom khi chỉ demo lái/viz).
Muốn map: dùng mapping.launch.py thủ công, tắt service demo trước.
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
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    foxglove_port = LaunchConfiguration("foxglove_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_hardware",
                default_value="true",
                description="ESP32 bridge + wheel odometry (/wheel_ticks, /odom)",
            ),
            DeclareLaunchArgument(
                "start_teleop",
                default_value="true",
                description="Web teleop on :8080",
            ),
            DeclareLaunchArgument(
                "start_lidar",
                default_value="true",
                description="RPLIDAR /scan",
            ),
            DeclareLaunchArgument(
                "start_foxglove",
                default_value="true",
                description="foxglove_bridge on :8765",
            ),
            DeclareLaunchArgument(
                "lidar_serial_port",
                default_value="",
                description="Override LiDAR serial device",
            ),
            DeclareLaunchArgument(
                "foxglove_port",
                default_value="8765",
                description="foxglove_bridge websocket port",
            ),
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
                    _subsystem_launch("warotrans_teleop", "teleop.launch.py")
                ),
                condition=IfCondition(LaunchConfiguration("start_teleop")),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    _subsystem_launch("warotrans_sensors", "lidar.launch.py")
                ),
                launch_arguments={"serial_port": lidar_serial_port}.items(),
                condition=IfCondition(LaunchConfiguration("start_lidar")),
            ),
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
