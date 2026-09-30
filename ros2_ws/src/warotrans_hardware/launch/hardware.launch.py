"""Launch the ESP32 bridge and wheel odometry nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    hardware_config = PathJoinSubstitution(
        [
            FindPackageShare("warotrans_hardware"),
            "config",
            "hardware.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/warotrans",
                description="Stable ESP32 serial device path",
            ),
            DeclareLaunchArgument(
                "baud_rate",
                default_value="921600",
                description="ESP32 serial baud rate",
            ),
            Node(
                package="warotrans_hardware",
                executable="esp32_bridge_node",
                name="esp32_bridge_node",
                output="screen",
                parameters=[
                    hardware_config,
                    {
                        "serial_port": serial_port,
                        "baud_rate": ParameterValue(
                            baud_rate,
                            value_type=int,
                        ),
                    },
                ],
            ),
            Node(
                package="warotrans_hardware",
                executable="wheel_odom_node",
                name="wheel_odom_node",
                output="screen",
                parameters=[hardware_config],
            ),
        ]
    )
