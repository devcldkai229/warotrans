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
    publish_tf = LaunchConfiguration("publish_tf")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    required_cmd_vel_publisher = LaunchConfiguration("required_cmd_vel_publisher")
    motors_enabled = LaunchConfiguration("motors_enabled")
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
            DeclareLaunchArgument(
                "cmd_vel_topic",
                default_value="/cmd_vel",
                description="Velocity topic driving the motors (authority gate on: /cmd_vel_motor)",
            ),
            DeclareLaunchArgument(
                "required_cmd_vel_publisher",
                default_value="",
                description=(
                    "Node that alone may publish cmd_vel_topic (authority gate on: cmd_vel_authority_gate); "
                    "any other publisher makes the bridge send zero"
                ),
            ),
            DeclareLaunchArgument(
                "motors_enabled",
                default_value="true",
                description="false: encoders/odometry only; the bridge never writes a velocity to the ESP32",
            ),
            DeclareLaunchArgument(
                "publish_tf",
                default_value="true",
                description=(
                    "wheel_odom publish TF odom→base_footprint "
                    "(set false when EKF owns that edge)"
                ),
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
                        "cmd_vel_topic": cmd_vel_topic,
                        "required_cmd_vel_publisher": required_cmd_vel_publisher,
                        "motors_enabled": ParameterValue(motors_enabled, value_type=bool),
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
                parameters=[
                    hardware_config,
                    {
                        "publish_tf": ParameterValue(
                            publish_tf,
                            value_type=bool,
                        ),
                    },
                ],
            ),
        ]
    )
