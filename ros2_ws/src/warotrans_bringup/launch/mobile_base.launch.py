"""Launch WaroTrans mobile base commissioning stack.

CẢNH BÁO: không chạy đồng thời với demo.launch.py / mapping.launch.py
hoặc service warotrans-demo (trùng esp32_bridge_node).
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    description_launch = PathJoinSubstitution(
        [FindPackageShare("warotrans_description"), "launch", "description.launch.py"]
    )
    hardware_launch = PathJoinSubstitution(
        [FindPackageShare("warotrans_hardware"), "launch", "hardware.launch.py"]
    )
    teleop_launch = PathJoinSubstitution(
        [FindPackageShare("warotrans_teleop"), "launch", "teleop.launch.py"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(hardware_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_launch)
            ),
        ]
    )
