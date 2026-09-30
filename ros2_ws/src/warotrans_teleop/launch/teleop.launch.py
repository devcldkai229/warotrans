"""Launch the WaroTrans mobile web teleop node."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    teleop_config = PathJoinSubstitution(
        [FindPackageShare("warotrans_teleop"), "config", "teleop.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="warotrans_teleop",
                executable="web_teleop_node",
                name="web_teleop_node",
                output="screen",
                parameters=[teleop_config],
            ),
        ]
    )
