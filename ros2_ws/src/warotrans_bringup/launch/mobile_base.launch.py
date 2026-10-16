"""Launch WaroTrans mobile base commissioning stack.

CẢNH BÁO: không chạy đồng thời với demo.launch.py / mapping.launch.py
hoặc service warotrans-demo (trùng esp32_bridge_node).

enable_ekf:=true → BNO055 + EKF owns TF odom→base_footprint;
wheel_odom publish_tf:=false.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_ekf = LaunchConfiguration("enable_ekf")
    publish_tf = PythonExpression(
        ["'false' if '", enable_ekf, "' == 'true' else 'true'"]
    )

    description_launch = PathJoinSubstitution(
        [FindPackageShare("warotrans_description"), "launch", "description.launch.py"]
    )
    hardware_launch = PathJoinSubstitution(
        [FindPackageShare("warotrans_hardware"), "launch", "hardware.launch.py"]
    )
    teleop_launch = PathJoinSubstitution(
        [FindPackageShare("warotrans_teleop"), "launch", "teleop.launch.py"]
    )
    imu_launch = PathJoinSubstitution(
        [FindPackageShare("warotrans_sensors"), "launch", "imu.launch.py"]
    )
    ekf_launch = PathJoinSubstitution(
        [FindPackageShare("warotrans_localization"), "launch", "ekf.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_ekf",
                default_value="false",
                description=(
                    "Start BNO055 + EKF; EKF owns odom→base_footprint "
                    "(wheel publish_tf off)"
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(description_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(hardware_launch),
                launch_arguments={"publish_tf": publish_tf}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(imu_launch),
                condition=IfCondition(enable_ekf),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ekf_launch),
                condition=IfCondition(enable_ekf),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(teleop_launch)
            ),
        ]
    )
