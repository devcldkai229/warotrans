"""WaroTrans simulation — Gazebo Harmonic world + simulated robot (ADR 0006).

Starts:
    gz sim (world)                     worlds/warehouse.sdf (generated from config/warehouse_layout.yaml)
    robot_state_publisher              urdf/warotrans_sim.urdf.xacro (real URDF + sim wheels/lidar/plugins)
    ros_gz_sim create                  spawns the robot at layout "spawn"
    ros_gz_bridge                      /clock /cmd_vel /odom /tf /scan /joint_states

Replaces on the real robot: esp32_bridge_node, wheel_odom_node, lidar driver.
Does NOT start AMCL / Nav2 / gateway — use sim_bringup.launch.py for the full stack.

USAGE
    ros2 launch warotrans_simulation simulation.launch.py            # with Gazebo GUI
    ros2 launch warotrans_simulation simulation.launch.py gui:=false # headless
"""

from __future__ import annotations

import os

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_yaml(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def robot_description_xml() -> str:
    """Expand the sim xacro with the measured geometry of the real robot (geometry.yaml)."""
    sim_pkg = get_package_share_directory("warotrans_simulation")
    desc_pkg = get_package_share_directory("warotrans_description")
    geometry_file = os.path.join(desc_pkg, "config", "geometry.yaml")
    geometry = load_yaml(geometry_file)
    laser = geometry["laser"]
    imu = geometry.get("imu") or {}
    mappings = {
        "geometry_file": geometry_file,
        "base_link_z": str(geometry["base_footprint_to_base_link"]["z"]),
        "lidar_x": str(laser["x"]),
        "lidar_y": str(laser["y"]),
        "lidar_z": str(laser["z"]),
        "lidar_yaw": str(laser["yaw"]),
        "enable_ultrasonic_front_rear": "false",
        # Same imu_link pose as the real robot (BNO055 mount, geometry.yaml imu.*).
        "enable_imu": "true",
        **{f"imu_{k}": str(imu.get(k) or 0.0) for k in ("x", "y", "z", "roll", "pitch", "yaw")},
    }
    doc = xacro.process_file(os.path.join(sim_pkg, "urdf", "warotrans_sim.urdf.xacro"), mappings=mappings)
    return doc.toxml()


def _setup(context: LaunchContext, *args, **kwargs):
    del args, kwargs
    sim_pkg = get_package_share_directory("warotrans_simulation")
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() in ("true", "1", "yes")
    layout = load_yaml(LaunchConfiguration("layout").perform(context))
    spawn = layout["spawn"]

    gz_args = f"-r {'' if gui else '-s '}{world}"
    robot_description = robot_description_xml()
    flag = lambda name: LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")  # noqa: E731
    enable_ekf = flag("enable_ekf")
    # enable_ekf: /odom = drifting wheel odometry, no TF from Gazebo (the EKF owns odom->base_footprint).
    bridge_config = os.path.join(sim_pkg, "config", "ros_gz_bridge_ekf.yaml" if enable_ekf else "ros_gz_bridge.yaml")
    # motor_deadband > 0: Nav2 / teleop /cmd_vel goes through motor_deadband_emulator, which feeds
    # Gazebo on /cmd_vel_motor (reproduces the real skid-steer that cannot turn at low PWM).
    deadband = float(LaunchConfiguration("motor_deadband").perform(context))
    bridge_remaps = [("/cmd_vel", "/cmd_vel_motor")] if deadband > 0.0 else []
    extra_nodes = []
    if deadband > 0.0:
        extra_nodes.append(
            Node(
                package="warotrans_simulation",
                executable="motor_deadband_emulator.py",
                name="motor_deadband_emulator",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "deadband_mps": deadband,
                        "compensate": flag("deadband_compensation"),
                        "wheel_separation": 0.4535,
                    }
                ],
            )
        )

    return extra_nodes + [
        LogInfo(
            msg=f"[warotrans_simulation] world={world} gui={gui} spawn={spawn} ekf={enable_ekf} "
            f"motor_deadband={deadband}"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
            ),
            launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": True}],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_warotrans",
            output="screen",
            arguments=[
                "-topic", "robot_description",
                "-name", "warotrans",
                "-x", str(spawn["x"]),
                "-y", str(spawn["y"]),
                "-z", "0.02",
                "-Y", str(spawn["yaw"]),
            ],
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge",
            output="screen",
            parameters=[
                {"config_file": bridge_config},
                {"use_sim_time": True},
            ],
            remappings=bridge_remaps,
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    sim_pkg = get_package_share_directory("warotrans_simulation")
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=os.path.join(sim_pkg, "worlds", "warehouse.sdf")),
            DeclareLaunchArgument("layout", default_value=os.path.join(sim_pkg, "config", "warehouse_layout.yaml")),
            DeclareLaunchArgument("gui", default_value="true", description="Show the Gazebo window"),
            DeclareLaunchArgument(
                "enable_ekf",
                default_value="false",
                description="Bridge wheel odometry as /odom without TF (EKF + IMU own odom->base_footprint)",
            ),
            DeclareLaunchArgument(
                "motor_deadband",
                default_value="0.0",
                description="Wheel speed (m/s) below which the simulated motors stall; 0 = ideal motors",
            ),
            DeclareLaunchArgument(
                "deadband_compensation",
                default_value="false",
                description="Emulate firmware min_start_duty: slow wheels are raised to the deadband",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
