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


def _resolve_spawn(layout_path: str, endpoint_code: str, map_yaml: str | None):
    import sys

    launch_dir = os.path.dirname(os.path.abspath(__file__))
    if launch_dir not in sys.path:
        sys.path.insert(0, launch_dir)
    from _spawn_util import resolve_layout_spawn

    return resolve_layout_spawn(layout_path, endpoint_code=endpoint_code or None, map_yaml=map_yaml)


def load_yaml(path: str) -> dict:
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def robot_description_xml(topic_prefix: str = "") -> str:
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
        "topic_prefix": topic_prefix,
        **{f"imu_{k}": str(imu.get(k) or 0.0) for k in ("x", "y", "z", "roll", "pitch", "yaw")},
    }
    doc = xacro.process_file(os.path.join(sim_pkg, "urdf", "warotrans_sim.urdf.xacro"), mappings=mappings)
    return doc.toxml()


def _setup(context: LaunchContext, *args, **kwargs):
    del args, kwargs
    sim_pkg = get_package_share_directory("warotrans_simulation")
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() in ("true", "1", "yes")
    layout_path = LaunchConfiguration("layout").perform(context)
    endpoint = LaunchConfiguration("spawn_endpoint_code").perform(context).strip()
    map_yaml = LaunchConfiguration("map").perform(context).strip() or None
    robot_name = LaunchConfiguration("robot_name").perform(context).strip() or "warotrans"
    topic_prefix = LaunchConfiguration("topic_prefix").perform(context).strip()
    bridge_config_override = LaunchConfiguration("bridge_config").perform(context).strip()
    namespace = LaunchConfiguration("robot_namespace").perform(context).strip()
    spawn = _resolve_spawn(layout_path, endpoint, map_yaml)
    skip_gazebo = LaunchConfiguration("skip_gazebo").perform(context).lower() in ("true", "1", "yes")

    gz_args = f"-r {'' if gui else '-s '}{world}"
    robot_description = robot_description_xml(topic_prefix=topic_prefix)
    flag = lambda name: LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")  # noqa: E731
    enable_ekf = flag("enable_ekf")
    # enable_ekf: /odom = drifting wheel odometry, no TF from Gazebo (the EKF owns odom->base_footprint).
    if bridge_config_override:
        bridge_config = bridge_config_override
    else:
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
                namespace=namespace or None,
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

    actions = extra_nodes + [
        LogInfo(
            msg=(
                f"[warotrans_simulation] world={world} gui={gui} skip_gazebo={skip_gazebo} "
                f"spawn=({spawn['x']:.4f},{spawn['y']:.4f},{spawn['yaw']:.4f}) "
                f"source={spawn.get('source')} endpoint={spawn.get('endpoint_code')} "
                f"robot={robot_name} ns={namespace or '-'} topic_prefix={topic_prefix!r} "
                f"ekf={enable_ekf} motor_deadband={deadband}"
            )
        ),
    ]
    if not skip_gazebo:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
                ),
                launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items(),
            )
        )
    actions.extend(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace=namespace or None,
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": True,
                        # Prefer explicit topic_prefix frames (sim01/) over ROS namespace alone.
                        "frame_prefix": topic_prefix if topic_prefix else (f"{namespace}/" if namespace else ""),
                    }
                ],
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                name=f"spawn_{robot_name}",
                output="screen",
                arguments=[
                    "-string",
                    robot_description,
                    "-name",
                    robot_name,
                    "-x",
                    str(spawn["x"]),
                    "-y",
                    str(spawn["y"]),
                    "-z",
                    "0.02",
                    "-Y",
                    str(spawn["yaw"]),
                ],
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="ros_gz_bridge",
                namespace=namespace or None,
                output="screen",
                parameters=[
                    {"config_file": bridge_config},
                    {"use_sim_time": True},
                ],
                remappings=bridge_remaps,
            ),
        ]
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    sim_pkg = get_package_share_directory("warotrans_simulation")
    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=os.path.join(sim_pkg, "worlds", "warehouse.sdf")),
            DeclareLaunchArgument("layout", default_value=os.path.join(sim_pkg, "config", "warehouse_layout.yaml")),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(sim_pkg, "maps", "sim_warehouse.yaml"),
                description="Occupancy YAML for PARK footprint clearance (same as Nav2 map)",
            ),
            DeclareLaunchArgument(
                "spawn_endpoint_code",
                default_value="",
                description="Override layout spawn_endpoint_code (e.g. EP-PARK-2); empty = layout default",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="warotrans",
                description="Gazebo model name (use warotrans_sim_02 for a second robot in the same world)",
            ),
            DeclareLaunchArgument(
                "topic_prefix",
                default_value="",
                description="Gazebo plugin topic prefix (e.g. sim01/) for multi-robot isolation",
            ),
            DeclareLaunchArgument(
                "robot_namespace",
                default_value="",
                description="ROS namespace for rsp/bridge (e.g. sim01); empty = global",
            ),
            DeclareLaunchArgument(
                "bridge_config",
                default_value="",
                description="Override ros_gz_bridge YAML; empty = default single-robot config",
            ),
            DeclareLaunchArgument(
                "skip_gazebo",
                default_value="false",
                description="Spawn robot into an already-running Gazebo world (Stage-3 dual)",
            ),
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
