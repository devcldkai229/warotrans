"""WaroTrans simulation — full stack: Gazebo + localization + Nav2 + fleet gateway.

    simulation.launch.py      Gazebo world + robot + ros_gz_bridge          (replaces hardware)
    map_server + AMCL         maps/sim_warehouse.yaml, initial pose = spawn (same nodes as the Pi)
    navigation.launch.py      Nav2 with nav2_params.yaml + outdoor + sim overlay (same launch as the Pi)
    robot_gateway_node        MQTT <-> Nav2 bridge for the backend (same node as the Pi, RBT-SIM-01)
    rviz2 (optional)

The backend talks to the simulated robot exactly as to the real one, through MQTT
topics warotrans/v1/robots/RBT-SIM-01/{command,command_ack,command_result,telemetry,heartbeat}.

USAGE
    ros2 launch warotrans_simulation sim_bringup.launch.py
    ros2 launch warotrans_simulation sim_bringup.launch.py rviz:=true gateway:=false
    ros2 launch warotrans_simulation sim_bringup.launch.py map_version_code:=MAP-WHDEMO-V001
"""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _flag(context: LaunchContext, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def _setup(context: LaunchContext, *args, **kwargs):
    del args, kwargs
    sim_pkg = get_package_share_directory("warotrans_simulation")
    loc_pkg = get_package_share_directory("warotrans_localization")
    nav_pkg = get_package_share_directory("warotrans_navigation")

    layout_path = LaunchConfiguration("layout").perform(context)
    with open(layout_path, encoding="utf-8") as handle:
        spawn = yaml.safe_load(handle)["spawn"]
    map_yaml = LaunchConfiguration("map").perform(context)
    nav_delay = float(LaunchConfiguration("nav_start_delay").perform(context))
    # Gateway after Nav2 is ACTIVE: otherwise the robot looks online while bt_navigator still rejects goals.
    gateway_delay = nav_delay + float(LaunchConfiguration("gateway_extra_delay").perform(context))
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    amcl_params = os.path.join(loc_pkg, "config", "amcl.yaml")

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim_pkg, "launch", "simulation.launch.py")),
            launch_arguments={"gui": LaunchConfiguration("gui"), "layout": layout_path}.items(),
        ),
        # Localization: same nodes/params as warotrans_localization, plus sim clock and a
        # known initial pose (the spawn pose) so no manual "2D Pose Estimate" is needed.
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[amcl_params, {"use_sim_time": True, "yaml_filename": map_yaml}],
            remappings=remappings,
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[
                amcl_params,
                {
                    "use_sim_time": True,
                    "set_initial_pose": True,
                    "initial_pose.x": float(spawn["x"]),
                    "initial_pose.y": float(spawn["y"]),
                    "initial_pose.z": 0.0,
                    "initial_pose.yaw": float(spawn["yaw"]),
                    # Sim odometry is exact (OdometryPublisher), unlike the skidding real robot:
                    # low motion noise, and NO random particle injection (recovery_alpha_*), which in a
                    # warehouse of identical aisles teleported the estimate into a look-alike aisle.
                    "alpha1": 0.05,
                    "alpha2": 0.05,
                    "alpha3": 0.05,
                    "alpha4": 0.05,
                    "recovery_alpha_slow": 0.0,
                    "recovery_alpha_fast": 0.0,
                    "max_beams": 120,
                },
            ],
            remappings=remappings,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{"use_sim_time": True, "autostart": True, "node_names": ["map_server", "amcl"]}],
        ),
        # Nav2 after Gazebo publishes /clock, /scan and AMCL publishes map -> odom.
        TimerAction(
            period=nav_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(nav_pkg, "launch", "navigation.launch.py")),
                    launch_arguments={
                        "use_sim_time": "true",
                        "enable_costmap_filters": "false",
                        "extra_params_file": os.path.join(sim_pkg, "config", "nav2_params_sim.yaml"),
                    }.items(),
                ),
            ],
        ),
    ]

    if _flag(context, "gateway"):
        gateway_params: list = [os.path.join(sim_pkg, "config", "robot_gateway_sim.yaml")]
        overrides = {}
        for key in ("robot_code", "mqtt_host", "map_version_code"):
            value = LaunchConfiguration(key).perform(context).strip()
            if value:
                overrides[key] = value
        if overrides:
            gateway_params.append(overrides)
        actions.append(
            TimerAction(
                period=gateway_delay,
                actions=[
                    Node(
                        package="warotrans_fleet",
                        executable="robot_gateway_node",
                        name="robot_gateway",
                        output="screen",
                        parameters=gateway_params,
                    )
                ],
            )
        )

    if _flag(context, "rviz"):
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", os.path.join(nav_pkg, "rviz", "nav2_warotrans.rviz")],
                parameters=[{"use_sim_time": True}],
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    sim_pkg = get_package_share_directory("warotrans_simulation")
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Show the Gazebo window"),
            DeclareLaunchArgument("rviz", default_value="false", description="Start RViz with the Nav2 view"),
            DeclareLaunchArgument("gateway", default_value="true", description="Start the MQTT fleet gateway"),
            DeclareLaunchArgument("layout", default_value=os.path.join(sim_pkg, "config", "warehouse_layout.yaml")),
            DeclareLaunchArgument("map", default_value=os.path.join(sim_pkg, "maps", "sim_warehouse.yaml")),
            DeclareLaunchArgument("nav_start_delay", default_value="8.0", description="Seconds before Nav2 starts"),
            DeclareLaunchArgument(
                "gateway_extra_delay",
                default_value="15.0",
                description="Extra seconds after Nav2 before the MQTT gateway starts (robot shows online only when Nav2 accepts goals)",
            ),
            DeclareLaunchArgument("robot_code", default_value="", description="Override gateway robot_code"),
            DeclareLaunchArgument("mqtt_host", default_value="", description="Override MQTT broker host"),
            DeclareLaunchArgument("map_version_code", default_value="", description="Backend MapVersion code"),
            OpaqueFunction(function=_setup),
        ]
    )
