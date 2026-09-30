"""Launch robot_state_publisher with the WaroTrans Xacro model."""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _parse_override(value: str) -> float | None:
    if value == "":
        return None
    return float(value)


def _resolve_geometry(context: LaunchContext) -> dict[str, Any]:
    geometry_path = Path(
        context.launch_configurations["geometry_file"]
    ).expanduser()
    if not geometry_path.is_file():
        raise RuntimeError(
            f"geometry_file not found: {geometry_path}. "
            "Check warotrans_description install or pass geometry_file:=..."
        )

    with geometry_path.open(encoding="utf-8") as handle:
        geometry = yaml.safe_load(handle)

    footprint = geometry.get("base_footprint_to_base_link", {})
    laser = geometry.get("laser", {})
    ultrasonic = geometry.get("ultrasonic") or {}

    base_link_z = _parse_override(context.launch_configurations["base_link_z"])
    if base_link_z is None:
        base_link_z = footprint.get("z")

    lidar_values: dict[str, float | None] = {}
    for key in ("x", "y", "z", "yaw"):
        override = _parse_override(context.launch_configurations[f"lidar_{key}"])
        lidar_values[key] = override if override is not None else laser.get(key)

    missing: list[str] = []
    if base_link_z is None:
        missing.append("base_footprint_to_base_link.z (use base_link_z:=...)")
    for key, value in lidar_values.items():
        if value is None:
            missing.append(f"laser.{key} (use lidar_{key}:=...)")

    if missing:
        joined = ", ".join(missing)
        raise RuntimeError(
            "Unmeasured geometry in geometry.yaml: "
            f"{joined}. Record measurements in docs/calibration.md, "
            "update config/geometry.yaml, or pass explicit launch overrides "
            "for structural smoke tests only."
        )

    # FRONT + REAR only for Nav2 Phase 0.5 TF (LEFT/RIGHT ignored)
    us_front = ultrasonic.get("front") or {}
    us_rear = ultrasonic.get("rear") or {}
    front_ok = all(us_front.get(k) is not None for k in ("x", "y", "z", "yaw"))
    rear_ok = all(us_rear.get(k) is not None for k in ("x", "y", "z", "yaw"))
    enable_us = front_ok and rear_ok

    result: dict[str, Any] = {
        "base_link_z": float(base_link_z),
        "lidar_x": float(lidar_values["x"]),
        "lidar_y": float(lidar_values["y"]),
        "lidar_z": float(lidar_values["z"]),
        "lidar_yaw": float(lidar_values["yaw"]),
        "enable_ultrasonic_front_rear": enable_us,
    }
    if enable_us:
        result.update(
            {
                "ultrasonic_front_x": float(us_front["x"]),
                "ultrasonic_front_y": float(us_front["y"]),
                "ultrasonic_front_z": float(us_front["z"]),
                "ultrasonic_front_yaw": float(us_front["yaw"]),
                "ultrasonic_rear_x": float(us_rear["x"]),
                "ultrasonic_rear_y": float(us_rear["y"]),
                "ultrasonic_rear_z": float(us_rear["z"]),
                "ultrasonic_rear_yaw": float(us_rear["yaw"]),
            }
        )
    return result


def _launch_setup(context: LaunchContext, *args: object, **kwargs: object) -> list[Node]:
    del args, kwargs

    resolved = _resolve_geometry(context)
    geometry_file = context.launch_configurations["geometry_file"]

    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("warotrans_description"),
            "urdf",
            "warotrans.urdf.xacro",
        ]
    )

    xacro_cmd = [
        "xacro ",
        xacro_file,
        " geometry_file:=",
        geometry_file,
        " base_link_z:=",
        str(resolved["base_link_z"]),
        " lidar_x:=",
        str(resolved["lidar_x"]),
        " lidar_y:=",
        str(resolved["lidar_y"]),
        " lidar_z:=",
        str(resolved["lidar_z"]),
        " lidar_yaw:=",
        str(resolved["lidar_yaw"]),
        " enable_ultrasonic_front_rear:=",
        "true" if resolved["enable_ultrasonic_front_rear"] else "false",
    ]
    if resolved["enable_ultrasonic_front_rear"]:
        xacro_cmd.extend(
            [
                " ultrasonic_front_x:=",
                str(resolved["ultrasonic_front_x"]),
                " ultrasonic_front_y:=",
                str(resolved["ultrasonic_front_y"]),
                " ultrasonic_front_z:=",
                str(resolved["ultrasonic_front_z"]),
                " ultrasonic_front_yaw:=",
                str(resolved["ultrasonic_front_yaw"]),
                " ultrasonic_rear_x:=",
                str(resolved["ultrasonic_rear_x"]),
                " ultrasonic_rear_y:=",
                str(resolved["ultrasonic_rear_y"]),
                " ultrasonic_rear_z:=",
                str(resolved["ultrasonic_rear_z"]),
                " ultrasonic_rear_yaw:=",
                str(resolved["ultrasonic_rear_yaw"]),
            ]
        )

    robot_description = ParameterValue(
        Command(xacro_cmd),
        value_type=str,
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "use_sim_time": False,
                }
            ],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_geometry = str(
        Path(get_package_share_directory("warotrans_description"))
        / "config"
        / "geometry.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "geometry_file",
                default_value=default_geometry,
                description="YAML file with chassis and sensor mount geometry",
            ),
            DeclareLaunchArgument(
                "base_link_z",
                default_value="",
                description="Override base_footprint_to_base_link.z in metres",
            ),
            DeclareLaunchArgument(
                "lidar_x",
                default_value="",
                description="Override laser.x in metres (base_link frame)",
            ),
            DeclareLaunchArgument(
                "lidar_y",
                default_value="",
                description="Override laser.y in metres (base_link frame)",
            ),
            DeclareLaunchArgument(
                "lidar_z",
                default_value="",
                description="Override laser.z in metres (base_link frame)",
            ),
            DeclareLaunchArgument(
                "lidar_yaw",
                default_value="",
                description="Override laser.yaw in radians (base_link frame)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
