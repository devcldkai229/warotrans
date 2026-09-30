"""Launch the RPLIDAR A1 driver and publish /scan.

Driver layer only: file này không tính state và không consume /scan.

Trước khi tin kết quả, DoD Phase 2 phải PASS:

    ros2 topic hz /scan
    ros2 topic echo /scan --once
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node


def _launch_setup(context: LaunchContext, *args: object, **kwargs: object) -> list[Node]:
    del args, kwargs

    params_file = Path(context.launch_configurations["params_file"]).expanduser()
    if not params_file.is_file():
        raise RuntimeError(
            f"lidar params_file not found: {params_file}. "
            "Check the warotrans_sensors install, or pass params_file:=<path>."
        )

    # Override chỉ áp dụng khi người dùng truyền giá trị khác rỗng; mặc định
    # lấy toàn bộ từ YAML để YAML vẫn là source of truth.
    overrides: dict[str, str] = {}
    for name in ("serial_port", "frame_id"):
        value = context.launch_configurations[name]
        if value:
            overrides[name] = value

    parameters: list[object] = [str(params_file)]
    if overrides:
        parameters.append(overrides)

    return [
        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            # Tên node phải khớp key namespace trong config/lidar.yaml.
            name="rplidar_node",
            output="screen",
            parameters=parameters,
        )
    ]


def generate_launch_description() -> LaunchDescription:
    default_params = str(
        Path(get_package_share_directory("warotrans_sensors"))
        / "config"
        / "lidar.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML file with RPLIDAR driver parameters",
            ),
            DeclareLaunchArgument(
                "serial_port",
                default_value="",
                description="Override serial_port from lidar.yaml (udev symlink)",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="",
                description="Override frame_id; must match a link in the URDF",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
