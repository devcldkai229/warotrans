"""Launch the WaroTrans Robot Gateway (ROS2 <-> MQTT: commands, heartbeat/telemetry, traffic).

Real robot, Semantic traffic (Nav2 plans, backend checks traffic and grants):

    ros2 launch warotrans_fleet fleet.launch.py \
        params_file:=$(ros2 pkg prefix warotrans_fleet)/share/warotrans_fleet/config/robot_gateway_semantic.yaml \
        semantic_manifest:=$(ros2 pkg prefix warotrans_navigation)/share/warotrans_navigation/maps/map_one_v002/semantic_manifest.json \
        robot_code:=RBT-001 mqtt_host:=<broker LAN IP>

semantic_manifest is the bundle the backend loads (Traffic:SemanticMapDirectory): its fingerprint ties every grant
to that map, and its mapCode becomes map_version_code. Empty launch args leave the YAML / WARO_* env values alone.
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def gateway_overrides(arg, environ) -> tuple[dict, str]:
    """Node parameter overrides from launch args (+ semantic bundle); returns (overrides, log line)."""
    overrides: dict = {}
    for name in ("robot_code", "mqtt_host", "map_version_code"):
        if arg(name):
            overrides[name] = arg(name)
    if arg("mqtt_port"):
        overrides["mqtt_port"] = int(arg("mqtt_port"))

    log = ""
    manifest_path = os.path.expanduser(arg("semantic_manifest"))
    if manifest_path:
        with open(manifest_path, encoding="utf-8") as f:
            manifest = json.load(f)
        fingerprint = str(manifest.get("semanticMapFingerprint") or "")
        map_code = str(manifest.get("mapCode") or "")
        if not fingerprint or not map_code:
            raise RuntimeError(f"{manifest_path} has no semanticMapFingerprint/mapCode")
        # The node prefers WARO_MAP_VERSION_CODE over its parameter, so a stale env would silently pair one
        # map code with another map's fingerprint.
        for source, value in (
            ("map_version_code", overrides.get("map_version_code")),
            ("WARO_MAP_VERSION_CODE", environ.get("WARO_MAP_VERSION_CODE")),
        ):
            if value and value != map_code:
                raise RuntimeError(f"{source}={value} != bundle mapCode {map_code}")
        bundle = os.path.dirname(manifest_path)
        policies = manifest.get("policies") or {}
        overrides.update(
            {
                "semantic_map_fingerprint": fingerprint,
                "map_version_code": map_code,
                "single_occupancy_json": os.path.join(bundle, policies.get("single_occupancy", "single_occupancy.json")),
                "no_stop_json": os.path.join(bundle, policies.get("no_stop", "no_stop.json")),
                "keepout_json": os.path.join(bundle, policies.get("keepout_zones", "keepout.json")),
                "lanes_json": os.path.join(bundle, policies.get("lanes", "lanes.json")),
            }
        )
        log = f"[warotrans_fleet] semantic bundle {map_code} fingerprint={fingerprint[:12]}… ({bundle})"
    return overrides, log


def _gateway(context, *args, **kwargs):
    del args, kwargs
    overrides, log = gateway_overrides(
        lambda name: LaunchConfiguration(name).perform(context).strip(), os.environ
    )
    actions = [LogInfo(msg=log)] if log else []
    actions.append(
        Node(
            package="warotrans_fleet",
            executable="robot_gateway_node",
            name="robot_gateway",
            output="screen",
            # Dict last: explicit launch args win over the YAML (env WARO_* still wins inside the node).
            parameters=[LaunchConfiguration("params_file"), overrides],
        )
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("warotrans_fleet")
    default_params = os.path.join(pkg_share, "config", "robot_gateway.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("robot_code", default_value=""),
            DeclareLaunchArgument("mqtt_host", default_value=""),
            DeclareLaunchArgument("mqtt_port", default_value=""),
            DeclareLaunchArgument("map_version_code", default_value=""),
            DeclareLaunchArgument(
                "semantic_manifest",
                default_value="",
                description="semantic_manifest.json of the published map bundle (required for semantic_active)",
            ),
            OpaqueFunction(function=_gateway),
        ]
    )
