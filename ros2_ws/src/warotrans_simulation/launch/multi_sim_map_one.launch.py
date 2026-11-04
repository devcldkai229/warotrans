"""Stage-3: two namespaced robots (SIM-01 / SIM-02) on MAP-ONE in one Gazebo world.

Each robot has isolated TF, sensors, Nav2, authority gate and MQTT gateway.
Shared: world, occupancy map, semantic masks, Fleet Traffic Coordinator (Backend).
"""

from __future__ import annotations

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.actions import SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def _flag(context: LaunchContext, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).lower() in ("true", "1", "yes")


def _robot_stack(
    *,
    context: LaunchContext,
    sim_pkg: str,
    loc_pkg: str,
    nav_pkg: str,
    ns: str,
    robot_name: str,
    robot_code: str,
    spawn_endpoint: str,
    gateway_yaml: str,
    bridge_yaml: str,
    map_yaml: str,
    layout_path: str,
    world: str,
    skip_gazebo: bool,
    gui: str,
    nav_delay: float,
    gateway_delay: float,
    lanes_json: str,
    no_stop_json: str,
    keepout_json: str,
    single_occ_json: str,
    keepout_mask: str,
    speed_mask: str,
    fingerprint: str,
    enable_filters: str,
    open_grant: bool,
    extra_params: str,
) -> list:
    launch_dir = os.path.dirname(os.path.abspath(__file__))
    if launch_dir not in sys.path:
        sys.path.insert(0, launch_dir)
    from _spawn_util import resolve_layout_spawn

    spawn = resolve_layout_spawn(layout_path, endpoint_code=spawn_endpoint, map_yaml=map_yaml)
    amcl_params = os.path.join(loc_pkg, "config", "amcl.yaml")
    topic_prefix = f"{ns}/"
    odom_frame = f"{ns}/odom"
    base_frame = f"{ns}/base_footprint"
    remappings = [("tf", "tf"), ("tf_static", "tf_static")]

    sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(sim_pkg, "launch", "simulation.launch.py")),
        launch_arguments={
            "gui": gui if not skip_gazebo else "false",
            "world": world,
            "layout": layout_path,
            "map": map_yaml,
            "spawn_endpoint_code": spawn_endpoint,
            "robot_name": robot_name,
            "topic_prefix": topic_prefix,
            # Already under PushRosNamespace(ns) — do not nest a second namespace.
            "robot_namespace": "",
            "bridge_config": bridge_yaml,
            "skip_gazebo": "true" if skip_gazebo else "false",
            "enable_ekf": "false",
            "motor_deadband": "0.0",
        }.items(),
    )

    loc_nodes = [
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
                    "alpha1": 0.05,
                    "alpha2": 0.05,
                    "alpha3": 0.05,
                    "alpha4": 0.05,
                    "max_beams": 120,
                    # Relative topic under PushRosNamespace → /{ns}/scan
                    "scan_topic": "scan",
                    "odom_frame_id": odom_frame,
                    "base_frame_id": base_frame,
                    "global_frame_id": "map",
                },
            ],
            remappings=remappings,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "autostart": True,
                    "node_names": ["map_server", "amcl"],
                }
            ],
        ),
    ]

    nav_include = TimerAction(
        period=nav_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav_pkg, "launch", "navigation.launch.py")),
                launch_arguments={
                    "use_sim_time": "true",
                    "namespace": ns,
                    "enable_costmap_filters": enable_filters,
                    "keepout_mask": keepout_mask,
                    "speed_mask": speed_mask,
                    "lanes_json": lanes_json,
                    "enable_ekf": "false",
                    "extra_params_file": extra_params,
                    "route_graph": "",
                    "enable_cmd_vel_pipeline": "true",
                    "enable_authority_gate": "true",
                }.items(),
            )
        ],
    )

    validator = Node(
        package="warotrans_fleet",
        executable="path_policy_validator",
        name="path_policy_validator",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "enable": True,
                "lanes_json": lanes_json,
                "no_stop_json": no_stop_json,
                "keepout_json": keepout_json,
                "margin": 0.05,
                # Match sim_bringup: log REJECT, do not cancel_goal storm (ONE_WAY false-positives
                # mid-route canceled every follow-up NavigateToPose after first long leg).
                "cancel_on_violation": False,
                "plan_topic": "plan",
                "goal_topic": "goal_pose",
            }
        ],
    )

    # YAML top-level `robot_gateway:` must be rewritten under PushRosNamespace(ns).
    gateway_params = ParameterFile(
        RewrittenYaml(
            source_file=gateway_yaml,
            root_key=ns,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    gateway = TimerAction(
        period=gateway_delay,
        actions=[
            Node(
                package="warotrans_fleet",
                executable="robot_gateway_node",
                name="robot_gateway",
                output="screen",
                parameters=[
                    gateway_params,
                    {
                        "robot_code": robot_code,
                        "map_version_code": "MAP-ONE-V001",
                        "semantic_map_fingerprint": fingerprint,
                        "lanes_json": lanes_json,
                        "no_stop_json": no_stop_json,
                        "keepout_json": keepout_json,
                        "single_occupancy_json": single_occ_json,
                        "use_sim_time": True,
                        # Belt-and-suspenders: dict params always apply under namespace.
                        "traffic_mode": "semantic_active",
                        "mqtt_host": "127.0.0.1",
                        "enable_authority_gate": True,
                        "cmd_vel_topic": "cmd_vel_safe",
                        # Under stage1_open_grant the open-grant node owns
                        # traffic/authority_ok. Leaving the semantic client on the same
                        # topic made the two fight: it publishes False with no Host grant,
                        # so the gate flapped True/False all run (measured 30/102 samples
                        # false, run stretched to ~205 s). Park the semantic signal on a
                        # sibling topic so it stays observable without gating.
                        "authority_ok_topic": (
                            "traffic/authority_ok_semantic"
                            if open_grant
                            else "traffic/authority_ok"
                        ),
                        "odom_topic": "odom",
                        "amcl_pose_topic": "amcl_pose",
                    },
                ],
            )
        ],
    )

    # Stage-1 open grant: isolates Nav2 geometry from the Semantic grant/renew path so a
    # direct NavigateToPose can be measured without authority expiry as a confounder. The
    # authority GATE stays on; only the grant source changes. Relative topic so it lands on
    # /<ns>/traffic/authority_ok. use_sim_time False because cmd_vel_authority_gate ages
    # authority_ok with time.monotonic() against max_age 0.3 s -- a sim-time timer under slow
    # Gazebo blows that window and blocks /cmd_vel even while the grant is "active".
    open_grant_nodes = (
        [
            Node(
                package="warotrans_simulation",
                executable="stage1_open_grant_node.py",
                name="stage1_open_grant",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "enable": True,
                        "hz": 20.0,
                        "topic": "traffic/authority_ok",
                    }
                ],
            )
        ]
        if open_grant
        else []
    )

    return [
        LogInfo(
            msg=(
                f"[multi_sim] {ns} robot={robot_name} code={robot_code} "
                f"spawn=({spawn['x']:.3f},{spawn['y']:.3f}) endpoint={spawn_endpoint}"
                f"{' STAGE1_OPEN_GRANT' if open_grant else ''}"
            )
        ),
        GroupAction(
            actions=[
                PushRosNamespace(ns),
                SetRemap(src="/tf", dst="tf"),
                SetRemap(src="/tf_static", dst="tf_static"),
                sim_include,
                *loc_nodes,
                nav_include,
                validator,
                gateway,
                *open_grant_nodes,
            ]
        ),
    ]


def _setup(context: LaunchContext, *args, **kwargs):
    del args, kwargs
    sim_pkg = get_package_share_directory("warotrans_simulation")
    loc_pkg = get_package_share_directory("warotrans_localization")
    nav_pkg = get_package_share_directory("warotrans_navigation")

    layout = LaunchConfiguration("layout").perform(context)
    map_yaml = LaunchConfiguration("map").perform(context)
    world = LaunchConfiguration("world").perform(context)
    gui = LaunchConfiguration("gui").perform(context)
    nav_delay = float(LaunchConfiguration("nav_start_delay").perform(context))
    gateway_delay = nav_delay + float(LaunchConfiguration("gateway_extra_delay").perform(context))

    map_dir = os.path.dirname(os.path.abspath(map_yaml))
    keepout_mask = os.path.join(map_dir, "keepout.yaml")
    speed_mask = os.path.join(map_dir, "speed.yaml")
    lanes_json = os.path.join(map_dir, "lanes.json")
    no_stop_json = os.path.join(map_dir, "no_stop.json")
    keepout_json = os.path.join(map_dir, "keepout.json")
    single_occ = os.path.join(map_dir, "single_occupancy.json")
    manifest = os.path.join(map_dir, "semantic_manifest.json")
    fingerprint = ""
    if os.path.isfile(manifest):
        import json

        with open(manifest, encoding="utf-8") as fh:
            fingerprint = str(json.load(fh).get("semanticMapFingerprint") or "")

    for required in (keepout_mask, speed_mask, lanes_json, world, map_yaml):
        if not os.path.isfile(required):
            raise RuntimeError(f"MAP-ONE Stage-3 artifact missing: {required}")

    lane_overlay = os.path.join(nav_pkg, "config", "nav2_params_lane_aware.yaml")
    sim_overlay = os.path.join(sim_pkg, "config", "nav2_params_sim.yaml")
    ns_template = os.path.join(sim_pkg, "config", "nav2_params_namespaced_overlay.yaml.template")

    def _extras_for(ns: str) -> str:
        # LaneAware keeps paths in ONE_WAY corridors (parking leave → junction → aisle)
        # instead of Navfn geodesic wall-hugging. Soft-handoff/revoke-grace on gateway
        # mitigate the prior ~0.85 m progress-abort thrash.
        parts = [p for p in (sim_overlay, lane_overlay) if os.path.isfile(p)]
        if os.path.isfile(ns_template):
            text = open(ns_template, encoding="utf-8").read()
            text = text.replace("__NS__", ns).replace("__NS_SLASH__", f"{ns}/")
            out = f"/tmp/warotrans_nav2_ns_{ns}.yaml"
            with open(out, "w", encoding="utf-8") as fh:
                fh.write(text)
            parts.append(out)
        return ";".join(parts)

    robots = LaunchConfiguration("robots").perform(context).strip().lower()
    if robots not in ("sim01", "sim02", "both"):
        raise RuntimeError(f"robots must be sim01|sim02|both, got {robots!r}")

    open_grant = _flag(context, "stage1_open_grant")
    actions = [
        LogInfo(
            msg=(
                f"[multi_sim] Stage-3 MAP-ONE robots={robots} "
                + (
                    "STAGE1_OPEN_GRANT=on (gate still ON; isolation run only, never the Pi)"
                    if open_grant
                    else "(no stage1_open_grant)"
                )
            )
        ),
        # Shared clock bridge (world started by first robot stack).
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="ros_gz_bridge_clock",
            output="screen",
            parameters=[
                {"config_file": os.path.join(sim_pkg, "config", "ros_gz_bridge_clock.yaml")},
                {"use_sim_time": True},
            ],
        ),
    ]

    spawn_sim01 = robots in ("sim01", "both")
    spawn_sim02 = robots in ("sim02", "both")
    # First spawned robot owns Gazebo world start.
    first_is_sim01 = spawn_sim01

    if spawn_sim01:
        actions += _robot_stack(
            context=context,
            sim_pkg=sim_pkg,
            loc_pkg=loc_pkg,
            nav_pkg=nav_pkg,
            ns="sim01",
            robot_name="warotrans_sim_01",
            robot_code="RBT-SIM-01",
            spawn_endpoint="EP-PARKING-A",
            gateway_yaml=os.path.join(sim_pkg, "config", "robot_gateway_sim01_semantic_active.yaml"),
            bridge_yaml=os.path.join(sim_pkg, "config", "ros_gz_bridge_sim01.yaml"),
            map_yaml=map_yaml,
            layout_path=layout,
            world=world,
            skip_gazebo=False,
            gui=gui,
            nav_delay=nav_delay,
            gateway_delay=gateway_delay,
            lanes_json=lanes_json,
            no_stop_json=no_stop_json,
            keepout_json=keepout_json,
            single_occ_json=single_occ,
            keepout_mask=keepout_mask,
            speed_mask=speed_mask,
            fingerprint=fingerprint,
            # KeepoutFilter mask QoS/race left local_costmap in "Filter mask was not received"
            # loop → progress abort ~0.85 m. KEEP_OUT still enforced by path_policy_validator JSON.
            enable_filters="false",
            open_grant=open_grant,
            extra_params=_extras_for("sim01"),
        )
    if spawn_sim02:
        actions += _robot_stack(
            context=context,
            sim_pkg=sim_pkg,
            loc_pkg=loc_pkg,
            nav_pkg=nav_pkg,
            ns="sim02",
            robot_name="warotrans_sim_02",
            robot_code="RBT-SIM-02",
            spawn_endpoint="EP-PARKING-B",
            gateway_yaml=os.path.join(sim_pkg, "config", "robot_gateway_sim02_semantic_active.yaml"),
            bridge_yaml=os.path.join(sim_pkg, "config", "ros_gz_bridge_sim02.yaml"),
            map_yaml=map_yaml,
            layout_path=layout,
            world=world,
            skip_gazebo=first_is_sim01,
            gui="false" if first_is_sim01 else gui,
            nav_delay=nav_delay + (2.0 if first_is_sim01 else 0.0),
            gateway_delay=gateway_delay + (2.0 if first_is_sim01 else 0.0),
            lanes_json=lanes_json,
            no_stop_json=no_stop_json,
            keepout_json=keepout_json,
            single_occ_json=single_occ,
            keepout_mask=keepout_mask,
            speed_mask=speed_mask,
            fingerprint=fingerprint,
            # KeepoutFilter mask QoS/race left local_costmap in "Filter mask was not received"
            # loop → progress abort ~0.85 m. KEEP_OUT still enforced by path_policy_validator JSON.
            enable_filters="false",
            open_grant=open_grant,
            extra_params=_extras_for("sim02"),
        )
    return actions


def generate_launch_description() -> LaunchDescription:
    sim_pkg = get_package_share_directory("warotrans_simulation")
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument(
                "robots",
                default_value="both",
                description="sim01 | sim02 | both — standalone acceptance vs dual",
            ),
            DeclareLaunchArgument(
                "layout",
                default_value=os.path.join(sim_pkg, "config", "layouts", "map_one_v001.yaml"),
            ),
            DeclareLaunchArgument(
                "map",
                default_value=os.path.join(sim_pkg, "maps", "map_one_v001", "map.yaml"),
            ),
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(sim_pkg, "worlds", "map_one_v001.sdf"),
            ),
            # Allow AMCL map→odom after prefixed laser TF aligns (dual bringup is heavier).
            DeclareLaunchArgument("nav_start_delay", default_value="25.0"),
            DeclareLaunchArgument("gateway_extra_delay", default_value="20.0"),
            # Sim-only, opt-in, logged. Stage-1 open grant is NOT a production traffic
            # authorization and must never ship to the physical robot.
            DeclareLaunchArgument("stage1_open_grant", default_value="false"),
            OpaqueFunction(function=_setup),
        ]
    )
