"""WaroTrans Nav2 navigation launch — Phase 0 + optional Phase 3 costmap filters.

cmd_vel (Twist) reaches esp32_bridge_node through
controller/behaviors -> cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed
-> collision_monitor (LiDAR "approach") -> /cmd_vel.
enable_cmd_vel_pipeline:=false restores the direct controller -> /cmd_vel path.

IMPORTANT: launch arg is nav2_params_file (NOT params_file) to avoid colliding
with lidar.launch.py / slam.launch.py which also declare params_file in the
same bringup LaunchContext.

enable_costmap_filters:=false (default, outdoor):
  no keepout/speed mask servers; outdoor overlay keeps filters [].

enable_costmap_filters:=true (Phase 3 demo):
  KEEP_OUT / SPEED_LIMIT mask servers + filters_on overlay.

route_graph:=<file.geojson> (lane graph from the backend route-graph API):
  adds nav2_route route_server (config/route_server.yaml) and switches NavigateToPose to
  behavior_trees/navigate_to_pose_route.xml: the robot drives only along the graph, in the lane
  directions. Empty (default, real robot) = free GridBased planning as before.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    del args, kwargs
    pkg = get_package_share_directory("warotrans_navigation")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    params_path = LaunchConfiguration("nav2_params_file").perform(context).strip()
    if not params_path:
        params_path = os.path.join(pkg, "config", "nav2_params.yaml")
    params_path = os.path.expanduser(params_path)
    if not os.path.isfile(params_path):
        raise RuntimeError(f"nav2 params file not found: {params_path}")

    enable_filters = (
        LaunchConfiguration("enable_costmap_filters").perform(context).strip().lower()
        in ("true", "1", "yes")
    )
    enable_ekf = (
        LaunchConfiguration("enable_ekf").perform(context).strip().lower()
        in ("true", "1", "yes")
    )

    outdoor_overlay = os.path.join(pkg, "config", "nav2_params_outdoor.yaml")
    filters_on_overlay = os.path.join(pkg, "config", "nav2_params_filters_on.yaml")
    ekf_overlay = os.path.join(pkg, "config", "nav2_params_ekf.yaml")
    route_graph = os.path.expanduser(LaunchConfiguration("route_graph").perform(context).strip())
    if route_graph and not os.path.isfile(route_graph):
        raise RuntimeError(
            f"route_graph not found: {route_graph} "
            "(fetch it with warotrans_simulation/scripts/fetch_route_graph.sh)"
        )
    bt_xml = os.path.join(
        pkg,
        "behavior_trees",
        "navigate_to_pose_route.xml" if route_graph else "navigate_to_pose_w_replanning_resilient.xml",
    )
    if not os.path.isfile(bt_xml):
        raise RuntimeError(f"BT XML not found: {bt_xml}")

    param_files = [params_path]
    if enable_filters:
        if not os.path.isfile(filters_on_overlay):
            raise RuntimeError(f"filters_on overlay missing: {filters_on_overlay}")
        param_files.append(filters_on_overlay)
    else:
        if os.path.isfile(outdoor_overlay):
            param_files.append(outdoor_overlay)
    if enable_ekf:
        if not os.path.isfile(ekf_overlay):
            raise RuntimeError(f"EKF overlay missing: {ekf_overlay}")
        param_files.append(ekf_overlay)

    # Lane graph mode: centre-line tracking defaults (lookahead). Applied before extra_params so the
    # simulation overlay can raise lookahead to match desired_linear_vel (v/L ≤ ~1.2) without being
    # clobbered by route's 0.20 m default (traffic-hardening Phase B, 2026-10-28).
    if route_graph:
        param_files.append(os.path.join(pkg, "config", "nav2_params_route.yaml"))

    # Optional last overlay (e.g. warotrans_simulation nav2_params_sim.yaml). Empty = none.
    extra_params = os.path.expanduser(
        LaunchConfiguration("extra_params_file").perform(context).strip()
    )
    if extra_params:
        if not os.path.isfile(extra_params):
            raise RuntimeError(f"extra_params_file not found: {extra_params}")
        param_files.append(extra_params)

    common_params = param_files + [{"use_sim_time": use_sim_time}]
    bt_navigator_params = common_params + [{"default_nav_to_pose_bt_xml": bt_xml}]
    if route_graph:
        bt_navigator_params.append(
            {"error_code_names": ["compute_route_error_code", "follow_path_error_code"]}
        )

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
    ]
    # route_server before bt_navigator: the route BT nodes need its actions when the tree loads.
    if route_graph:
        lifecycle_nodes.append("route_server")
    lifecycle_nodes.append("bt_navigator")

    # cmd_vel pipeline (default on): controller/behaviors -> cmd_vel_nav -> velocity_smoother
    # -> cmd_vel_smoothed -> collision_monitor (LiDAR) -> /cmd_vel. Off = controller writes
    # /cmd_vel directly (Phase 0 behaviour).
    enable_pipeline = (
        LaunchConfiguration("enable_cmd_vel_pipeline").perform(context).strip().lower()
        in ("true", "1", "yes")
    )
    enable_authority_gate = (
        LaunchConfiguration("enable_authority_gate").perform(context).strip().lower()
        in ("true", "1", "yes")
    )
    motion_remappings = remappings + ([("cmd_vel", "cmd_vel_nav")] if enable_pipeline else [])
    if enable_pipeline:
        lifecycle_nodes += ["velocity_smoother", "collision_monitor"]

    actions = [
        LogInfo(msg=["[warotrans_navigation] nav2_params_file=", params_path]),
        LogInfo(
            msg=[
                "[warotrans_navigation] enable_costmap_filters=",
                "true" if enable_filters else "false",
            ]
        ),
        LogInfo(
            msg=[
                "[warotrans_navigation] enable_ekf=",
                "true" if enable_ekf else "false",
                " odom_topic=",
                "/odometry/filtered" if enable_ekf else "/odom",
            ]
        ),
        LogInfo(msg=["[warotrans_navigation] default_nav_to_pose_bt_xml=", bt_xml]),
        LogInfo(msg=["[warotrans_navigation] param_files=", ", ".join(param_files)]),
        LogInfo(
            msg=[
                "[warotrans_navigation] cmd_vel pipeline=",
                "controller->velocity_smoother->collision_monitor(/scan)->/cmd_vel"
                if enable_pipeline
                else "controller->/cmd_vel (direct)",
            ]
        ),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=common_params,
            remappings=motion_remappings,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=common_params,
            remappings=remappings,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=common_params,
            remappings=motion_remappings,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=bt_navigator_params,
            remappings=remappings,
        ),
    ]

    if route_graph:
        actions += [
            LogInfo(msg=["[warotrans_navigation] route_graph=", route_graph, " (lane graph navigation)"]),
            Node(
                package="nav2_route",
                executable="route_server",
                name="route_server",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=[
                    os.path.join(pkg, "config", "route_server.yaml"),
                    {"graph_filepath": route_graph, "use_sim_time": use_sim_time},
                ],
                remappings=remappings,
            ),
        ]

    if enable_pipeline:
        collision_params = list(common_params)
        if enable_authority_gate:
            # Gate sits between collision_monitor and /cmd_vel (Active Mode only). launch_ros dict overrides
            # take the plain parameter name; a "node.ros__parameters.x" key is silently ignored and
            # collision_monitor kept publishing straight to /cmd_vel, bypassing the gate.
            collision_params = common_params + [{"cmd_vel_out_topic": "cmd_vel_safe"}]
        actions += [
            Node(
                package="nav2_velocity_smoother",
                executable="velocity_smoother",
                name="velocity_smoother",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=common_params,
                # in: cmd_vel_nav (remapped "cmd_vel"), out: cmd_vel_smoothed
                remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
            ),
            Node(
                package="nav2_collision_monitor",
                executable="collision_monitor",
                name="collision_monitor",
                output="screen",
                respawn=True,
                respawn_delay=2.0,
                parameters=collision_params,
                remappings=remappings,
            ),
        ]
        if enable_authority_gate:
            actions.append(
                Node(
                    package="warotrans_fleet",
                    executable="cmd_vel_authority_gate",
                    name="cmd_vel_authority_gate",
                    output="screen",
                    respawn=True,
                    respawn_delay=2.0,
                    parameters=[{"use_sim_time": use_sim_time}],
                    remappings=remappings,
                )
            )
            actions.append(
                LogInfo(
                    msg=[
                        "[warotrans_navigation] authority gate ON: "
                        "collision_monitor->/cmd_vel_safe->gate->/cmd_vel"
                    ]
                )
            )

    actions.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": lifecycle_nodes},
                {"bond_timeout": 25.0},
            ],
        )
    )

    if enable_filters:
        keepout_mask = LaunchConfiguration("keepout_mask").perform(context).strip()
        speed_mask = LaunchConfiguration("speed_mask").perform(context).strip()
        keepout_mask = os.path.expanduser(keepout_mask)
        speed_mask = os.path.expanduser(speed_mask)
        if not os.path.isfile(keepout_mask):
            raise RuntimeError(
                f"keepout mask yaml not found: {keepout_mask} "
                "(run tools/warotrans-ensure-filter-masks.sh)"
            )
        if not os.path.isfile(speed_mask):
            raise RuntimeError(
                f"speed mask yaml not found: {speed_mask} "
                "(run tools/warotrans-ensure-filter-masks.sh)"
            )

        filter_lifecycle_nodes = [
            "keepout_filter_mask_server",
            "keepout_costmap_filter_info_server",
            "speed_filter_mask_server",
            "speed_costmap_filter_info_server",
        ]
        actions.extend(
            [
                LogInfo(msg=["[warotrans_navigation] keepout_mask=", keepout_mask]),
                LogInfo(msg=["[warotrans_navigation] speed_mask=", speed_mask]),
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="keepout_filter_mask_server",
                    output="screen",
                    parameters=[
                        params_path,
                        {"use_sim_time": use_sim_time},
                        {"yaml_filename": keepout_mask},
                    ],
                    remappings=remappings,
                ),
                Node(
                    package="nav2_map_server",
                    executable="costmap_filter_info_server",
                    name="keepout_costmap_filter_info_server",
                    output="screen",
                    parameters=common_params,
                    remappings=remappings,
                ),
                Node(
                    package="nav2_map_server",
                    executable="map_server",
                    name="speed_filter_mask_server",
                    output="screen",
                    parameters=[
                        params_path,
                        {"use_sim_time": use_sim_time},
                        {"yaml_filename": speed_mask},
                    ],
                    remappings=remappings,
                ),
                Node(
                    package="nav2_map_server",
                    executable="costmap_filter_info_server",
                    name="speed_costmap_filter_info_server",
                    output="screen",
                    parameters=common_params,
                    remappings=remappings,
                ),
                Node(
                    package="nav2_lifecycle_manager",
                    executable="lifecycle_manager",
                    name="lifecycle_manager_costmap_filters",
                    output="screen",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"autostart": autostart},
                        {"node_names": filter_lifecycle_nodes},
                        {"bond_timeout": 25.0},
                    ],
                ),
            ]
        )

    return actions


def generate_launch_description():
    pkg = get_package_share_directory("warotrans_navigation")
    default_params = os.path.join(pkg, "config", "nav2_params.yaml")
    home = os.path.expanduser("~")
    default_keepout = os.path.join(home, "maps", "warotrans_keepout.yaml")
    default_speed = os.path.join(home, "maps", "warotrans_speed.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=default_params,
                description="Nav2 params YAML (do not name this params_file)",
            ),
            DeclareLaunchArgument(
                "extra_params_file",
                default_value="",
                description="Optional Nav2 params overlay merged last (after route overlay when set)",
            ),
            DeclareLaunchArgument(
                "enable_costmap_filters",
                default_value="false",
                description="Phase 3 keepout/speed filters (default off for outdoor)",
            ),
            DeclareLaunchArgument(
                "enable_authority_gate",
                default_value="false",
                description=(
                    "Active Mode: insert cmd_vel_authority_gate between collision_monitor "
                    "(/cmd_vel_safe) and /cmd_vel. Default false (legacy)."
                ),
            ),
            DeclareLaunchArgument(
                "enable_cmd_vel_pipeline",
                default_value="true",
                description=(
                    "controller -> velocity_smoother -> collision_monitor (LiDAR) -> /cmd_vel. "
                    "false = controller publishes /cmd_vel directly"
                ),
            ),
            DeclareLaunchArgument(
                "enable_ekf",
                default_value="false",
                description=(
                    "Use /odometry/filtered (nav2_params_ekf.yaml). "
                    "Must match bringup enable_ekf:=true + ekf_filter_node."
                ),
            ),
            DeclareLaunchArgument(
                "route_graph",
                default_value="",
                description=(
                    "Nav2 Route Server graph (GeoJSON from the backend route-graph API). "
                    "Set = drive only along the lane graph; empty = free planning (default)."
                ),
            ),
            DeclareLaunchArgument(
                "keepout_mask",
                default_value=default_keepout,
                description="KEEP_OUT OccupancyGrid mask YAML (when filters on)",
            ),
            DeclareLaunchArgument(
                "speed_mask",
                default_value=default_speed,
                description="SPEED_LIMIT OccupancyGrid mask YAML (when filters on)",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
