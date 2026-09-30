"""WaroTrans Nav2 navigation launch — Phase 0 + optional Phase 3 costmap filters.

Controller publishes /cmd_vel (Twist) directly to esp32_bridge_node.

IMPORTANT: launch arg is nav2_params_file (NOT params_file) to avoid colliding
with lidar.launch.py / slam.launch.py which also declare params_file in the
same bringup LaunchContext.

enable_costmap_filters:=false (default, outdoor):
  no keepout/speed mask servers; outdoor overlay keeps filters [].

enable_costmap_filters:=true (Phase 3 demo):
  KEEP_OUT / SPEED_LIMIT mask servers + filters_on overlay.
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

    outdoor_overlay = os.path.join(pkg, "config", "nav2_params_outdoor.yaml")
    filters_on_overlay = os.path.join(pkg, "config", "nav2_params_filters_on.yaml")
    bt_xml = os.path.join(
        pkg, "behavior_trees", "navigate_to_pose_w_replanning_resilient.xml"
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

    common_params = param_files + [{"use_sim_time": use_sim_time}]
    bt_navigator_params = common_params + [{"default_nav_to_pose_bt_xml": bt_xml}]

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    actions = [
        LogInfo(msg=["[warotrans_navigation] nav2_params_file=", params_path]),
        LogInfo(
            msg=[
                "[warotrans_navigation] enable_costmap_filters=",
                "true" if enable_filters else "false",
            ]
        ),
        LogInfo(msg=["[warotrans_navigation] default_nav_to_pose_bt_xml=", bt_xml]),
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            respawn=True,
            respawn_delay=2.0,
            parameters=common_params,
            remappings=remappings,
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
            remappings=remappings,
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
        ),
    ]

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
                "enable_costmap_filters",
                default_value="false",
                description="Phase 3 keepout/speed filters (default off for outdoor)",
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
