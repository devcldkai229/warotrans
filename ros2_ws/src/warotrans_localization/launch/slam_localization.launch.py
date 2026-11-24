"""WaroTrans localization with SLAM Toolbox in LOCALIZATION mode (alternative to map_server + AMCL).

slam_toolbox loads the pose graph saved when the map was built (``<posegraph>.posegraph`` + ``<posegraph>.data``),
matches every scan against it and publishes /map, /pose and TF map -> odom. It is more robust than AMCL where the
warehouse has few features, but it needs that pose graph: a map drawn or edited only on the web has none.

TF OWNERSHIP
    slam_toolbox owns map -> odom. Never run it with AMCL (localization.launch.py).

GATEWAY (robot_gateway, warotrans_fleet)
    localizer:        slam_toolbox   (auto-localize: scan matching only, no global spin)
    amcl_pose_topic:  /pose          (readiness reads the pose covariance from there)

Saving the pose graph after mapping (slam.launch.py, mapping mode):
    ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \\
        "{filename: '$HOME/maps/map_one_v019/posegraph'}"

USAGE
    ros2 launch warotrans_localization slam_localization.launch.py posegraph:=$HOME/maps/map_one_v019/posegraph
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, RegisterEventHandler
from launch.events import matches_action
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _launch_setup(context: LaunchContext, *args: object, **kwargs: object):
    del args, kwargs
    params_file = Path(context.launch_configurations["slam_params_file"]).expanduser()
    if not params_file.is_file():
        raise RuntimeError(f"slam_toolbox params file not found: {params_file}")
    posegraph = str(Path(context.launch_configurations["posegraph"]).expanduser())
    if not Path(posegraph + ".posegraph").is_file() or not Path(posegraph + ".data").is_file():
        raise RuntimeError(
            f"No pose graph at {posegraph}(.posegraph/.data). Build the map with slam.launch.py and save it with "
            "/slam_toolbox/serialize_map, or use AMCL (localizer:=amcl)."
        )
    use_sim_time = context.launch_configurations["use_sim_time"].lower() == "true"

    # The mapping parameters (scanner, solver, matcher) are shared; only the mode and the graph to load differ.
    node = LifecycleNode(
        package="slam_toolbox",
        executable="localization_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            str(params_file),
            {
                "mode": "localization",
                "map_file_name": posegraph,
                # Start where the map started (the dock / parking); auto-localize or /initialpose corrects it.
                "map_start_at_dock": True,
                "use_sim_time": use_sim_time,
            },
        ],
    )
    configure = EmitEvent(
        event=ChangeState(lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_CONFIGURE)
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[slam_localization] slam_toolbox activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node), transition_id=Transition.TRANSITION_ACTIVATE
                    )
                ),
            ],
        )
    )
    return [node, configure, activate]


def generate_launch_description() -> LaunchDescription:
    default_params = str(Path(get_package_share_directory("warotrans_slam")) / "config" / "slam_toolbox.yaml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("posegraph", description="Pose graph path without extension (.posegraph/.data)"),
            DeclareLaunchArgument("slam_params_file", default_value=default_params),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
