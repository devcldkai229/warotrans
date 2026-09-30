"""Launch SLAM Toolbox in online asynchronous mapping mode.

Yêu cầu đầu vào đã PASS trước khi chạy file này:

    /scan       ổn định từ warotrans_sensors
    /odom       ổn định từ warotrans_hardware
    TF          odom -> base_footprint -> base_link -> laser

Node này là publisher DUY NHẤT của map -> odom.

Trên Jazzy, async_slam_toolbox_node là lifecycle node: phải configure +
activate mới publish /map và TF map->odom (giống online_async_launch upstream).
"""

from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def _launch_setup(context: LaunchContext, *args: object, **kwargs: object):
    del args, kwargs

    params_file = Path(context.launch_configurations["params_file"]).expanduser()
    if not params_file.is_file():
        raise RuntimeError(
            f"slam_toolbox params_file not found: {params_file}. "
            "Check the warotrans_slam install, or pass params_file:=<path>."
        )

    use_sim_time = context.launch_configurations["use_sim_time"].lower() == "true"
    autostart = context.launch_configurations["autostart"]
    use_lifecycle_manager = context.launch_configurations["use_lifecycle_manager"]

    slam_node = LifecycleNode(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            str(params_file),
            {
                "use_sim_time": use_sim_time,
                "use_lifecycle_manager": use_lifecycle_manager.lower() == "true",
            },
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            context.launch_configurations["log_level"],
        ],
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="[LifecycleLaunch] slam_toolbox activating."),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    return [slam_node, configure_event, activate_event]


def generate_launch_description() -> LaunchDescription:
    default_params = str(
        Path(get_package_share_directory("warotrans_slam"))
        / "config"
        / "slam_toolbox.yaml"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="YAML file with slam_toolbox mapping parameters",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Real robot uses wall clock; only true in simulation",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="slam_toolbox node log level",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Configure+activate slam_toolbox without external lifecycle manager",
            ),
            DeclareLaunchArgument(
                "use_lifecycle_manager",
                default_value="false",
                description="If true, leave lifecycle to an external manager (no autostart)",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
