"""NavigatorPort implementation bằng rclpy ActionClient(nav2_msgs/action/NavigateToPose).

Mọi lời gọi phải xảy ra trên ROS executor thread của node (robot_gateway_node đảm bảo điều này).
Callback từ future được bọc try/except: exception trong done-callback của rclpy sẽ
lan lên executor và làm chết node.
"""

from __future__ import annotations

from typing import Any, Callable, Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

from warotrans_fleet.navigator import (
    STATUS_UNKNOWN,
    GoalCallbacks,
    GoalTicket,
    NavFeedback,
    NavGoal,
    NavResult,
)
from warotrans_fleet.transforms import yaw_to_quaternion

# action_msgs/srv/CancelGoal.Response.return_code
_CANCEL_RETURN_CODES = {0: "ERROR_NONE", 1: "ERROR_REJECTED", 2: "ERROR_UNKNOWN_GOAL_ID", 3: "ERROR_GOAL_TERMINATED"}


def _duration_sec(msg: Any) -> Optional[float]:
    if msg is None:
        return None
    return float(msg.sec) + float(msg.nanosec) * 1e-9


class Nav2ActionNavigator:
    def __init__(self, node: Node, action_name: str) -> None:
        self._node = node
        self._log = node.get_logger()
        self._client = ActionClient(node, NavigateToPose, action_name)

    def is_ready(self) -> bool:
        return bool(self._client.server_is_ready())

    def send_goal(self, goal: NavGoal, ticket: GoalTicket, callbacks: GoalCallbacks) -> None:
        pose = PoseStamped()
        pose.header.frame_id = goal.frame_id
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = float(goal.x)
        pose.pose.position.y = float(goal.y)
        qx, qy, qz, qw = yaw_to_quaternion(goal.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        msg = NavigateToPose.Goal()
        msg.pose = pose

        def _on_feedback(feedback_msg: Any) -> None:
            fb = feedback_msg.feedback
            self._safe(
                "feedback",
                goal.command_id,
                callbacks.on_feedback,
                NavFeedback(
                    distance_remaining=float(fb.distance_remaining),
                    navigation_time_sec=_duration_sec(getattr(fb, "navigation_time", None)),
                    estimated_time_remaining_sec=_duration_sec(getattr(fb, "estimated_time_remaining", None)),
                    number_of_recoveries=int(getattr(fb, "number_of_recoveries", 0)),
                ),
            )

        def _on_result(future: Any) -> None:
            try:
                wrapped = future.result()
                res = wrapped.result
                nav_result = NavResult(
                    status=int(wrapped.status),
                    error_code=int(getattr(res, "error_code", 0)) if hasattr(res, "error_code") else None,
                    error_message=(getattr(res, "error_msg", "") or None),
                )
            except Exception as exc:  # noqa: BLE001
                nav_result = NavResult(status=STATUS_UNKNOWN, error_message=f"get_result failed: {exc}")
            self._safe("result", goal.command_id, callbacks.on_result, nav_result)

        def _on_goal_response(future: Any) -> None:
            try:
                handle = future.result()
            except Exception as exc:  # noqa: BLE001
                self._safe("response", goal.command_id, callbacks.on_response, False, f"{type(exc).__name__}: {exc}")
                return
            if handle is None or not handle.accepted:
                self._safe("response", goal.command_id, callbacks.on_response, False, None)
                return
            ticket.handle = handle
            self._safe("response", goal.command_id, callbacks.on_response, True, None)
            handle.get_result_async().add_done_callback(_on_result)

        self._client.send_goal_async(msg, feedback_callback=_on_feedback).add_done_callback(_on_goal_response)

    def cancel_goal(self, ticket: GoalTicket, on_response: Callable[[bool, str], None]) -> None:
        handle = ticket.handle
        if handle is None:
            on_response(False, "goal not accepted yet")
            return

        def _done(future: Any) -> None:
            try:
                code = int(future.result().return_code)
            except Exception as exc:  # noqa: BLE001
                self._safe("cancel", "", on_response, False, f"cancel request failed: {exc}")
                return
            self._safe("cancel", "", on_response, code == 0, _CANCEL_RETURN_CODES.get(code, str(code)))

        handle.cancel_goal_async().add_done_callback(_done)

    def _safe(self, what: str, command_id: str, fn: Callable[..., None], *args: Any) -> None:
        try:
            fn(*args)
        except Exception as exc:  # noqa: BLE001
            self._log.error(f"[Nav2CallbackError] {what} commandId={command_id} {type(exc).__name__}: {exc}")

