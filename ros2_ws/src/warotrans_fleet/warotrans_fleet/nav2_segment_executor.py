"""Nav2 ComputeRoute + FollowPath adapter for Active Mode segment execution.

Uses existing nav2_route_client for ComputeRoute/adjust_edges coordination and
nav2_msgs/action/FollowPath for controller tracking without BT recoveries.
"""

from __future__ import annotations

import math
import threading
from typing import Any, Callable, Optional, Sequence

from warotrans_fleet.segment_executor import (
    RESULT_ABORTED,
    RESULT_CANCELED,
    RESULT_SUCCEEDED,
    RESULT_TIMEOUT,
    Pose2D,
    SegmentCallbacks,
    SegmentResult,
    SegmentTicket,
)


PATH_STEP_M = 0.05


def densify_path(
    polylines: Sequence[Sequence[tuple[float, float]]], step: float = PATH_STEP_M
) -> list[tuple[float, float, float]]:
    """Chain edge polylines into evenly spaced (x, y, yaw) poses along the lane centre line.

    Shared vertices between consecutive edges are emitted once; each pose faces along its sub-segment so
    the controller never sees a zero quaternion or a jump between sparse vertices.
    """
    vertices: list[tuple[float, float]] = []
    for line in polylines:
        for x, y in line:
            point = (float(x), float(y))
            if not vertices or math.dist(vertices[-1], point) > 1e-6:
                vertices.append(point)
    if len(vertices) < 2:
        return [(x, y, 0.0) for x, y in vertices]

    poses: list[tuple[float, float, float]] = []
    for (x0, y0), (x1, y1) in zip(vertices, vertices[1:]):
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        steps = max(1, math.ceil(length / step))
        for i in range(steps):
            t = i / steps
            poses.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t, yaw))
    poses.append((vertices[-1][0], vertices[-1][1], poses[-1][2]))
    return poses


class Nav2SegmentExecutor:
    """ROS2 adapter. Construct only when rclpy/nav2_msgs are available."""

    def __init__(
        self,
        *,
        node: Any,
        route_port: Any,
        follow_path_action: str = "follow_path",
        goal_timeout_sec: float = 120.0,
        log: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._node = node
        self._route_port = route_port
        self._log = log or (lambda _m: None)
        self._goal_timeout_sec = goal_timeout_sec
        self._lock = threading.Lock()
        self._ticket: Optional[SegmentTicket] = None
        self._callbacks: Optional[SegmentCallbacks] = None
        self._send_goal_future = None
        self._result_future = None
        self._goal_handle = None

        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import FollowPath
        from rclpy.action import ActionClient
        from rclpy.duration import Duration

        self._GoalStatus = GoalStatus
        self._FollowPath = FollowPath
        self._Duration = Duration
        self._client = ActionClient(node, FollowPath, follow_path_action)
        self._edge_poly: dict[int, list[tuple[float, float]]] = {}
        # Optional: inject polylines via route_port.edge_polyline or set_edge_polylines().

    def set_edge_polylines(self, polylines: dict[int, Sequence[tuple[float, float]]]) -> None:
        self._edge_poly = {int(k): [(float(x), float(y)) for x, y in v] for k, v in polylines.items()}

    def compute_route_edge_ids(self, start: Pose2D, goal: Pose2D) -> list[int]:
        if self._route_port is None:
            return []
        try:
            return list(self._route_port.compute_route_edge_ids(start, goal) or [])
        except Exception as exc:  # noqa: BLE001
            self._log(f"[Nav2SegmentExecutor] compute_route failed: {exc!r}")
            return []

    def follow_path(self, edge_ids: Sequence[int], callbacks: SegmentCallbacks) -> SegmentTicket:
        ticket = SegmentTicket(edge_ids)
        with self._lock:
            self._ticket = ticket
            self._callbacks = callbacks

        if not self._client.wait_for_server(timeout_sec=2.0):
            callbacks.on_result(SegmentResult(RESULT_ABORTED, "FOLLOW_PATH_UNAVAILABLE"))
            ticket.finished = True
            return ticket

        # Build a sparse path from route_port geometry when available; otherwise empty path
        # (controller may reject — reported as ABORTED).
        path_msg = self._build_path_msg(list(edge_ids))
        goal = self._FollowPath.Goal()
        goal.path = path_msg
        # Must match controller_server plugin name (nav2_params); empty string aborts on Jazzy.
        goal.controller_id = "FollowPath"
        goal.goal_checker_id = "general_goal_checker"

        self._send_goal_future = self._client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._on_goal_response)
        return ticket

    def cancel(self) -> None:
        with self._lock:
            ticket = self._ticket
            handle = self._goal_handle
            if ticket is not None:
                ticket.canceled = True
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self._log(f"[Nav2SegmentExecutor] cancel failed: {exc!r}")

    def _on_goal_response(self, future: Any) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001
            self._finish(RESULT_ABORTED, f"goal_response_error:{exc!r}")
            return
        if handle is None or not handle.accepted:
            self._finish(RESULT_ABORTED, "GOAL_REJECTED")
            return
        with self._lock:
            self._goal_handle = handle
            ticket = self._ticket
            if ticket is not None:
                ticket.handle = handle
        self._result_future = handle.get_result_async()
        self._result_future.add_done_callback(self._on_result)

    def _on_result(self, future: Any) -> None:
        try:
            wrapped = future.result()
            status = int(wrapped.status)
        except Exception as exc:  # noqa: BLE001
            self._finish(RESULT_ABORTED, f"result_error:{exc!r}")
            return

        with self._lock:
            canceled = self._ticket is not None and self._ticket.canceled

        if status == self._GoalStatus.STATUS_SUCCEEDED:
            self._finish(RESULT_SUCCEEDED, None)
        elif canceled or status == self._GoalStatus.STATUS_CANCELED:
            self._finish(RESULT_CANCELED, None)
        elif status == self._GoalStatus.STATUS_ABORTED:
            self._finish(RESULT_ABORTED, "CONTROLLER_ABORTED")
        else:
            self._finish(RESULT_TIMEOUT if status == self._GoalStatus.STATUS_UNKNOWN else RESULT_ABORTED, f"status={status}")

    def _finish(self, status: str, detail: Optional[str]) -> None:
        with self._lock:
            callbacks = self._callbacks
            ticket = self._ticket
            self._goal_handle = None
            if ticket is not None:
                ticket.finished = True
        if callbacks is not None:
            callbacks.on_result(SegmentResult(status, detail))

    def _build_path_msg(self, edge_ids: list[int]) -> Any:
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self._node.get_clock().now().to_msg()
        get_poly = getattr(self._route_port, "edge_polyline", None)

        def _points(eid: int) -> list[tuple[float, float]]:
            if eid in self._edge_poly:
                return list(self._edge_poly[eid])
            if callable(get_poly):
                try:
                    return list(get_poly(eid) or [])
                except Exception:  # noqa: BLE001
                    return []
            return []

        for x, y, yaw in densify_path([_points(eid) for eid in edge_ids]):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        if not path.poses:
            self._log(f"[Nav2SegmentExecutor] empty FollowPath for edges={edge_ids}")
        return path
