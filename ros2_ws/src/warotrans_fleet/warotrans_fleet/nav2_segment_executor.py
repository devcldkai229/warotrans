"""Nav2 ComputeRoute + FollowPath / FollowPathReverse / Spin for Active Mode segments.

Uses existing nav2_route_client for ComputeRoute/adjust_edges coordination and
nav2_msgs actions for tracking without BT recoveries.

Maneuver kinds (CompilerVersion 10+):
  Forward — densify polylines, FollowPath, allow_reversing=false (primary controller)
  Reverse — densify with reversed vertex order, FollowPathReverse (dedicated RPP)
  Spin    — nav2_msgs/Spin with spin_dist from yaw delta; requires authority_ok
"""

from __future__ import annotations

import math
import threading
from typing import Any, Callable, Optional, Sequence

from warotrans_fleet.authority_store import ManeuverStep
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
CONTROLLER_FORWARD = "FollowPath"
CONTROLLER_REVERSE = "FollowPathReverse"


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


def densify_path_reversed(
    polylines: Sequence[Sequence[tuple[float, float]]], step: float = PATH_STEP_M
) -> list[tuple[float, float, float]]:
    """Densify with reversed vertex order so RPP follow path drives reverse along the lane."""
    reversed_lines: list[list[tuple[float, float]]] = []
    for line in reversed(list(polylines)):
        pts = [(float(x), float(y)) for x, y in line]
        pts.reverse()
        reversed_lines.append(pts)
    return densify_path(reversed_lines, step=step)


def _normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class Nav2SegmentExecutor:
    """ROS2 adapter. Construct only when rclpy/nav2_msgs are available."""

    def __init__(
        self,
        *,
        node: Any,
        route_port: Any,
        follow_path_action: str = "follow_path",
        spin_action: str = "spin",
        goal_timeout_sec: float = 120.0,
        authority_ok_fn: Optional[Callable[[], bool]] = None,
        get_yaw_fn: Optional[Callable[[], Optional[float]]] = None,
        log: Optional[Callable[[str], None]] = None,
    ) -> None:
        self._node = node
        self._route_port = route_port
        self._log = log or (lambda _m: None)
        self._goal_timeout_sec = goal_timeout_sec
        self._authority_ok_fn = authority_ok_fn
        self._get_yaw_fn = get_yaw_fn
        self._lock = threading.Lock()
        self._ticket: Optional[SegmentTicket] = None
        self._callbacks: Optional[SegmentCallbacks] = None
        self._send_goal_future = None
        self._result_future = None
        self._goal_handle = None
        self._pending_steps: list[ManeuverStep] = []
        self._active_controller = CONTROLLER_FORWARD
        self._spin_client = None

        from action_msgs.msg import GoalStatus
        from nav2_msgs.action import FollowPath
        from rclpy.action import ActionClient
        from rclpy.duration import Duration

        self._GoalStatus = GoalStatus
        self._FollowPath = FollowPath
        self._Duration = Duration
        self._client = ActionClient(node, FollowPath, follow_path_action)
        try:
            from nav2_msgs.action import Spin

            self._Spin = Spin
            self._spin_client = ActionClient(node, Spin, spin_action)
        except Exception as exc:  # noqa: BLE001
            self._log(f"[Nav2SegmentExecutor] Spin action unavailable: {exc!r}")
            self._Spin = None
        self._edge_poly: dict[int, list[tuple[float, float]]] = {}

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
        """Legacy: single Forward step over edge_ids."""
        return self.execute_maneuvers(
            [ManeuverStep(kind="Forward", edge_ids=tuple(int(e) for e in edge_ids))],
            callbacks,
        )

    def execute_maneuvers(
        self, steps: Sequence[ManeuverStep], callbacks: SegmentCallbacks
    ) -> SegmentTicket:
        all_edges: list[int] = []
        for step in steps:
            all_edges.extend(int(e) for e in step.edge_ids)
        ticket = SegmentTicket(all_edges)
        with self._lock:
            self._ticket = ticket
            self._callbacks = callbacks
            self._pending_steps = list(steps)

        if not steps:
            callbacks.on_result(SegmentResult(RESULT_SUCCEEDED, None))
            ticket.finished = True
            return ticket

        self._start_next_step()
        return ticket

    def cancel(self) -> None:
        with self._lock:
            ticket = self._ticket
            handle = self._goal_handle
            self._pending_steps = []
            if ticket is not None:
                ticket.canceled = True
        if handle is not None:
            try:
                handle.cancel_goal_async()
            except Exception as exc:  # noqa: BLE001
                self._log(f"[Nav2SegmentExecutor] cancel failed: {exc!r}")

    def _require_authority(self) -> bool:
        if self._authority_ok_fn is None:
            return True
        try:
            return bool(self._authority_ok_fn())
        except Exception:  # noqa: BLE001
            return False

    def _start_next_step(self) -> None:
        with self._lock:
            if self._ticket is not None and self._ticket.canceled:
                self._finish(RESULT_CANCELED, None)
                return
            if not self._pending_steps:
                self._finish(RESULT_SUCCEEDED, None)
                return
            step = self._pending_steps.pop(0)

        if step.kind not in ("Forward", "Reverse", "Spin"):
            self._log(f"[Nav2SegmentExecutor] unknown maneuver kind={step.kind!r}; refusing")
            self._finish(RESULT_ABORTED, "UNKNOWN_MANEUVER")
            return

        if step.kind in ("Reverse", "Spin") and not self._require_authority():
            self._finish(RESULT_ABORTED, "AUTHORITY_REQUIRED")
            return

        if step.kind == "Spin":
            self._start_spin(step)
            return

        controller = CONTROLLER_REVERSE if step.kind == "Reverse" else CONTROLLER_FORWARD
        self._start_follow(list(step.edge_ids), controller, reverse=step.kind == "Reverse")

    def _start_follow(self, edge_ids: list[int], controller_id: str, *, reverse: bool) -> None:
        if not self._client.wait_for_server(timeout_sec=2.0):
            self._finish(RESULT_ABORTED, "FOLLOW_PATH_UNAVAILABLE")
            return

        missing = [e for e in edge_ids if not self._edge_points(int(e))]
        if missing:
            self._log(f"[Nav2SegmentExecutor] no geometry for edges {missing}; refusing FollowPath")
            self._finish(RESULT_ABORTED, "UNKNOWN_EDGE")
            return

        path_msg = self._build_path_msg(edge_ids, reverse=reverse)
        goal = self._FollowPath.Goal()
        goal.path = path_msg
        goal.controller_id = controller_id
        goal.goal_checker_id = "general_goal_checker"
        self._active_controller = controller_id

        self._send_goal_future = self._client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._on_goal_response)

    def _start_spin(self, step: ManeuverStep) -> None:
        if self._spin_client is None or self._Spin is None:
            self._finish(RESULT_ABORTED, "SPIN_UNAVAILABLE")
            return
        if not self._spin_client.wait_for_server(timeout_sec=2.0):
            self._finish(RESULT_ABORTED, "SPIN_UNAVAILABLE")
            return
        if step.yaw_target is None:
            self._finish(RESULT_ABORTED, "SPIN_YAW_REQUIRED")
            return

        current = None
        if self._get_yaw_fn is not None:
            try:
                current = self._get_yaw_fn()
            except Exception:  # noqa: BLE001
                current = None
        if current is None:
            current = 0.0
        spin_dist = _normalize_angle(float(step.yaw_target) - float(current))

        goal = self._Spin.Goal()
        goal.target_yaw = spin_dist  # nav2 Spin uses relative spin_dist on some distros
        # Jazzy Spin.action field is `target_yaw` (relative). Prefer setattr for both names.
        if hasattr(goal, "spin_dist"):
            goal.spin_dist = spin_dist  # type: ignore[attr-defined]
        self._active_controller = "Spin"
        self._send_goal_future = self._spin_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self._on_goal_response)

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
            has_more = bool(self._pending_steps)

        if canceled or status == self._GoalStatus.STATUS_CANCELED:
            self._finish(RESULT_CANCELED, None)
            return
        if status == self._GoalStatus.STATUS_SUCCEEDED:
            if has_more:
                with self._lock:
                    self._goal_handle = None
                self._start_next_step()
            else:
                self._finish(RESULT_SUCCEEDED, None)
            return
        if status == self._GoalStatus.STATUS_ABORTED:
            self._finish(RESULT_ABORTED, "CONTROLLER_ABORTED")
            return
        self._finish(
            RESULT_TIMEOUT if status == self._GoalStatus.STATUS_UNKNOWN else RESULT_ABORTED,
            f"status={status}",
        )

    def _finish(self, status: str, detail: Optional[str]) -> None:
        with self._lock:
            callbacks = self._callbacks
            ticket = self._ticket
            self._goal_handle = None
            self._pending_steps = []
            if ticket is not None:
                ticket.finished = True
        if callbacks is not None:
            callbacks.on_result(SegmentResult(status, detail))

    def _edge_points(self, edge_id: int) -> list[tuple[float, float]]:
        if edge_id in self._edge_poly:
            return list(self._edge_poly[edge_id])
        get_poly = getattr(self._route_port, "edge_polyline", None)
        if callable(get_poly):
            try:
                return list(get_poly(edge_id) or [])
            except Exception:  # noqa: BLE001
                return []
        return []

    def _build_path_msg(self, edge_ids: list[int], *, reverse: bool = False) -> Any:
        from geometry_msgs.msg import PoseStamped
        from nav_msgs.msg import Path

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self._node.get_clock().now().to_msg()

        lines = [self._edge_points(eid) for eid in edge_ids]
        poses = densify_path_reversed(lines) if reverse else densify_path(lines)
        for x, y, yaw in poses:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        if not path.poses:
            self._log(f"[Nav2SegmentExecutor] empty FollowPath for edges={edge_ids} reverse={reverse}")
        return path
