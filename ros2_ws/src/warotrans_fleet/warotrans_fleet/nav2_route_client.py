"""rclpy clients for nav2_msgs DynamicEdges + ComputeRoute (optional — import may fail off-robot)."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING, Optional, Sequence

if TYPE_CHECKING:
    from rclpy.node import Node


class Nav2RouteGraphClient:
    """Blocks briefly on service/action calls from the ROS executor thread (same pattern as Nav2 goal send)."""

    def __init__(
        self,
        node: "Node",
        *,
        adjust_edges_service: str = "route_server/DynamicEdgesScorer/adjust_edges",
        compute_route_action: str = "compute_route",
        timeout_sec: float = 5.0,
    ) -> None:
        from nav2_msgs.action import ComputeRoute
        from nav2_msgs.srv import DynamicEdges
        from rclpy.action import ActionClient

        self._node = node
        self._timeout = float(timeout_sec)
        self._dyn = node.create_client(DynamicEdges, adjust_edges_service)
        self._compute = ActionClient(node, ComputeRoute, compute_route_action)
        self._DynamicEdges = DynamicEdges
        self._ComputeRoute = ComputeRoute

    def adjust_edges(self, *, close_ids: Sequence[int], open_ids: Sequence[int]) -> bool:
        if not self._dyn.wait_for_service(timeout_sec=self._timeout):
            self._node.get_logger().error("DynamicEdges service not available")
            return False
        req = self._DynamicEdges.Request()
        req.closed_edges = [int(i) & 0xFFFF for i in close_ids]
        req.opened_edges = [int(i) & 0xFFFF for i in open_ids]
        future = self._dyn.call_async(req)
        if not self._spin_until(future):
            return False
        result = future.result()
        return bool(result and result.success)

    def compute_route_edge_ids(
        self,
        *,
        frame_id: str,
        start_x: float,
        start_y: float,
        start_yaw: float,
        goal_x: float,
        goal_y: float,
        goal_yaw: float,
    ) -> list[int]:
        if not self._compute.wait_for_server(timeout_sec=self._timeout):
            raise RuntimeError("ComputeRoute action server not available")

        goal = self._ComputeRoute.Goal()
        goal.use_poses = True
        goal.use_start = True
        goal.start = _pose(frame_id, start_x, start_y, start_yaw)
        goal.goal = _pose(frame_id, goal_x, goal_y, goal_yaw)

        send_future = self._compute.send_goal_async(goal)
        if not self._spin_until(send_future):
            raise RuntimeError("ComputeRoute send_goal timed out")
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("ComputeRoute goal rejected")

        result_future = goal_handle.get_result_async()
        if not self._spin_until(result_future):
            raise RuntimeError("ComputeRoute result timed out")
        wrapped = result_future.result()
        if wrapped is None:
            raise RuntimeError("ComputeRoute returned no result")
        result = wrapped.result
        if result.error_code != 0:
            raise RuntimeError(f"ComputeRoute error_code={result.error_code} {result.error_msg}")
        return [int(e.edgeid) for e in result.route.edges]

    def _spin_until(self, future) -> bool:
        import time

        deadline = time.monotonic() + self._timeout
        while not future.done():
            if time.monotonic() >= deadline:
                return False
            # Executor thread already spinning the node; brief yield.
            time.sleep(0.01)
        return True


def _pose(frame_id: str, x: float, y: float, yaw: float):
    from geometry_msgs.msg import PoseStamped

    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.orientation.z = math.sin(yaw / 2.0)
    msg.pose.orientation.w = math.cos(yaw / 2.0)
    return msg


def try_create_nav2_route_client(node: "Node", **kwargs) -> Optional[Nav2RouteGraphClient]:
    try:
        return Nav2RouteGraphClient(node, **kwargs)
    except ImportError:
        return None
