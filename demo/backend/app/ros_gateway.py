"""rclpy Nav2 gateway: /map, TF, /plan, NavigateToPose, NavigateThroughPoses, filters.

Phase 0 stable contracts (do not invent alternate topics):
  /map, /plan, /navigate_to_pose, frame map, base_footprint

Phase 3:
  /navigate_through_poses — Lane sequence hints
  KEEP_OUT / SPEED_LIMIT masks via LoadMap on filter mask servers
"""

from __future__ import annotations

import math
import os
import threading
import time
from pathlib import Path
from typing import Callable, List, Optional, Sequence

from .filter_masks import rasterize_keepout, rasterize_speed, write_mask_pgm_yaml
from .lane_nav import build_lane_sequence_poses
from .map_coords import MapMeta
from .map_image import occupancy_to_png
from .session import NavPath, NavStatus, PathPose, SessionStore, store

PLAN_TOPIC = "/plan"
BASE_FRAME = "base_footprint"
MAP_FRAME = "map"

DEFAULT_KEEPOUT_YAML = Path(
    os.path.expanduser(os.environ.get("WARO_KEEPOUT_MASK", "~/maps/warotrans_keepout.yaml"))
)
DEFAULT_SPEED_YAML = Path(
    os.path.expanduser(os.environ.get("WARO_SPEED_MASK", "~/maps/warotrans_speed.yaml"))
)


def _yaw_from_quat(z: float, w: float) -> float:
    return math.atan2(2.0 * w * z, 1.0 - 2.0 * z * z)


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


class NavGateway:
    """Bridge FastAPI session <-> ROS 2 Nav2."""

    def __init__(self, session: SessionStore = store) -> None:
        self.session = session
        self._lock = threading.Lock()
        self._meta: Optional[MapMeta] = None
        self._png: Optional[bytes] = None
        self._node = None
        self._action_client = None
        self._through_client = None
        self._goal_handle = None
        self._tf_buffer = None
        self._tf_listener = None
        self._spin_thread: Optional[threading.Thread] = None
        self._ws_listeners: List[Callable[[dict], None]] = []
        self._started = False
        self._last_tf_wall = 0.0
        self._keepout_yaml = DEFAULT_KEEPOUT_YAML
        self._speed_yaml = DEFAULT_SPEED_YAML

    def add_ws_listener(self, cb: Callable[[dict], None]) -> None:
        self._ws_listeners.append(cb)

    def remove_ws_listener(self, cb: Callable[[dict], None]) -> None:
        try:
            self._ws_listeners.remove(cb)
        except ValueError:
            pass

    def _broadcast(self, payload: dict) -> None:
        for cb in list(self._ws_listeners):
            try:
                cb(payload)
            except Exception:
                pass

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "nav": self.session.nav.model_dump(),
                "robot": self.session.robot.model_dump(),
                "path": self.session.path.model_dump(),
                "map_ready": self._meta is not None,
            }

    def get_meta(self) -> Optional[MapMeta]:
        with self._lock:
            return self._meta

    def get_png(self) -> Optional[bytes]:
        with self._lock:
            return self._png

    def _clear_path(self) -> None:
        self.session.path = NavPath()
        self._broadcast({"type": "path", "path": self.session.path.model_dump()})

    def _set_robot(self, x: float, y: float, yaw: float, stamp_sec: float) -> None:
        self.session.robot.x = float(x)
        self.session.robot.y = float(y)
        self.session.robot.yaw = float(yaw)
        self.session.robot.timestamp = float(stamp_sec)
        self.session.robot.valid = True
        self._broadcast(
            {
                "type": "robot",
                "robot": {
                    "x": self.session.robot.x,
                    "y": self.session.robot.y,
                    "yaw": self.session.robot.yaw,
                    "timestamp": self.session.robot.timestamp,
                    "valid": True,
                },
            }
        )

    def start(self) -> None:
        if self._started:
            return
        import rclpy
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
        from nav2_msgs.msg import SpeedLimit
        from nav_msgs.msg import OccupancyGrid, Path
        from rclpy.action import ActionClient
        from rclpy.duration import Duration
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from tf2_ros import Buffer, TransformListener

        if not rclpy.ok():
            rclpy.init(args=None)

        outer = self

        class GatewayNode(Node):
            def __init__(self_node) -> None:
                super().__init__("warotrans_demo_gateway")
                map_qos = QoSProfile(
                    depth=1,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                )
                self_node.create_subscription(
                    OccupancyGrid, "/map", self_node._on_map, map_qos
                )
                self_node.create_subscription(
                    PoseWithCovarianceStamped,
                    "/amcl_pose",
                    self_node._on_amcl,
                    10,
                )
                self_node.create_subscription(
                    Path, PLAN_TOPIC, self_node._on_plan, 10
                )
                self_node.create_subscription(
                    Path, "/received_global_plan", self_node._on_plan, 10
                )
                self_node.create_subscription(
                    SpeedLimit, "/speed_limit", self_node._on_speed_limit, 10
                )
                self_node.create_timer(0.1, self_node._on_tf_timer)
                self_node._outer = outer

            def _on_map(self_node, msg: OccupancyGrid) -> None:
                info = msg.info
                yaw = _yaw_from_quat(
                    info.origin.orientation.z, info.origin.orientation.w
                )
                if abs(yaw) < 1e-6:
                    yaw = 0.0
                meta = MapMeta(
                    width=int(info.width),
                    height=int(info.height),
                    resolution=float(info.resolution),
                    origin_x=float(info.origin.position.x),
                    origin_y=float(info.origin.position.y),
                    origin_yaw=float(yaw),
                )
                try:
                    png = occupancy_to_png(list(msg.data), meta)
                except ValueError as exc:
                    self_node.get_logger().error(f"map encode failed: {exc}")
                    return
                with outer._lock:
                    outer._meta = meta
                    outer._png = png
                outer._broadcast({"type": "map_updated", "meta": meta.to_dict()})
                self_node.get_logger().info(
                    f"Map received {meta.width}x{meta.height} @ {meta.resolution} m"
                    f" origin_yaw={meta.origin_yaw:.4f}"
                )

            def _on_amcl(self_node, msg: PoseWithCovarianceStamped) -> None:
                if time.time() - outer._last_tf_wall < 0.5:
                    return
                p = msg.pose.pose
                yaw = _yaw_from_quat(p.orientation.z, p.orientation.w)
                stamp = float(msg.header.stamp.sec) + float(
                    msg.header.stamp.nanosec
                ) * 1e-9
                if stamp <= 0.0:
                    stamp = time.time()
                outer._set_robot(p.position.x, p.position.y, yaw, stamp)

            def _on_plan(self_node, msg: Path) -> None:
                try:
                    poses: List[PathPose] = []
                    for ps in msg.poses:
                        p = ps.pose
                        yaw = _yaw_from_quat(p.orientation.z, p.orientation.w)
                        poses.append(
                            PathPose(
                                x=float(p.position.x),
                                y=float(p.position.y),
                                yaw=float(yaw),
                            )
                        )
                    if not poses:
                        return
                    frame = msg.header.frame_id or MAP_FRAME
                    outer.session.path = NavPath(frame_id=frame, poses=poses)
                    outer._broadcast(
                        {"type": "path", "path": outer.session.path.model_dump()}
                    )
                except Exception as exc:
                    self_node.get_logger().error(f"plan callback failed: {exc}")

            def _on_speed_limit(self_node, msg: SpeedLimit) -> None:
                update = {
                    "speed_limit_mps": None if msg.percentage else float(msg.speed_limit),
                    "speed_limit_percent": (
                        float(msg.speed_limit) if msg.percentage else None
                    ),
                }
                outer.session.nav = outer.session.nav.model_copy(update=update)
                outer._broadcast(
                    {
                        "type": "speed_limit",
                        "speed_limit_mps": update["speed_limit_mps"],
                        "speed_limit_percent": update["speed_limit_percent"],
                    }
                )

            def _on_tf_timer(self_node) -> None:
                if outer._tf_buffer is None:
                    return
                try:
                    tf = outer._tf_buffer.lookup_transform(
                        MAP_FRAME,
                        BASE_FRAME,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.05),
                    )
                except Exception:
                    return
                t = tf.transform.translation
                r = tf.transform.rotation
                yaw = _yaw_from_quat(r.z, r.w)
                stamp = float(tf.header.stamp.sec) + float(
                    tf.header.stamp.nanosec
                ) * 1e-9
                if stamp <= 0.0:
                    stamp = time.time()
                outer._last_tf_wall = time.time()
                outer._set_robot(t.x, t.y, yaw, stamp)

        self._node = GatewayNode()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self._node)
        self._action_client = ActionClient(
            self._node, NavigateToPose, "navigate_to_pose"
        )
        self._through_client = ActionClient(
            self._node, NavigateThroughPoses, "navigate_through_poses"
        )
        self._spin_thread = threading.Thread(
            target=rclpy.spin, args=(self._node,), daemon=True, name="rclpy-spin"
        )
        self._spin_thread.start()
        self._started = True
        self._node.get_logger().info(
            f"warotrans_demo_gateway started (plan={PLAN_TOPIC}, "
            f"tf={MAP_FRAME}->{BASE_FRAME}, through_poses=on)"
        )

    def stop(self) -> None:
        if not self._started:
            return
        import rclpy

        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        self._tf_buffer = None
        self._tf_listener = None
        if rclpy.ok():
            rclpy.shutdown()
        self._started = False

    def _cancel_active_goal(self) -> None:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None

    def navigate_to_endpoint(self, endpoint_id: str) -> dict:
        ep = self.session.get(endpoint_id)
        if ep is None:
            raise KeyError(f"endpoint not found: {endpoint_id}")
        if not ep.enabled:
            raise RuntimeError(f"endpoint disabled: {endpoint_id}")
        if self._node is None or self._action_client is None:
            raise RuntimeError("ROS gateway not started")

        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose

        self._cancel_active_goal()

        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.session.nav = self.session.nav.model_copy(
                update={
                    "status": NavStatus.FAILED,
                    "endpoint_id": endpoint_id,
                    "mode": "endpoint",
                    "lane_ids": [],
                    "message": "navigate_to_pose action server not available",
                    "goal_x": ep.x,
                    "goal_y": ep.y,
                    "goal_yaw": ep.yaw,
                }
            )
            self._broadcast({"type": "nav", "nav": self.session.nav.model_dump()})
            raise TimeoutError("Nav2 action server not available")

        pose = PoseStamped()
        pose.header.frame_id = MAP_FRAME
        pose.header.stamp = self._node.get_clock().now().to_msg()
        pose.pose.position.x = ep.x
        pose.pose.position.y = ep.y
        pose.pose.position.z = 0.0
        qx, qy, qz, qw = _quat_from_yaw(ep.yaw)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.session.nav = self.session.nav.model_copy(
            update={
                "status": NavStatus.NAVIGATING,
                "endpoint_id": endpoint_id,
                "mode": "endpoint",
                "lane_ids": [],
                "message": f"Navigating to {ep.name}",
                "goal_x": ep.x,
                "goal_y": ep.y,
                "goal_yaw": ep.yaw,
            }
        )
        self._broadcast({"type": "nav", "nav": self.session.nav.model_dump()})
        self._broadcast(
            {
                "type": "goal",
                "goal": {"x": ep.x, "y": ep.y, "yaw": ep.yaw, "frame_id": MAP_FRAME},
            }
        )

        send_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(
            lambda fut: self._on_goal_response(fut, endpoint_id)
        )
        return self.session.nav.model_dump()

    def navigate_lanes(
        self, lane_ids: Sequence[str], *, spacing_m: float = 0.35
    ) -> dict:
        if self._node is None or self._through_client is None:
            raise RuntimeError("ROS gateway not started")
        lanes = []
        for lid in lane_ids:
            ln = self.session.get_lane(lid)
            if ln is None:
                raise KeyError(f"lane not found: {lid}")
            lanes.append(ln)
        robot = None
        if self.session.robot.valid:
            robot = (self.session.robot.x, self.session.robot.y)
        poses = build_lane_sequence_poses(
            lanes,
            junctions=self.session.list_zones(),
            robot=robot,
            spacing_m=spacing_m,
        )

        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateThroughPoses

        self._cancel_active_goal()
        if not self._through_client.wait_for_server(timeout_sec=5.0):
            raise TimeoutError("navigate_through_poses action server not available")

        stamp = self._node.get_clock().now().to_msg()
        goal = NavigateThroughPoses.Goal()
        for p in poses:
            ps = PoseStamped()
            ps.header.frame_id = MAP_FRAME
            ps.header.stamp = stamp
            ps.pose.position.x = float(p["x"])
            ps.pose.position.y = float(p["y"])
            qx, qy, qz, qw = _quat_from_yaw(float(p["yaw"]))
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            goal.poses.append(ps)

        last = poses[-1]
        names = ", ".join(ln.name for ln in lanes)
        self.session.nav = self.session.nav.model_copy(
            update={
                "status": NavStatus.NAVIGATING,
                "endpoint_id": None,
                "mode": "lanes",
                "lane_ids": list(lane_ids),
                "message": f"Lane nav ({len(poses)} poses): {names}",
                "goal_x": last["x"],
                "goal_y": last["y"],
                "goal_yaw": last["yaw"],
            }
        )
        self._broadcast({"type": "nav", "nav": self.session.nav.model_dump()})
        self._broadcast(
            {
                "type": "lane_poses",
                "poses": poses,
                "lane_ids": list(lane_ids),
            }
        )

        send_future = self._through_client.send_goal_async(
            goal, feedback_callback=self._on_feedback
        )
        send_future.add_done_callback(
            lambda fut: self._on_goal_response(fut, f"lanes:{','.join(lane_ids)}")
        )
        return self.session.nav.model_dump()

    def sync_filters(self) -> dict:
        """Rasterize KEEP_OUT / SPEED_LIMIT → mask files → LoadMap services."""
        meta = self.get_meta()
        if meta is None:
            raise RuntimeError("map meta not available yet")
        zones = self.session.list_zones()
        keepout_data = rasterize_keepout(meta, zones)
        speed_data = rasterize_speed(meta, zones)
        write_mask_pgm_yaml(meta, keepout_data, self._keepout_yaml)
        write_mask_pgm_yaml(meta, speed_data, self._speed_yaml)

        result = {
            "keepout_yaml": str(self._keepout_yaml),
            "speed_yaml": str(self._speed_yaml),
            "keepout_cells": sum(1 for v in keepout_data if v >= 100),
            "speed_cells": sum(1 for v in speed_data if v > 0),
            "load_keepout": None,
            "load_speed": None,
        }
        if self._node is not None:
            result["load_keepout"] = self._load_map(
                "/keepout_filter_mask_server/load_map", self._keepout_yaml
            )
            result["load_speed"] = self._load_map(
                "/speed_filter_mask_server/load_map", self._speed_yaml
            )
        self.session.nav = self.session.nav.model_copy(
            update={"filters_applied_at": time.time()}
        )
        self._broadcast({"type": "filters_synced", **result})
        return result

    def _load_map(self, service_name: str, yaml_path: Path) -> str:
        from nav2_msgs.srv import LoadMap
        from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

        assert self._node is not None
        client = self._node.create_client(
            LoadMap, service_name, callback_group=MutuallyExclusiveCallbackGroup()
        )
        if not client.wait_for_service(timeout_sec=3.0):
            self._node.destroy_client(client)
            return "service_unavailable"
        req = LoadMap.Request()
        req.map_url = str(yaml_path)
        future = client.call_async(req)
        deadline = time.time() + 8.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        self._node.destroy_client(client)
        if not future.done():
            return "timeout"
        try:
            resp = future.result()
            return f"result={getattr(resp, 'result', resp)}"
        except Exception as exc:
            return f"error:{exc}"

    def cancel(self) -> dict:
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception as exc:
                self.session.nav = self.session.nav.model_copy(
                    update={
                        "status": NavStatus.FAILED,
                        "message": f"cancel failed: {exc}",
                    }
                )
                self._broadcast(
                    {"type": "nav", "nav": self.session.nav.model_dump()}
                )
                return self.session.nav.model_dump()
        self.session.nav = self.session.nav.model_copy(
            update={
                "status": NavStatus.CANCELLED,
                "message": "Cancelled by operator",
            }
        )
        self._clear_path()
        self._broadcast({"type": "nav", "nav": self.session.nav.model_dump()})
        return self.session.nav.model_dump()

    def _on_feedback(self, feedback_msg) -> None:
        del feedback_msg

    def _on_goal_response(self, future, endpoint_id: str) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.session.nav = self.session.nav.model_copy(
                update={
                    "status": NavStatus.FAILED,
                    "message": str(exc),
                }
            )
            self._broadcast({"type": "nav", "nav": self.session.nav.model_dump()})
            return

        if not goal_handle.accepted:
            self.session.nav = self.session.nav.model_copy(
                update={
                    "status": NavStatus.FAILED,
                    "message": "Goal rejected by Nav2",
                }
            )
            self._broadcast({"type": "nav", "nav": self.session.nav.model_dump()})
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda fut: self._on_result(fut, endpoint_id)
        )

    def _on_result(self, future, endpoint_id: str) -> None:
        from action_msgs.msg import GoalStatus

        del endpoint_id
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self.session.nav = self.session.nav.model_copy(
                update={
                    "status": NavStatus.FAILED,
                    "message": str(exc),
                }
            )
            self._broadcast({"type": "nav", "nav": self.session.nav.model_dump()})
            self._goal_handle = None
            self._clear_path()
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            nav_status = NavStatus.SUCCEEDED
            message = "Arrived"
        elif status in (
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_CANCELING,
        ):
            nav_status = NavStatus.CANCELLED
            message = "Cancelled"
        else:
            nav_status = NavStatus.FAILED
            message = f"Nav2 status={status}"

        self.session.nav = self.session.nav.model_copy(
            update={
                "status": nav_status,
                "message": message,
            }
        )
        self._broadcast({"type": "nav", "nav": self.session.nav.model_dump()})
        self._goal_handle = None
        self._clear_path()


gateway = NavGateway()
