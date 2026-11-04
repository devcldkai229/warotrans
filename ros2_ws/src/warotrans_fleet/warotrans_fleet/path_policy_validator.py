"""Stage-2 path / goal policy validator (defense-in-depth for ONE_WAY / NO_STOP / KEEP_OUT).

Loads lanes.json + no_stop.json (+ optional keepout.json rings). Rejects illegal /plan and
NO_STOP goals; does not invent wait poses from JUNCTION.
"""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path as FsPath
from typing import Any, Optional

import rclpy
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from std_msgs.msg import Bool, String


def _adapter_dirs() -> list[FsPath]:
    dirs = [
        FsPath(__file__).resolve().parents[2] / "warotrans_simulation" / "scripts",
        FsPath(__file__).resolve().parents[3] / "warotrans_simulation" / "scripts",
    ]
    try:
        from ament_index_python.packages import get_package_prefix

        prefix = FsPath(get_package_prefix("warotrans_simulation"))
        dirs.append(prefix / "lib" / "warotrans_simulation")
        dirs.append(prefix / "share" / "warotrans_simulation" / "scripts")
    except Exception:  # noqa: BLE001
        pass
    return dirs


def _load_adapter():
    for d in _adapter_dirs():
        if (d / "semantic_map_adapter.py").is_file():
            if str(d) not in sys.path:
                sys.path.insert(0, str(d))
            import semantic_map_adapter as sma  # type: ignore

            return sma
    raise ImportError("semantic_map_adapter.py not found (build/install warotrans_simulation)")


def yaw_from_quat(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class PathPolicyValidator(Node):
    def __init__(self) -> None:
        super().__init__("path_policy_validator")
        self.declare_parameter("lanes_json", "")
        self.declare_parameter("no_stop_json", "")
        self.declare_parameter("keepout_json", "")
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("goal_topic", "/goal_pose")
        # Probe-only goals (must NOT be /goal_pose — bt_navigator also listens there).
        self.declare_parameter("probe_goal_topic", "/path_policy/check_goal")
        self.declare_parameter("probe_plan_topic", "/path_policy/check_plan")
        self.declare_parameter("margin", 0.05)
        self.declare_parameter("enable", True)
        self.declare_parameter("cancel_on_violation", True)

        self._sma = _load_adapter()
        self.lanes: list[dict[str, Any]] = []
        self.no_stop_rings: list[list[list[float]]] = []
        self.keepout_rings: list[list[list[float]]] = []
        self._reload()

        plan_topic = self.get_parameter("plan_topic").get_parameter_value().string_value
        goal_topic = self.get_parameter("goal_topic").get_parameter_value().string_value
        probe_goal = self.get_parameter("probe_goal_topic").get_parameter_value().string_value
        probe_plan = self.get_parameter("probe_plan_topic").get_parameter_value().string_value
        self.create_subscription(NavPath, plan_topic, self._on_plan, 10)
        self.create_subscription(PoseStamped, goal_topic, self._on_goal, 10)
        if probe_goal and probe_goal != goal_topic:
            self.create_subscription(PoseStamped, probe_goal, self._on_goal, 10)
        if probe_plan and probe_plan != plan_topic:
            self.create_subscription(NavPath, probe_plan, self._on_plan, 10)
        self._ok_pub = self.create_publisher(Bool, "/path_policy/ok", 10)
        self._reason_pub = self.create_publisher(String, "/path_policy/reason", 10)
        self._cancel_cli = self.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal"
        )

        self.get_logger().info(
            f"path_policy_validator ready lanes={len(self.lanes)} "
            f"no_stop={len(self.no_stop_rings)} keepout={len(self.keepout_rings)}"
        )

    def _reload(self) -> None:
        lanes_path = self.get_parameter("lanes_json").get_parameter_value().string_value
        no_stop_path = self.get_parameter("no_stop_json").get_parameter_value().string_value
        keepout_path = self.get_parameter("keepout_json").get_parameter_value().string_value
        if lanes_path and FsPath(lanes_path).is_file():
            data = json.loads(FsPath(lanes_path).read_text(encoding="utf-8"))
            self.lanes = list(data.get("lanes") or [])
        if no_stop_path and FsPath(no_stop_path).is_file():
            data = json.loads(FsPath(no_stop_path).read_text(encoding="utf-8"))
            self.no_stop_rings = [z["rings"] for z in data.get("zones") or [] if z.get("rings")]
        if keepout_path and FsPath(keepout_path).is_file():
            data = json.loads(FsPath(keepout_path).read_text(encoding="utf-8"))
            self.keepout_rings = [z["rings"] for z in data.get("zones") or [] if z.get("rings")]

    def _publish(self, ok: bool, reason: str) -> None:
        self._ok_pub.publish(Bool(data=ok))
        self._reason_pub.publish(String(data=reason))
        if ok:
            self.get_logger().debug(f"path_policy ok: {reason}")
        else:
            self.get_logger().error(f"path_policy REJECT: {reason}")

    def _cancel_nav(self) -> None:
        if not self.get_parameter("cancel_on_violation").get_parameter_value().bool_value:
            return
        if not self._cancel_cli.service_is_ready():
            self.get_logger().warn("navigate_to_pose cancel service not ready")
            return
        req = CancelGoal.Request()
        fut = self._cancel_cli.call_async(req)
        fut.add_done_callback(lambda _: None)

    def _on_goal(self, msg: PoseStamped) -> None:
        if not self.get_parameter("enable").get_parameter_value().bool_value:
            return
        x = float(msg.pose.position.x)
        y = float(msg.pose.position.y)
        yaw = yaw_from_quat(msg.pose.orientation)
        if self._sma.footprint_hits_polygons(x, y, yaw, self.no_stop_rings):
            self._publish(False, "NO_STOP_GOAL")
            self._cancel_nav()
            return
        if self._sma.footprint_hits_polygons(x, y, yaw, self.keepout_rings):
            self._publish(False, "KEEP_OUT_GOAL")
            self._cancel_nav()
            return
        self._publish(True, "goal_ok")

    def _on_plan(self, msg: NavPath) -> None:
        if not self.get_parameter("enable").get_parameter_value().bool_value:
            return
        margin = float(self.get_parameter("margin").value)
        poses = [
            (
                float(p.pose.position.x),
                float(p.pose.position.y),
                yaw_from_quat(p.pose.orientation),
            )
            for p in msg.poses
        ]
        ok, reason = self._sma.validate_path_poses(
            poses,
            lanes=self.lanes,
            keepout_rings=self.keepout_rings,
            no_stop_rings=self.no_stop_rings,
            check_no_stop_goal=True,
            margin=margin,
        )
        self._publish(ok, reason)
        if not ok:
            self._cancel_nav()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PathPolicyValidator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
