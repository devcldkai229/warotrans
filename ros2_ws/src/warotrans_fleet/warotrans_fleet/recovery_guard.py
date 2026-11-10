"""Guard in front of Nav2's BackUp and Spin: a recovery starts only with clearance and a valid traffic grant.

The behavior tree calls ``guarded_backup`` / ``guarded_spin`` instead of ``backup`` / ``spin``. This node answers those
actions: it refuses (goal ABORTED, the robot never gets a command) or forwards the goal unchanged to Nav2.

Why Nav2's own check is not enough: BackUp and Spin look only at the local costmap, whose obstacle layer ignores scan
returns closer than ``obstacle_min_range``, and they know nothing about traffic. The guard looks at the live scan with
the same padded footprint and point threshold as the collision monitor, and asks the gateway about the grant.

This only decides whether to START. While the manoeuvre runs every command still passes the collision monitor and the
authority gate. Not seen by anything here: objects below the LiDAR plane (~17.5 cm) and thin objects inside its 0.15 m
blind zone.

The logic is pure Python; the node exists only when rclpy can be imported.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

Point = Sequence[float]


@dataclass(frozen=True)
class GuardConfig:
    half_length: float = 0.16  # geometry.yaml chassis
    half_width: float = 0.13
    padding: float = 0.02  # costmap footprint_padding: the collision monitor's polygon includes it
    min_points: int = 4  # collision monitor FootprintApproach.min_points
    scan_max_age_sec: float = 0.5
    max_backup_m: float = 0.12  # what the gateway authorizes: see robot_gateway recovery_backup_check_m
    backup_margin_m: float = 0.03
    spin_step_rad: float = math.radians(3.0)


def _inside(cfg: GuardConfig, x: float, y: float) -> bool:
    hl, hw = cfg.half_length + cfg.padding, cfg.half_width + cfg.padding
    return -hl <= x <= hl and abs(y) <= hw


def count_in_footprint(cfg: GuardConfig, points: Iterable[Point]) -> int:
    return sum(1 for p in points if _inside(cfg, p[0], p[1]))


def backup_blocking_points(cfg: GuardConfig, points: Iterable[Point], distance: float) -> int:
    """Points in the way of reversing ``distance`` metres: the rear half of the padded footprint plus what it sweeps.

    Points ahead of the centre are left out: reversing moves away from them. If they are inside the polygon the
    collision monitor still decides, command by command, so this can never move the robot past the monitor.
    """
    hl, hw = cfg.half_length + cfg.padding, cfg.half_width + cfg.padding
    rear_limit = -hl - max(0.0, distance) - cfg.backup_margin_m
    return sum(1 for p in points if rear_limit <= p[0] < 0.0 and abs(p[1]) <= hw)


def spin_blocking_points(cfg: GuardConfig, points: Iterable[Point], angle: float) -> int:
    """Points the padded footprint sweeps turning by ``angle`` (+ = left, - = right). Points are base frame."""
    hl, hw = cfg.half_length + cfg.padding, cfg.half_width + cfg.padding
    reach = math.hypot(hl, hw)
    steps = max(1, math.ceil(abs(angle) / cfg.spin_step_rad))
    rotations = [(math.cos(angle * i / steps), math.sin(angle * i / steps)) for i in range(0, steps + 1)]
    n = 0
    for p in points:
        x, y = p[0], p[1]
        if x * x + y * y > reach * reach:
            continue  # cannot be reached at any heading
        # Turning the body by +a moves a world point by -a in the body frame.
        if any(_inside(cfg, c * x + s * y, -s * x + c * y) for c, s in rotations):
            n += 1
    return n


def clearance_block_reason(
    cfg: GuardConfig,
    kind: str,
    amount: float,
    points: Optional[Sequence[Point]],
    scan_age_sec: Optional[float],
    pose_known: bool,
) -> Optional[str]:
    """None when the sensors say this manoeuvre has room. Missing or old data is a refusal, never a pass."""
    if not pose_known:
        return "POSE_UNKNOWN"
    if points is None or scan_age_sec is None or scan_age_sec != scan_age_sec:
        return "NO_SCAN"
    if scan_age_sec < 0.0 or scan_age_sec > cfg.scan_max_age_sec:
        return "SCAN_STALE"
    if not math.isfinite(amount):
        return "INVALID_REQUEST"
    if kind == "backup":
        if amount <= 0.0 or amount > cfg.max_backup_m:
            return "BACKUP_DISTANCE_NOT_AUTHORIZED"
        return "REAR_NOT_CLEAR" if backup_blocking_points(cfg, points, amount) >= cfg.min_points else None
    if kind == "spin":
        # A turn sweeps all four corners, so anything already inside the polygon blocks it.
        if count_in_footprint(cfg, points) >= cfg.min_points:
            return "OBSTACLE_IN_FOOTPRINT"
        return "SPIN_NOT_CLEAR" if spin_blocking_points(cfg, points, amount) >= cfg.min_points else None
    return "INVALID_REQUEST"


def decision_record(kind: str, amount: float, reason: Optional[str]) -> str:
    return json.dumps({"kind": kind, "amount": round(amount, 3), "allowed": reason is None, "reason": reason})


# ----------------------------------------------------------------------------- ROS node (optional)

try:  # pragma: no cover - only with ROS
    import rclpy
    from nav2_msgs.action import BackUp, Spin
    from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_ros import Buffer, TransformListener

    _HAS_RCLPY = True
except ImportError:  # pragma: no cover
    _HAS_RCLPY = False


if _HAS_RCLPY:  # pragma: no cover - needs rclpy; the decisions are tested through the functions above

    class RecoveryGuard(Node):
        def __init__(self) -> None:
            super().__init__("recovery_guard")
            self.declare_parameter("require_traffic_authorization", True)
            self.declare_parameter("traffic_timeout_sec", 0.5)
            self.declare_parameter("scan_topic", "/scan")
            self.declare_parameter("base_frame", "base_footprint")
            self.declare_parameter("global_frame", "map")
            self._require_traffic = bool(self.get_parameter("require_traffic_authorization").value)
            self._traffic_timeout = float(self.get_parameter("traffic_timeout_sec").value)
            self._base = str(self.get_parameter("base_frame").value)
            self._global = str(self.get_parameter("global_frame").value)
            self._cfg = GuardConfig()
            self._group = ReentrantCallbackGroup()
            self._tf = Buffer()
            self._listener = TransformListener(self._tf, self)
            self._scan: Optional[tuple[float, list[tuple[float, float]]]] = None
            self.create_subscription(
                LaserScan, str(self.get_parameter("scan_topic").value), self._on_scan, qos_profile_sensor_data,
                callback_group=self._group,
            )
            self._decisions = self.create_publisher(String, "recovery_guard/decision", 10)
            self._authorize = {
                "backup": self.create_client(Trigger, "traffic/authorize_backup", callback_group=self._group),
                "spin": self.create_client(Trigger, "traffic/authorize_spin", callback_group=self._group),
            }
            self._downstream = {
                "backup": ActionClient(self, BackUp, "backup", callback_group=self._group),
                "spin": ActionClient(self, Spin, "spin", callback_group=self._group),
            }
            for name, action, kind in (("guarded_backup", BackUp, "backup"), ("guarded_spin", Spin, "spin")):
                ActionServer(
                    self, action, name, execute_callback=self._executor_for(kind, action),
                    goal_callback=lambda _goal: GoalResponse.ACCEPT,
                    cancel_callback=lambda _handle: CancelResponse.ACCEPT,
                    callback_group=self._group,
                )
            self.get_logger().info(f"recovery guard up (traffic authorization required: {self._require_traffic})")

        def _on_scan(self, msg: "LaserScan") -> None:
            try:
                tr = self._tf.lookup_transform(self._base, msg.header.frame_id, rclpy.time.Time())
            except Exception:  # noqa: BLE001 - no transform: no usable scan, which refuses recovery
                self._scan = None
                return
            q = tr.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            ox, oy = tr.transform.translation.x, tr.transform.translation.y
            points = []
            for i, r in enumerate(msg.ranges):
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    a = msg.angle_min + i * msg.angle_increment + yaw
                    points.append((ox + r * math.cos(a), oy + r * math.sin(a)))
            self._scan = (self.get_clock().now().nanoseconds / 1e9, points)

        def _clearance_reason(self, kind: str, amount: float) -> Optional[str]:
            pose_known = self._tf.can_transform(self._global, self._base, rclpy.time.Time())
            scan = self._scan
            age = None if scan is None else self.get_clock().now().nanoseconds / 1e9 - scan[0]
            return clearance_block_reason(self._cfg, kind, amount, None if scan is None else scan[1], age, pose_known)

        async def _traffic_reason(self, kind: str) -> Optional[str]:
            if not self._require_traffic:
                return None
            client = self._authorize[kind]
            if not client.service_is_ready():
                return "TRAFFIC_AUTHORIZATION_UNAVAILABLE"
            future = client.call_async(Trigger.Request())
            done = self.create_rate(50, self.get_clock())
            deadline = self.get_clock().now().nanoseconds / 1e9 + self._traffic_timeout
            try:
                while not future.done():
                    if self.get_clock().now().nanoseconds / 1e9 > deadline:
                        future.cancel()
                        return "TRAFFIC_AUTHORIZATION_TIMEOUT"
                    done.sleep()
            finally:
                self.destroy_rate(done)
            response = future.result()
            if response is None:
                return "TRAFFIC_AUTHORIZATION_UNAVAILABLE"
            return None if response.success else (response.message or "TRAFFIC_REFUSED")

        def _publish(self, kind: str, amount: float, reason: Optional[str]) -> None:
            text = decision_record(kind, amount, reason)
            self._decisions.publish(String(data=text))
            if reason is None:
                self.get_logger().info(f"recovery allowed: {text}")
            else:
                self.get_logger().warning(f"recovery refused, robot not commanded: {text}")

        def _executor_for(self, kind: str, action):
            async def execute(goal_handle):
                amount = goal_handle.request.target.x if kind == "backup" else goal_handle.request.target_yaw
                reason = self._clearance_reason(kind, float(amount))
                if reason is None:
                    reason = await self._traffic_reason(kind)
                self._publish(kind, float(amount), reason)
                result = action.Result()
                if reason is not None:
                    result.error_code = action.Result.COLLISION_AHEAD if "CLEAR" in reason or "FOOTPRINT" in reason else action.Result.UNKNOWN
                    goal_handle.abort()
                    return result
                return await self._forward(kind, action, goal_handle)
            return execute

        async def _forward(self, kind: str, action, goal_handle):
            client = self._downstream[kind]
            result = action.Result()
            if not client.server_is_ready():
                result.error_code = action.Result.UNKNOWN
                goal_handle.abort()
                return result
            inner = await client.send_goal_async(goal_handle.request)
            if not inner.accepted:
                result.error_code = action.Result.UNKNOWN
                goal_handle.abort()
                return result
            done = inner.get_result_async()
            canceled = False
            rate = self.create_rate(20, self.get_clock())
            try:
                while not done.done():
                    if goal_handle.is_cancel_requested and not canceled:
                        canceled = True
                        inner.cancel_goal_async()
                    rate.sleep()
            finally:
                self.destroy_rate(rate)
            wrapped = done.result()
            status = wrapped.status
            if status == 4:  # SUCCEEDED
                goal_handle.succeed()
            elif status == 5 or goal_handle.is_cancel_requested:  # CANCELED
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return wrapped.result

    def main(args=None) -> None:
        rclpy.init(args=args)
        node = RecoveryGuard()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        finally:
            node.destroy_node()
            rclpy.try_shutdown()

else:  # pragma: no cover

    def main(args=None) -> None:
        raise RuntimeError("rclpy not available; recovery_guard node needs a ROS 2 environment")


if __name__ == "__main__":
    main()
