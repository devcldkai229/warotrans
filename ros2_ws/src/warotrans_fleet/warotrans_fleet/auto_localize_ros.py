"""ROS side of AUTO_LOCALIZE (auto_localize.py): the map, the latest scan, the localizer and the spin.

Runs in the auto-localize worker thread, never in the executor: it waits by polling what the executor callbacks store,
so the SingleThreadedExecutor of the gateway keeps spinning meanwhile.

Localizer (parameter ``localizer``):
  amcl          /initialpose, convergence from /amcl_pose covariance, global localization by the AMCL service
                ``reinitialize_global_localization`` plus a slow turn on the spot.
  slam_toolbox  /initialpose (slam_toolbox localization mode takes it), convergence from its /pose covariance (set
                ``amcl_pose_topic`` to it). It has no global localization: no spin, suggestions instead.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Optional

from warotrans_fleet.scan_match_localizer import GridMap, LikelihoodField, scan_points

LOCALIZER_AMCL = "amcl"
LOCALIZER_SLAM_TOOLBOX = "slam_toolbox"


class AutoLocalizeRos:
    def __init__(
        self,
        node: Any,
        *,
        tf_buffer: Any,
        map_frame: str,
        base_frame: str,
        readiness: Any,  # AmclReadiness or None (simulation: TF only)
        publish_cmd_vel: Optional[Callable[[float, float], None]],
        motion_held: Callable[[], bool],
        localizer: str = LOCALIZER_AMCL,
        map_topic: str = "/map",
        scan_topic: str = "/scan",
        global_service: str = "/reinitialize_global_localization",
        spin_speed_rad_s: float = 0.35,
        spin_clearance_m: float = 0.22,
    ) -> None:
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from sensor_msgs.msg import LaserScan

        self._node = node
        self._tf = tf_buffer
        self._map_frame = map_frame
        self._base_frame = base_frame
        self._readiness = readiness
        self._publish_cmd_vel = publish_cmd_vel
        self._motion_held = motion_held
        self.localizer = localizer.strip().lower() or LOCALIZER_AMCL
        self._global_service = global_service
        self._spin_speed = abs(spin_speed_rad_s)
        self._spin_clearance = spin_clearance_m
        self._lock = threading.Lock()
        self._grid_msg = None
        self._field: Optional[LikelihoodField] = None
        self._field_key: Optional[tuple] = None
        self._scan = None
        self._scan_at = 0.0
        self._mount: dict[str, tuple[float, float, float]] = {}
        self._service_client = None

        # map_server publishes /map once, reliable + transient_local.
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, map_topic, self._on_map, map_qos)
        node.create_subscription(LaserScan, scan_topic, self._on_scan, qos_profile_sensor_data)

    # ------------------------------------------------------------------ executor callbacks

    def _on_map(self, msg: Any) -> None:
        with self._lock:
            self._grid_msg = msg

    def _on_scan(self, msg: Any) -> None:
        with self._lock:
            self._scan = msg
            self._scan_at = time.monotonic()

    # ------------------------------------------------------------------ handler ports

    @property
    def can_spin(self) -> bool:
        return self.localizer == LOCALIZER_AMCL and self._publish_cmd_vel is not None

    def get_field(self) -> Optional[LikelihoodField]:
        with self._lock:
            msg = self._grid_msg
        if msg is None:
            return None
        info = msg.info
        key = (info.width, info.height, info.resolution, info.origin.position.x, info.origin.position.y, len(msg.data))
        if key != self._field_key:
            grid = GridMap(
                int(info.width), int(info.height), float(info.resolution),
                float(info.origin.position.x), float(info.origin.position.y), list(msg.data),
            )
            self._field = LikelihoodField(grid)
            self._field_key = key
        return self._field

    def get_points(self, timeout_sec: float) -> Optional[list[tuple[float, float]]]:
        """A scan taken after this call (the robot may have just stopped), in the base frame."""
        asked = time.monotonic()
        deadline = asked + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                scan, at = self._scan, self._scan_at
            if scan is not None and at >= asked:
                mount = self._laser_mount(scan.header.frame_id)
                if mount is None:
                    return None
                return scan_points(
                    scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max, mount,
                    max_beams=120,
                )
            time.sleep(0.05)
        return None

    def wait_converged(self, timeout_sec: float) -> Optional[str]:
        """None once the localizer took the pose; otherwise why not (after the timeout)."""
        deadline = time.monotonic() + timeout_sec
        if self._readiness is None:
            time.sleep(1.0)  # simulation: TF only, give the localizer one cycle
            return None if self.current_pose() is not None else "NO_TRANSFORM"
        reason: Optional[str] = "NO_AMCL_POSE"
        while time.monotonic() < deadline:
            reason = self._readiness.blocking_reason()
            if reason is None:
                return None
            time.sleep(0.2)
        return reason

    def current_pose(self) -> Optional[tuple[float, float, float]]:
        from rclpy.time import Time
        from tf2_ros import TransformException

        from warotrans_fleet.transforms import quaternion_to_yaw

        try:
            tf = self._tf.lookup_transform(self._map_frame, self._base_frame, Time()).transform
        except TransformException:
            return None
        q = tf.rotation
        return float(tf.translation.x), float(tf.translation.y), quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def global_localize(self) -> Optional[str]:
        """AMCL only: spread the particles over the map, turn once on the spot, wait for convergence."""
        if not self.can_spin:
            return "GLOBAL_LOCALIZATION_UNSUPPORTED"
        if self._motion_held():
            return "MOTION_HELD"
        nearest = self._nearest_obstacle()
        if nearest is None:
            return "NO_SCAN"
        if nearest < self._spin_clearance:
            return f"OBSTACLE_TOO_CLOSE {nearest:.2f}m"
        reason = self._call_global_service()
        if reason is not None:
            return reason
        if self._readiness is not None:
            self._readiness.on_initial_pose()  # the system set the estimate; convergence is still required
        reason = self._spin_once()
        if reason is not None:
            return reason
        return self.wait_converged(10.0)

    # ------------------------------------------------------------------ helpers

    def _laser_mount(self, frame: str) -> Optional[tuple[float, float, float]]:
        mount = self._mount.get(frame)
        if mount is not None:
            return mount
        from rclpy.time import Time
        from tf2_ros import TransformException

        from warotrans_fleet.transforms import quaternion_to_yaw

        try:
            tf = self._tf.lookup_transform(self._base_frame, frame, Time()).transform
        except TransformException:
            return None
        q = tf.rotation
        mount = (float(tf.translation.x), float(tf.translation.y), quaternion_to_yaw(q.x, q.y, q.z, q.w))
        self._mount[frame] = mount
        return mount

    def _nearest_obstacle(self) -> Optional[float]:
        points = self.get_points(1.0)
        if not points:
            return None
        return min(math.hypot(x, y) for x, y in points)

    def _call_global_service(self) -> Optional[str]:
        from std_srvs.srv import Empty

        if self._service_client is None:
            self._service_client = self._node.create_client(Empty, self._global_service)
        client = self._service_client
        if not client.wait_for_service(timeout_sec=3.0):
            return "GLOBAL_LOCALIZATION_SERVICE_UNAVAILABLE"
        future = client.call_async(Empty.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        return None if future.done() else "GLOBAL_LOCALIZATION_SERVICE_TIMEOUT"

    def _spin_once(self) -> Optional[str]:
        assert self._publish_cmd_vel is not None
        duration = 1.15 * 2.0 * math.pi / max(self._spin_speed, 0.05)
        end = time.monotonic() + duration
        try:
            while time.monotonic() < end:
                if self._motion_held():
                    return "MOTION_HELD"
                nearest = self._nearest_obstacle()
                if nearest is not None and nearest < self._spin_clearance:
                    return f"OBSTACLE_TOO_CLOSE {nearest:.2f}m"
                self._publish_cmd_vel(0.0, self._spin_speed)
                time.sleep(0.1)
        finally:
            for _ in range(3):
                self._publish_cmd_vel(0.0, 0.0)
                time.sleep(0.05)
        return None
