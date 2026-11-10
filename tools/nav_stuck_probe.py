#!/usr/bin/env python3
"""Read-only Nav2 stuck/tracking recorder: says WHICH layer swallowed a velocity command.

Publishes nothing, calls no service, sends no goal. Safe with motors locked or running.

    # on the robot / rig, with the stack up:
    python3 nav_stuck_probe.py record out.jsonl [--lidar-min 0.15]
    # anywhere (no ROS needed):
    python3 nav_stuck_probe.py report out.jsonl

Every sample (10 Hz) holds the last value of each layer of the cmd_vel pipeline, the collision monitor state,
authority_ok, pose sources, path-tracking error, and how many scan points lie inside the padded footprint.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import Counter
from typing import Iterable, Optional, Sequence

# Pipeline order: first layer whose output is zero while the previous one is not, swallowed the command.
LAYERS = ("cmd_vel_nav", "cmd_vel_smoothed", "cmd_vel_safe", "cmd_vel_motor")
NAMES = {
    "cmd_vel_smoothed": "velocity_smoother",
    "cmd_vel_safe": "collision_monitor",
    "cmd_vel_motor": "authority_gate",
}
ACTIONS = (
    "navigate_to_pose", "follow_path", "compute_path_to_pose", "backup", "spin", "wait",
    "guarded_backup", "guarded_spin", "smooth_path",
)
# Footprint from nav2_params.yaml + footprint_padding (geometry.yaml chassis 0.32 x 0.26).
HALF_LENGTH, HALF_WIDTH, PADDING = 0.16, 0.13, 0.02
MOVING = 0.005


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def is_moving(cmd: Optional[Sequence[float]]) -> bool:
    return cmd is not None and (abs(cmd[0]) > MOVING or abs(cmd[1]) > MOVING)


def swallowing_layer(cmds: dict, known: Iterable[str] = LAYERS) -> Optional[str]:
    """Layer that turned a non-zero command into zero or into silence (None: nothing commanded, or it got through).

    The collision monitor stops publishing after stop_pub_timeout, so a silent layer can be the one blocking. ``known``
    names the layers that exist in this stack: without a gate cmd_vel_safe never publishes at all.
    """
    known = set(known)
    previous = None
    for name in LAYERS:
        if name not in known:
            continue
        cmd = cmds.get(name)
        if is_moving(previous) and not is_moving(cmd):
            return NAMES.get(name, name)
        previous = cmd
    return None


def scale_of(upper: Optional[Sequence[float]], lower: Optional[Sequence[float]]) -> Optional[float]:
    """How much of the upper command survived (largest component). None if the upper one is ~zero."""
    if upper is None or lower is None or not is_moving(upper):
        return None
    ratios = [abs(b) / abs(a) for a, b in zip(upper, lower) if abs(a) > MOVING]
    return round(max(ratios), 3) if ratios else None


def points_in_padded_footprint(points: Iterable[Sequence[float]]) -> int:
    hl, hw = HALF_LENGTH + PADDING, HALF_WIDTH + PADDING
    return sum(1 for x, y in points if abs(x) <= hl and abs(y) <= hw)


def lateral_error(path: Sequence[Sequence[float]], x: float, y: float) -> Optional[float]:
    """Distance from the pose to the nearest path segment (m)."""
    if len(path) < 2:
        return None
    best = math.inf
    for (ax, ay), (bx, by) in zip(path, path[1:]):
        dx, dy = bx - ax, by - ay
        seg = dx * dx + dy * dy
        t = 0.0 if seg < 1e-12 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def summarize(lines: Iterable[dict]) -> dict:
    """Pure analysis of a recording: blocked time per layer, recovery activity, tracking error, pose jumps."""
    lines = list(lines)
    samples = [ln for ln in lines if ln.get("t") == "s"]
    events = [ln for ln in lines if ln.get("t") == "action"]
    guard = [ln for ln in lines if ln.get("t") == "guard"]
    period = 0.1
    blocked = Counter()
    errors: list[float] = []
    jumps: list[float] = []
    near = []
    prev = None
    for s in samples:
        layer = s.get("layer")
        if layer:
            blocked[layer] += 1
        if s.get("lat_err") is not None:
            errors.append(s["lat_err"])
        if s.get("n_in_fp") is not None:
            near.append(s["n_in_fp"])
        pose = s.get("map_pose")
        if pose and prev:
            step = math.hypot(pose[0] - prev[0], pose[1] - prev[1])
            # 0.10 m/s * 0.1 s = 1 cm per sample: anything above 5 cm is a localization correction, not motion.
            if step > 0.05:
                jumps.append(round(step, 3))
        prev = pose or prev
    errors.sort()
    actions = Counter(f"{e['action']}:{e['status']}" for e in events)
    return {
        "samples": len(samples),
        "duration_s": round(len(samples) * period, 1),
        "blocked_s": {k: round(v * period, 1) for k, v in blocked.most_common()},
        "max_points_in_padded_footprint": max(near) if near else None,
        "lateral_error_m": None if not errors else {
            "p50": round(errors[len(errors) // 2], 3),
            "p95": round(errors[int(len(errors) * 0.95)], 3),
            "max": round(errors[-1], 3),
        },
        "pose_jumps_over_5cm": jumps,
        "actions": dict(sorted(actions.items())),
        "guard": dict(sorted(Counter(f"{g['kind']}:{g['reason'] or 'ALLOWED'}" for g in guard).items())),
    }


def report(path: str) -> int:
    with open(path, encoding="utf-8") as f:
        lines = [json.loads(x) for x in f if x.strip()]
    print(json.dumps(summarize(lines), indent=1, ensure_ascii=False))
    return 0


def record(path: str, lidar_min: float) -> int:  # pragma: no cover - needs ROS
    import rclpy
    from action_msgs.msg import GoalStatusArray
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from nav_msgs.msg import Odometry, Path
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener

    try:
        from nav2_msgs.msg import CollisionMonitorState
    except ImportError:
        CollisionMonitorState = None

    STATUS = {1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}

    def yaw_of(q) -> float:
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("nav_stuck_probe")
            self.out = open(path, "a", encoding="utf-8", buffering=1)
            self.tf = Buffer()
            TransformListener(self.tf, self)
            self.cmd = {}
            self.cmd_at = {}
            self.state = {}
            self.plan: list = []
            self.smoothed: list = []
            self.scan_pts = None
            self.goal_status = {}
            for name in LAYERS + ("cmd_vel",):
                self.create_subscription(Twist, "/" + name, self._cmd(name), 10)
            self.create_subscription(Bool, "/traffic/authority_ok", lambda m: self.state.update(authority_ok=m.data), 10)
            self.create_subscription(Odometry, "/odom", lambda m: self._odom("odom", m), 10)
            self.create_subscription(Odometry, "/odometry/filtered", lambda m: self._odom("ekf", m), 10)
            self.create_subscription(
                PoseWithCovarianceStamped, "/amcl_pose",
                lambda m: self.state.update(amcl=[m.pose.pose.position.x, m.pose.pose.position.y, yaw_of(m.pose.pose.orientation)]),
                10,
            )
            self.create_subscription(
                Imu, "/imu/data",
                lambda m: self.state.update(imu_yaw=yaw_of(m.orientation), imu_w=m.angular_velocity.z),
                qos_profile_sensor_data,
            )
            self.create_subscription(Path, "/plan", lambda m: self._path(m, "plan"), 10)
            self.create_subscription(Path, "/plan_smoothed", lambda m: self._path(m, "smoothed"), 10)
            self.create_subscription(LaserScan, "/scan", self._scan, qos_profile_sensor_data)
            if CollisionMonitorState is not None:
                self.create_subscription(
                    CollisionMonitorState, "/collision_monitor_state",
                    lambda m: self.state.update(cm=[int(m.action_type), m.polygon_name]), 10,
                )
            self.create_subscription(String, "/recovery_guard/decision", lambda m: self._write({"t": "guard", **json.loads(m.data)}), 10)
            for action in ACTIONS:
                self.create_subscription(GoalStatusArray, f"/{action}/_action/status", self._status(action), 10)
            self.create_timer(0.1, self._tick)

        def _write(self, obj: dict) -> None:
            obj["wall"] = round(time.time(), 3)
            self.out.write(json.dumps(obj) + "\n")

        def _cmd(self, name: str):
            def cb(m) -> None:
                self.cmd[name] = (round(m.linear.x, 4), round(m.angular.z, 4))
                self.cmd_at[name] = time.monotonic()
            return cb

        def _odom(self, key: str, m) -> None:
            p = m.pose.pose
            self.state[key] = [round(p.position.x, 4), round(p.position.y, 4), round(yaw_of(p.orientation), 4)]

        def _path(self, m, key: str) -> None:
            pts = [(q.pose.position.x, q.pose.position.y) for q in m.poses]
            setattr(self, key, pts)
            self._write({"t": "path", "which": key, "n": len(pts), "start": pts[0] if pts else None, "end": pts[-1] if pts else None})

        def _scan(self, m) -> None:
            try:
                tr = self.tf.lookup_transform("base_footprint", m.header.frame_id, rclpy.time.Time())
            except Exception:
                return
            yaw = yaw_of(tr.transform.rotation)
            ox, oy = tr.transform.translation.x, tr.transform.translation.y
            pts = []
            for i, r in enumerate(m.ranges):
                if math.isfinite(r) and m.range_min <= r <= m.range_max:
                    a = m.angle_min + i * m.angle_increment + yaw
                    pts.append((ox + r * math.cos(a), oy + r * math.sin(a)))
            self.scan_pts = pts
            self.scan_inf = sum(1 for r in m.ranges if not math.isfinite(r))
            self.scan_rmin = m.range_min

        def _status(self, action: str):
            def cb(m) -> None:
                for s in m.status_list:
                    key = bytes(s.goal_info.goal_id.uuid).hex()
                    if self.goal_status.get(key) != s.status:
                        self.goal_status[key] = s.status
                        self._write({"t": "action", "action": action, "status": STATUS.get(s.status, str(s.status)), "goal": key[:8]})
            return cb

        def _tick(self) -> None:
            now = time.monotonic()
            # A layer that stopped publishing is reported as silence, not as its last (maybe non-zero) value.
            cmd = {n: self.cmd[n] for n in self.cmd if now - self.cmd_at[n] < 0.6}
            sample = {"t": "s", "cmd": cmd, "layer": swallowing_layer(cmd, self.cmd), "state": self.state}
            try:
                tr = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
                x, y = tr.transform.translation.x, tr.transform.translation.y
                sample["map_pose"] = [round(x, 4), round(y, 4), round(yaw_of(tr.transform.rotation), 4)]
                active = self.smoothed or self.plan
                err = lateral_error(active, x, y)
                sample["lat_err"] = None if err is None else round(err, 4)
            except Exception:
                sample["map_pose"] = None
            if self.scan_pts is not None:
                sample["n_in_fp"] = points_in_padded_footprint(self.scan_pts)
                ranges = sorted(math.hypot(px, py) for px, py in self.scan_pts)
                sample["nearest_from_base"] = round(ranges[0], 3) if ranges else None
                sample["scan_inf"] = self.scan_inf
            self._write(sample)

    rclpy.init()
    node = Probe()
    node.get_logger().info(f"recording to {path} (read-only: this node has no publishers of its own)")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record")
    r.add_argument("path")
    r.add_argument("--lidar-min", type=float, default=0.15)
    rep = sub.add_parser("report")
    rep.add_argument("path")
    args = ap.parse_args(argv)
    return record(args.path, args.lidar_min) if args.cmd == "record" else report(args.path)


if __name__ == "__main__":
    sys.exit(main())
