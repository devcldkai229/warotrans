#!/usr/bin/env python3
"""Simulated base + RPLIDAR A1M8 for the recovery rig. No hardware, no physics engine.

Integrates /cmd_vel_motor (the only topic the real ESP32 bridge drives from), publishes /odom, TF and a /scan
ray-cast against the real map plus virtual obstacles. The laser sits where geometry.yaml says, with the A1M8 data
sheet's 0.15 m minimum range. It also knows the truth the robot does not: the real gap between the chassis and the
nearest obstacle, and it refuses to move through anything (a collision is counted, and fails the scenario).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

HALF_LENGTH, HALF_WIDTH = 0.16, 0.13  # chassis, geometry.yaml (no padding: this is the physical body)


def _body_offset() -> float:
    """base_footprint -> base_link x from geometry.yaml: the turning centre sits that far behind the chassis centre
    (2026-11-11 lab). The simulated base turns about base_footprint, like the real one."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2_ws", "src",
                        "warotrans_description", "config", "geometry.yaml")
    try:
        m = re.search(r"base_footprint_to_base_link:\s*\n(?:\s*#.*\n)*\s*x:\s*([-\d.]+)",
                      open(path, encoding="utf-8").read())
        return float(m.group(1)) if m else 0.0
    except OSError:
        return 0.0


BODY_X = _body_offset()  # chassis centre in base_footprint
LASER_X, LASER_Y, LASER_Z, LASER_YAW = BODY_X + 0.01, 0.0017, 0.1745, math.pi / 2  # geometry.yaml laser in base_link
RANGE_MIN, RANGE_MAX = 0.15, 12.0  # A1M8-R6 data sheet
BEAMS, SCAN_HZ = 1000, 8.0  # 8 kHz samples / 8 Hz, 0.36 deg
FINE = 0.01  # m, ray-march and collision grid


def load_grid(map_yaml: str, obstacles: list) -> tuple[np.ndarray, float, float]:
    """Occupancy on a 1 cm grid: map.pgm occupied cells plus virtual rectangles [xmin, ymin, xmax, ymax]."""
    with open(map_yaml, encoding="utf-8") as f:
        text = f.read()
    res = float(re.search(r"resolution:\s*([\d.]+)", text).group(1))
    ox, oy = [float(v) for v in re.search(r"origin:\s*\[([^\]]+)\]", text).group(1).split(",")[:2]]
    image = map_yaml.rsplit("/", 1)[0] + "/" + re.search(r"image:\s*(\S+)", text).group(1)
    raw = open(image, "rb").read()
    head = re.match(rb"P5\s+(?:#.*\n)?(\d+)\s+(\d+)\s+(\d+)\s", raw)
    w, h = int(head[1]), int(head[2])
    px = np.frombuffer(raw, np.uint8, offset=head.end()).reshape(h, w)[::-1]  # row 0 = lowest y
    k = int(round(res / FINE))
    grid = np.kron(px < 50, np.ones((k, k), bool))
    for xmin, ymin, xmax, ymax in obstacles:
        i0, i1 = int((xmin - ox) / FINE), int(math.ceil((xmax - ox) / FINE))
        j0, j1 = int((ymin - oy) / FINE), int(math.ceil((ymax - oy) / FINE))
        grid[max(j0, 0):j1, max(i0, 0):i1] = True
    return grid, ox, oy


class Base(Node):
    def __init__(self, a: argparse.Namespace) -> None:
        super().__init__("rig_base")
        self.a = a
        self.x, self.y, self.yaw = a.x, a.y, a.yaw
        self.odom_x, self.odom_y, self.odom_yaw = a.x, a.y, a.yaw  # map->odom is identity
        self._base_obstacles = json.loads(a.obstacles)
        self._timed = json.loads(getattr(a, "timed_obstacles", "[]") or "[]")
        self._t0: float | None = None  # set when the goal is accepted: Nav2 needs a varying time to come up first
        self._active_timed: list = []
        self.grid, self.ox, self.oy = load_grid(a.map, self._base_obstacles)
        self.cmd = (0.0, 0.0)
        self.cmd_at = -1e9
        self.v, self.w = 0.0, 0.0
        self.v_cmd, self.w_cmd = 0.0, 0.0
        self.collisions = 0
        self.travel, self.turned = 0.0, 0.0
        self.rng = np.random.default_rng(7)
        self.truth = open(a.truth, "a", encoding="utf-8", buffering=1)
        # First-order lag toward cmd (lab-like inertia); tau=0 → instantaneous.
        self.inertia_tau = float(getattr(a, "inertia_tau", 0.0) or 0.0)
        angles = -math.pi + np.arange(BEAMS) * (2 * math.pi / BEAMS)
        self.angles = angles
        self.steps = np.arange(RANGE_MIN, 8.0, FINE)
        self.create_subscription(Twist, a.cmd_topic, self._on_cmd, 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        self.static.sendTransform([
            self._tf("map", "odom", 0.0, 0.0, 0.0, 0.0),
            self._tf("base_footprint", "laser", LASER_X, LASER_Y, LASER_Z, LASER_YAW),
        ])
        self.dt = 0.02
        self.create_timer(self.dt, self._step)
        self.create_timer(0.05, self._publish_state)
        self.create_timer(1.0 / SCAN_HZ, self._publish_scan)
        self.create_timer(0.1, self._log_truth)
        self.create_timer(0.2, self._refresh_timed_obstacles)

    def _refresh_timed_obstacles(self) -> None:
        """Apply appear_at / disappear_at schedule: [xmin,ymin,xmax,ymax,appear,disappear]."""
        if not self._timed:
            return
        if self._t0 is None:
            if not os.path.exists(self.a.clock_file):
                return
            self._t0 = time.monotonic()
        age = time.monotonic() - self._t0
        active = []
        for item in self._timed:
            if len(item) < 6:
                continue
            xmin, ymin, xmax, ymax, appear, disappear = item[:6]
            if age >= float(appear) and (float(disappear) < 0 or age < float(disappear)):
                active.append([xmin, ymin, xmax, ymax])
        if active != self._active_timed:
            self._active_timed = active
            self.grid, self.ox, self.oy = load_grid(self.a.map, self._base_obstacles + active)
            self.get_logger().info(f"timed obstacles active={len(active)} age={age:.1f}s")

    def _tf(self, parent: str, child: str, x: float, y: float, z: float, yaw: float) -> TransformStamped:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = x, y, z
        t.transform.rotation.z, t.transform.rotation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        return t

    def _on_cmd(self, m: Twist) -> None:
        self.cmd = (m.linear.x, m.angular.z)
        self.cmd_at = time.monotonic()

    def _occupied(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        i = ((xs - self.ox) / FINE).astype(int)
        j = ((ys - self.oy) / FINE).astype(int)
        h, w = self.grid.shape
        inside = (i >= 0) & (j >= 0) & (i < w) & (j < h)
        out = np.zeros(xs.shape, bool)
        out[inside] = self.grid[j[inside], i[inside]]
        return out

    def _body_hits(self, x: float, y: float, yaw: float) -> bool:
        lx = np.arange(-HALF_LENGTH, HALF_LENGTH + FINE, FINE) + BODY_X
        ly = np.arange(-HALF_WIDTH, HALF_WIDTH + FINE, FINE)
        px = np.concatenate([lx, lx, np.full_like(ly, BODY_X - HALF_LENGTH), np.full_like(ly, BODY_X + HALF_LENGTH)])
        py = np.concatenate([np.full_like(lx, -HALF_WIDTH), np.full_like(lx, HALF_WIDTH), ly, ly])
        c, s = math.cos(yaw), math.sin(yaw)
        return bool(self._occupied(x + c * px - s * py, y + s * px + c * py).any())

    def gap(self) -> float:
        """True distance from the chassis rectangle to the nearest occupied 1 cm cell (within 0.6 m)."""
        n = int(0.8 / FINE)
        ci, cj = int((self.x - self.ox) / FINE), int((self.y - self.oy) / FINE)
        j0, i0 = max(cj - n, 0), max(ci - n, 0)
        sub = self.grid[j0:cj + n, i0:ci + n]
        jj, ii = np.nonzero(sub)
        if ii.size == 0:
            return 0.8
        wx = self.ox + (ii + i0 + 0.5) * FINE - self.x
        wy = self.oy + (jj + j0 + 0.5) * FINE - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        bx, by = c * wx + s * wy, -s * wx + c * wy
        dx = np.maximum(np.abs(bx - BODY_X) - HALF_LENGTH, 0.0)
        dy = np.maximum(np.abs(by) - HALF_WIDTH, 0.0)
        return float(np.hypot(dx, dy).min())

    def _step(self) -> None:
        # Same 300 ms watchdog as the ESP32 firmware.
        v_cmd, w_cmd = self.cmd if time.monotonic() - self.cmd_at < 0.3 else (0.0, 0.0)
        # Motor deadband is unmeasured on the real robot: configurable, reported with every result.
        if abs(v_cmd) < self.a.deadband_v:
            v_cmd = 0.0
        if abs(w_cmd) < self.a.deadband_w:
            w_cmd = 0.0
        self.v_cmd, self.w_cmd = v_cmd, w_cmd
        # First-order lag: dv/dt = (cmd - v) / tau
        if self.inertia_tau > 1e-6:
            alpha = self.dt / self.inertia_tau
            self.v += (v_cmd - self.v) * alpha
            self.w += (w_cmd - self.w) * alpha
            if abs(self.v) < 1e-4 and v_cmd == 0.0:
                self.v = 0.0
            if abs(self.w) < 1e-4 and w_cmd == 0.0:
                self.w = 0.0
        else:
            self.v, self.w = v_cmd, w_cmd
        v, w = self.v, self.w
        if v == 0.0 and w == 0.0:
            return
        yaw = self.yaw + w * self.dt
        x = self.x + v * math.cos(yaw) * self.dt
        y = self.y + v * math.sin(yaw) * self.dt
        if self._body_hits(x, y, yaw):
            self.collisions += 1
            self.v, self.w = 0.0, 0.0
            return
        self.travel += abs(v) * self.dt
        self.turned += abs(w) * self.dt
        self.x, self.y, self.yaw = x, y, yaw

    def _publish_state(self) -> None:
        now = self.get_clock().now().to_msg()
        t = self._tf("odom", "base_footprint", self.x, self.y, 0.0, self.yaw)
        t.header.stamp = now
        self.tf.sendTransform(t)
        o = Odometry()
        o.header.stamp = now
        o.header.frame_id, o.child_frame_id = "odom", "base_footprint"
        o.pose.pose.position.x, o.pose.pose.position.y = self.x, self.y
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(self.yaw / 2), math.cos(self.yaw / 2)
        o.twist.twist.linear.x, o.twist.twist.angular.z = self.v, self.w
        self.odom_pub.publish(o)

    def _publish_scan(self) -> None:
        lx = self.x + math.cos(self.yaw) * LASER_X - math.sin(self.yaw) * LASER_Y
        ly = self.y + math.sin(self.yaw) * LASER_X + math.cos(self.yaw) * LASER_Y
        a = self.yaw + LASER_YAW + self.angles
        xs = lx + np.outer(np.cos(a), self.steps)
        ys = ly + np.outer(np.sin(a), self.steps)
        hits = self._occupied(xs, ys)
        first = hits.argmax(axis=1)
        ranges = self.steps[first].astype(float) + self.rng.normal(0.0, 0.005, BEAMS) * self.steps[first]
        ranges[~hits.any(axis=1)] = math.inf
        # A beam already inside the blind zone returns nothing: what the driver reports below range_min.
        blind = np.arange(0.01, RANGE_MIN, FINE)
        ranges[self._occupied(lx + np.outer(np.cos(a), blind), ly + np.outer(np.sin(a), blind)).any(axis=1)] = math.inf
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "laser"
        msg.angle_min = float(self.angles[0])
        msg.angle_max = float(self.angles[-1])
        msg.angle_increment = float(2 * math.pi / BEAMS)
        msg.scan_time = 1.0 / SCAN_HZ
        msg.range_min, msg.range_max = RANGE_MIN, RANGE_MAX
        msg.ranges = [float(r) for r in ranges]
        self.scan_pub.publish(msg)

    def _log_truth(self) -> None:
        self.truth.write(json.dumps({
            "wall": round(time.time(), 3), "x": round(self.x, 4), "y": round(self.y, 4), "yaw": round(self.yaw, 4),
            # chassis centre: what an endpoint and the lane metrics are about
            "cx": round(self.x + BODY_X * math.cos(self.yaw), 4), "cy": round(self.y + BODY_X * math.sin(self.yaw), 4),
            "v": round(self.v, 4), "w": round(self.w, 4), "cmd": [round(self.cmd[0], 4), round(self.cmd[1], 4)],
            "gap": round(self.gap(), 4), "collisions": self.collisions,
            "travel": round(self.travel, 4), "turned": round(self.turned, 4),
        }) + "\n")


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--map", required=True)
    p.add_argument("--truth", required=True)
    p.add_argument("--x", type=float, required=True)
    p.add_argument("--y", type=float, required=True)
    p.add_argument("--yaw", type=float, required=True)
    p.add_argument("--obstacles", default="[]")
    p.add_argument(
        "--timed-obstacles",
        default="[]",
        help='JSON list of [xmin,ymin,xmax,ymax,appear_s,disappear_s] after the goal; disappear<0 = stay',
    )
    p.add_argument("--clock-file", default="/nonexistent", help="timed obstacles count from when this file appears")
    p.add_argument("--inertia-tau", type=float, default=0.0, help="cmd→velocity lag seconds (0=instant)")
    p.add_argument("--cmd-topic", default="/cmd_vel_motor")
    p.add_argument("--deadband-v", type=float, default=0.0)
    p.add_argument("--deadband-w", type=float, default=0.0)
    a = p.parse_args()
    rclpy.init()
    node = Base(a)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
