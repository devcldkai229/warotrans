"""The OTHER robots as obstacle points, from the backend's `fleet/poses` broadcast.

A 2D lidar scanning ~18 cm above the floor sees little of a robot whose body is 10 cm high (RBT-002) or only its
lidar puck and stand-offs: two real robots came within 5 cm of each other (2026-11-22, 2026-11-23) although each had
a collision monitor. The backend knows where every robot is; each gateway turns the others into points filling their
footprint (plus a margin), published as a PointCloud2 in the map frame. warotrans_fleet_layer marks those points in
the global and local costmaps (and forgets them on the next update, so a robot that moved leaves no trace) and the
collision monitor takes them as a pointcloud source.

Pure geometry here; ROS wiring is in robot_gateway_node (FleetObstaclesPublisher).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Optional

#: A pose older than this (backend age + transport) is not drawn: the robot may be elsewhere by now.
MAX_POSE_AGE_SEC = 2.0
#: Added around each body: the other robot's localization error and the time since its pose.
DEFAULT_MARGIN_M = 0.02
#: Spacing of the points over a body; the costmap cell is 0.05 m.
DEFAULT_STEP_M = 0.04
#: Height of the points: inside the collision monitor's min/max height and the costmap's obstacle heights.
POINT_Z_M = 0.10


@dataclass(frozen=True)
class OtherRobot:
    code: str
    x: float
    y: float
    yaw: float
    length: float
    width: float
    localized: bool
    pose_age_sec: float


def parse_fleet_poses(body: dict[str, Any], self_code: str, received_age_sec: float = 0.0) -> list[OtherRobot]:
    """The other robots in a `fleet/poses` message whose pose is fresh enough to draw."""
    out: list[OtherRobot] = []
    for r in body.get("robots") or []:
        try:
            code = str(r.get("robotCode") or "")
            if not code or code == self_code:
                continue
            age = float(r.get("poseAgeMs") or 0) / 1000.0 + max(0.0, received_age_sec)
            if age > MAX_POSE_AGE_SEC:
                continue
            out.append(OtherRobot(
                code=code,
                x=float(r["x"]), y=float(r["y"]), yaw=float(r.get("yaw") or 0.0),
                length=max(0.05, float(r.get("lengthM") or 0.32)), width=max(0.05, float(r.get("widthM") or 0.26)),
                localized=bool(r.get("localized", True)), pose_age_sec=age,
            ))
        except (KeyError, TypeError, ValueError):
            continue
    return out


def footprint_points(robot: OtherRobot, margin: float = DEFAULT_MARGIN_M, step: float = DEFAULT_STEP_M) -> list[tuple[float, float]]:
    """Points filling the robot's body rectangle grown by `margin` (more when it is not localized: any heading)."""
    if robot.localized:
        hl, hw = robot.length / 2.0 + margin, robot.width / 2.0 + margin
        c, s = math.cos(robot.yaw), math.sin(robot.yaw)
        nx, ny = max(1, math.ceil(2 * hl / step)), max(1, math.ceil(2 * hw / step))
        pts = []
        for i in range(nx + 1):
            for j in range(ny + 1):
                lx, ly = -hl + 2 * hl * i / nx, -hw + 2 * hw * j / ny
                pts.append((robot.x + c * lx - s * ly, robot.y + s * lx + c * ly))
        return pts
    # Lost localization: its last pose, any heading, with a larger margin.
    radius = math.hypot(robot.length / 2.0, robot.width / 2.0) + 2 * margin
    n = max(1, math.ceil(2 * radius / step))
    pts = []
    for i in range(n + 1):
        for j in range(n + 1):
            lx, ly = -radius + 2 * radius * i / n, -radius + 2 * radius * j / n
            if lx * lx + ly * ly <= radius * radius:
                pts.append((robot.x + lx, robot.y + ly))
    return pts


def fleet_points(robots: Iterable[OtherRobot], margin: float = DEFAULT_MARGIN_M,
                 step: float = DEFAULT_STEP_M) -> list[tuple[float, float, float]]:
    out: list[tuple[float, float, float]] = []
    for r in robots:
        out.extend((x, y, POINT_Z_M) for x, y in footprint_points(r, margin, step))
    return out


#: "Waiting behind a robot": another robot's centre within this distance of ours, and within ±AHEAD_HALF_ANGLE_DEG of
#: our heading (lab 2026-11-25: waiting for the other robot was reported as NAVIGATION_BLOCKED).
AHEAD_RADIUS_M = 0.8
AHEAD_HALF_ANGLE_DEG = 70.0


def robot_ahead(robots: Iterable[OtherRobot], pose: Optional[tuple[float, float, float]],
                radius: float = AHEAD_RADIUS_M, half_angle_deg: float = AHEAD_HALF_ANGLE_DEG) -> bool:
    """True when another robot stands close in front of `pose` (x, y, yaw); False without a pose."""
    if pose is None:
        return False
    x, y, yaw = pose
    limit = math.radians(half_angle_deg)
    for r in robots:
        dx, dy = r.x - x, r.y - y
        d = math.hypot(dx, dy)
        if d > radius:
            continue
        if d < 1e-6:
            return True
        bearing = math.atan2(dy, dx) - yaw
        if abs(math.atan2(math.sin(bearing), math.cos(bearing))) <= limit:
            return True
    return False


def pointcloud2_fields_xyz(points: list[tuple[float, float, float]]) -> tuple[bytes, int]:
    """(data, width) of an unorganized float32 xyz PointCloud2 body (point_step 12, little endian)."""
    import struct

    return b"".join(struct.pack("<fff", x, y, z) for x, y, z in points), len(points)


class FleetObstaclesState:
    """Latest other robots, with the time they were received (the publisher drops them when they age)."""

    def __init__(self, self_code: str, clock) -> None:
        self._self = self_code
        self._clock = clock
        self._robots: list[OtherRobot] = []
        self._received_at: Optional[float] = None

    def on_message(self, body: dict[str, Any]) -> None:
        self._robots = parse_fleet_poses(body, self._self)
        self._received_at = self._clock()

    def current(self) -> list[OtherRobot]:
        if self._received_at is None:
            return []
        waited = self._clock() - self._received_at
        return [r for r in self._robots if r.pose_age_sec + waited <= MAX_POSE_AGE_SEC]
