"""Drive Assist for manual drive: slow down, then stop, before the robot body reaches an obstacle.

The lidar points (already in base_link) are checked inside a corridor as wide as the robot (plus a margin)
in the direction of travel. `gap` = free distance between the body edge and the nearest point in that corridor.

    gap >= slow_gap          → full requested speed
    stop_gap < gap < slow    → speed scaled linearly down to 0
    gap <= stop_gap          → no motion towards the obstacle (moving away / turning in place is still allowed)

Pure Python (no rclpy) so it is unit-tested with plain point lists.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional


@dataclass(frozen=True)
class Footprint:
    half_length: float  # base_link origin → front / rear edge (m)
    half_width: float  # base_link origin → side edge (m)
    margin: float = 0.05  # extra corridor width each side (m)


@dataclass(frozen=True)
class Gaps:
    front: Optional[float]  # None = nothing seen in the corridor
    rear: Optional[float]


def corridor_gaps(points: Iterable[tuple[float, float]], footprint: Footprint) -> Gaps:
    """Nearest obstacle ahead / behind inside the robot-wide corridor. Points are (x, y) in base_link."""
    half_corridor = footprint.half_width + footprint.margin
    front: Optional[float] = None
    rear: Optional[float] = None
    for x, y in points:
        if abs(y) > half_corridor:
            continue
        if x > footprint.half_length:
            gap = x - footprint.half_length
            front = gap if front is None or gap < front else front
        elif x < -footprint.half_length:
            gap = -x - footprint.half_length
            rear = gap if rear is None or gap < rear else rear
        # else: inside the body outline → the lidar hitting the robot itself (mast, frame); ignored.
    return Gaps(front=front, rear=rear)


def assisted_linear(linear: float, gaps: Optional[Gaps], stop_gap: float, slow_gap: float) -> float:
    """Scale the forward/backward speed by the gap in the direction of travel.

    `gaps is None` means no fresh scan: assist cannot see, so no linear motion (turning in place still works).
    """
    if linear == 0.0:
        return 0.0
    if gaps is None:
        return 0.0
    gap = gaps.front if linear > 0 else gaps.rear
    if gap is None or gap >= slow_gap:
        return linear
    if gap <= stop_gap:
        return 0.0
    return linear * (gap - stop_gap) / (slow_gap - stop_gap)


def scan_to_base_points(
    ranges: Iterable[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    laser_x: float,
    laser_y: float,
    laser_yaw: float,
) -> list[tuple[float, float]]:
    """LaserScan → (x, y) in base_link, given the laser mount pose in base_link (2D)."""
    cos_m, sin_m = math.cos(laser_yaw), math.sin(laser_yaw)
    points: list[tuple[float, float]] = []
    for i, r in enumerate(ranges):
        if not math.isfinite(r) or r < range_min or r > range_max:
            continue
        a = angle_min + i * angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)
        points.append((laser_x + cos_m * lx - sin_m * ly, laser_y + sin_m * lx + cos_m * ly))
    return points
