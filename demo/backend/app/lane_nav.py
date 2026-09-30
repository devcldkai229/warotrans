"""Lane centerline → Nav2 through-poses (map metres)."""

from __future__ import annotations

import math
from typing import Iterable, List, Optional, Sequence, Tuple

from .session import Lane, TrafficZone, ZonePoint, ZoneType

Point = Tuple[float, float]


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _point_in_poly(x: float, y: float, poly: Sequence[ZonePoint]) -> bool:
    """Ray casting."""
    n = len(poly)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i].x, poly[i].y
        xj, yj = poly[j].x, poly[j].y
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
        ):
            inside = not inside
        j = i
    return inside


def sample_lane_centerline(
    lane: Lane,
    *,
    spacing_m: float = 0.35,
    reverse: bool = False,
) -> List[dict]:
    """Sparse poses along lane centerline. Nav2 still plans between them."""
    sx, sy = lane.startX, lane.startY
    ex, ey = lane.endX, lane.endY
    if reverse:
        sx, sy, ex, ey = ex, ey, sx, sy
    dx, dy = ex - sx, ey - sy
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx) if length > 1e-9 else 0.0
    if length < 1e-6:
        return [{"x": sx, "y": sy, "yaw": yaw}]
    n = max(1, int(math.ceil(length / max(spacing_m, 0.05))))
    poses: List[dict] = []
    for i in range(n + 1):
        t = i / n
        poses.append(
            {
                "x": sx + dx * t,
                "y": sy + dy * t,
                "yaw": yaw,
            }
        )
    return poses


def _near_junction(
    p: Point,
    junctions: Sequence[TrafficZone],
    radius_m: float = 0.45,
) -> bool:
    for z in junctions:
        if not z.enabled or z.type != ZoneType.JUNCTION:
            continue
        if _point_in_poly(p[0], p[1], z.polygon):
            return True
        # also near any vertex
        for v in z.polygon:
            if _dist(p, (v.x, v.y)) <= radius_m:
                return True
    return False


def choose_lane_direction(
    lane: Lane,
    robot: Optional[Point],
) -> bool:
    """Return reverse=True if robot is closer to end (BIDIRECTIONAL only)."""
    if lane.direction.value == "ONE_WAY" or robot is None:
        return False
    d_start = _dist(robot, (lane.startX, lane.startY))
    d_end = _dist(robot, (lane.endX, lane.endY))
    return d_end + 0.05 < d_start


def build_lane_sequence_poses(
    lanes: Sequence[Lane],
    *,
    junctions: Sequence[TrafficZone] = (),
    robot: Optional[Point] = None,
    spacing_m: float = 0.35,
) -> List[dict]:
    """Ordered lanes → through-poses. Junctions allow loose transitions."""
    if not lanes:
        raise ValueError("lane list empty")
    out: List[dict] = []
    prev_end: Optional[Point] = None
    for idx, lane in enumerate(lanes):
        if not lane.enabled:
            raise ValueError(f"lane disabled: {lane.name}")
        reverse = choose_lane_direction(lane, robot if idx == 0 else None)
        # Continuity: prefer matching previous end
        if prev_end is not None and lane.direction.value == "BIDIRECTIONAL":
            d_s = _dist(prev_end, (lane.startX, lane.startY))
            d_e = _dist(prev_end, (lane.endX, lane.endY))
            reverse = d_e + 0.05 < d_s
        poses = sample_lane_centerline(lane, spacing_m=spacing_m, reverse=reverse)
        start_p = (poses[0]["x"], poses[0]["y"])
        if prev_end is not None:
            gap = _dist(prev_end, start_p)
            # Allow junction / nearby connection without MapNode
            if gap > 0.15:
                if gap > 2.5 and not (
                    _near_junction(prev_end, junctions)
                    and _near_junction(start_p, junctions)
                ):
                    raise ValueError(
                        f"lane gap {gap:.2f} m between segments — "
                        "place a JUNCTION or reorder lanes"
                    )
                mid = {
                    "x": (prev_end[0] + start_p[0]) * 0.5,
                    "y": (prev_end[1] + start_p[1]) * 0.5,
                    "yaw": math.atan2(
                        start_p[1] - prev_end[1], start_p[0] - prev_end[0]
                    ),
                }
                out.append(mid)
            # skip duplicate first pose if almost same
            if gap < 0.08:
                poses = poses[1:]
        out.extend(poses)
        if poses:
            prev_end = (poses[-1]["x"], poses[-1]["y"])
        robot = prev_end
    # Deduplicate near-identical consecutive poses
    deduped: List[dict] = []
    for p in out:
        if not deduped:
            deduped.append(p)
            continue
        last = deduped[-1]
        if _dist((last["x"], last["y"]), (p["x"], p["y"])) < 0.05:
            deduped[-1] = p
        else:
            deduped.append(p)
    if len(deduped) < 1:
        raise ValueError("no poses generated")
    return deduped
