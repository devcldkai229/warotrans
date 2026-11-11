"""What a rig run proves, computed from ground truth. Pure Python: no ROS, so it is unit tested anywhere."""

from __future__ import annotations

import math
import re
from typing import Iterable, Sequence

def geometry_body_offset(path: str | None = None) -> float:
    """base_footprint -> base_link x in geometry.yaml (the turning centre is that far behind the chassis centre)."""
    import os

    path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "ros2_ws", "src",
                                "warotrans_description", "config", "geometry.yaml")
    try:
        text = open(path, encoding="utf-8").read()
    except OSError:
        return 0.0
    m = re.search(r"base_footprint_to_base_link:\s*\n(?:\s*#.*\n)*\s*x:\s*([-\d.]+)", text)
    return float(m.group(1)) if m else 0.0


BODY_HALF_LENGTH_M = 0.18
BODY_HALF_WIDTH_M = 0.15
LANE_MAX_OFFSET_M = 0.10
YAW_MAX_ERROR_RAD = 0.14
GOAL_MAX_DISTANCE_M = 0.08  # position_tolerance of every MAP-ONE-V004 endpoint
# Heading within about 10 degrees of the lane: the robot follows it. Cutting across a lane on the way in or out of it,
# with no lane to follow there, is not a following error; that distance is what a report of the max would mostly show.
LANE_FOLLOWING_MIN_COS = 0.985
# Near the ends of a lane the robot is entering or leaving it, so its offset says nothing about following the centre.
LANE_END_MARGIN = 0.15
# The gate's stale-authority limit plus the firmware command watchdog: motion this soon after closing is the stop itself.
GATE_CLOSE_GRACE_S = 0.8
# A body corner this deep in a lane running the other way (heading > 120 deg off its direction) is a corner cut into the
# opposite lane. Crossing a lane at right angles at a junction is not counted: there is no way round it.
OPPOSITE_LANE_MIN_ANGLE_RAD = math.radians(120)
OPPOSITE_LANE_MAX_DEPTH_M = 0.02
# Speed zones: the SpeedFilter updates with the global costmap (1 Hz), so the limit is judged only this far inside.
SPEED_ZONE_INSET_M = 0.10
SPEED_ZONE_TOLERANCE_MPS = 0.005


def _point_to_segment(px: float, py: float, a: Sequence[float], b: Sequence[float]) -> float:
    dx, dy = b[0] - a[0], b[1] - a[1]
    c = dx * dx + dy * dy
    t = 0.0 if c <= 0 else max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / c))
    return math.hypot(px - a[0] - t * dx, py - a[1] - t * dy)


def _body_corners(x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    c, s = math.cos(yaw), math.sin(yaw)
    return [
        (x + u * c - v * s, y + u * s + v * c)
        for u, v in ((BODY_HALF_LENGTH_M, BODY_HALF_WIDTH_M), (BODY_HALF_LENGTH_M, -BODY_HALF_WIDTH_M),
                     (-BODY_HALF_LENGTH_M, -BODY_HALF_WIDTH_M), (-BODY_HALF_LENGTH_M, BODY_HALF_WIDTH_M))
    ]


def _in_ring(x: float, y: float, ring: Sequence[Sequence[float]]) -> bool:
    inside = False
    j = len(ring) - 1
    for i in range(len(ring)):
        xi, yi, xj, yj = ring[i][0], ring[i][1], ring[j][0], ring[j][1]
        if (yi > y) != (yj > y) and x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-15) + xi:
            inside = not inside
        j = i
    return inside


def _in_body(px: float, py: float, x: float, y: float, yaw: float) -> bool:
    c, s = math.cos(yaw), math.sin(yaw)
    u, v = (px - x) * c + (py - y) * s, -(px - x) * s + (py - y) * c
    return abs(u) <= BODY_HALF_LENGTH_M and abs(v) <= BODY_HALF_WIDTH_M


def body_clearance(x: float, y: float, yaw: float, ring: Sequence[Sequence[float]]) -> float:
    """Distance from the body rectangle to the polygon; 0 when they overlap."""
    corners = _body_corners(x, y, yaw)
    if any(_in_ring(cx, cy, ring) for cx, cy in corners) or any(_in_body(p[0], p[1], x, y, yaw) for p in ring):
        return 0.0
    best = math.inf
    n = len(ring)
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        for k in range(4):
            p, q = corners[k], corners[(k + 1) % 4]
            best = min(best, _point_to_segment(p[0], p[1], a, b), _point_to_segment(a[0], a[1], p, q))
    return best


def keepout_clearance(truth: Iterable[dict], keepout_zones: Iterable[dict]) -> float | None:
    """Smallest clearance of the true body from any KEEP_OUT polygon over the run. None without poses or zones."""
    rings = [ring for z in keepout_zones for ring in z.get("rings") or []]
    best = None
    for t in truth:
        for ring in rings:
            d = body_clearance(t["x"], t["y"], t["yaw"], ring)
            best = d if best is None else min(best, d)
    return best


def lane_offsets(truth: Sequence[dict], lanes: Iterable[dict], stride: int = 5) -> dict[str, dict]:
    """Per lane: offset from its centre while driving along it in its direction, and travel against that direction."""
    segs = []
    for lane in lanes:
        code = str(lane["code"]).split("---")[0].replace("EDGE-LANE-", "")
        half = 0.5 * float(lane["width_meters"])
        segs += [(s["a"], s["b"], s["tangent"], code, half) for s in lane["tangent_samples"]]
    out: dict[str, dict] = {}
    for a, b in zip(truth[::stride], truth[stride::stride]):
        if math.hypot(b["x"] - a["x"], b["y"] - a["y"]) < 0.005:
            continue
        heading = math.atan2(b["y"] - a["y"], b["x"] - a["x"])
        for p, q, tangent, code, half in segs:
            dx, dy = q[0] - p[0], q[1] - p[1]
            c = dx * dx + dy * dy
            if c <= 0:
                continue
            t = ((b["x"] - p[0]) * dx + (b["y"] - p[1]) * dy) / c
            if not LANE_END_MARGIN < t < 1.0 - LANE_END_MARGIN:
                continue
            d = _point_to_segment(b["x"], b["y"], p, q)
            if d > half:
                continue
            along = math.cos(heading - tangent)
            row = out.setdefault(code, {"samples": 0, "max": 0.0, "sum": 0.0, "wrong_way": 0})
            if along > LANE_FOLLOWING_MIN_COS:
                row["samples"] += 1
                row["sum"] += d
                row["max"] = max(row["max"], d)
            elif along < -0.85:
                row["wrong_way"] += 1
    return {
        code: {"samples": r["samples"], "mean": round(r["sum"] / r["samples"], 3) if r["samples"] else None,
               "max": round(r["max"], 3), "wrong_way": r["wrong_way"]}
        for code, r in out.items()
    }


def opposite_lane_intrusion(truth: Sequence[dict], lanes: Iterable[dict], stride: int = 2) -> dict[str, float]:
    """Per lane: deepest a body corner went into it while the robot was in another lane, heading against its direction."""
    segs = []
    for lane in lanes:
        code = str(lane["code"]).split("---")[0].replace("EDGE-LANE-", "")
        half = 0.5 * float(lane["width_meters"])
        segs += [(s["a"], s["b"], s["tangent"], code, half) for s in lane["tangent_samples"]]
    out: dict[str, float] = {}
    for t in truth[::stride]:
        for p, q, tangent, code, half in segs:
            if abs(math.atan2(math.sin(t["yaw"] - tangent), math.cos(t["yaw"] - tangent))) < OPPOSITE_LANE_MIN_ANGLE_RAD:
                continue
            if _point_to_segment(t["x"], t["y"], p, q) <= half:
                continue  # the robot is in that lane itself (wrong way is lane_offsets' job)
            dx, dy = q[0] - p[0], q[1] - p[1]
            c = dx * dx + dy * dy
            for cx, cy in _body_corners(t["x"], t["y"], t["yaw"]):
                u = ((cx - p[0]) * dx + (cy - p[1]) * dy) / c if c > 0 else -1.0
                if not 0.0 <= u <= 1.0:
                    continue
                depth = half - _point_to_segment(cx, cy, p, q)
                if depth > 0:
                    out[code] = max(out.get(code, 0.0), round(depth, 3))
    return out


def speed_zone_excess(truth: Sequence[dict], limit_at) -> dict:
    """Fastest speed well inside a speed zone versus its limit. limit_at(x, y) -> m/s, or 0/None outside every zone."""
    worst = {"samples": 0, "max_speed_mps": 0.0, "limit_mps": None, "excess_mps": 0.0}
    for t in truth:
        lim = limit_at(t["x"], t["y"])
        if not lim:
            continue
        r = SPEED_ZONE_INSET_M
        if any(limit_at(t["x"] + ox, t["y"] + oy) != lim for ox, oy in ((r, 0), (-r, 0), (0, r), (0, -r))):
            continue
        worst["samples"] += 1
        v = abs(t["v"])
        if v - lim > worst["excess_mps"] or worst["limit_mps"] is None:
            worst["limit_mps"] = lim
        worst["max_speed_mps"] = round(max(worst["max_speed_mps"], v), 3)
        worst["excess_mps"] = round(max(worst["excess_mps"], v - lim), 3)
    return worst


def speed_mask_reader(yaml_path: str):
    """limit_at(x, y) from a warotrans speed mask (mode raw, limit = pixel * 0.01 m/s, 0 = none)."""
    import os

    meta = {}
    for line in open(yaml_path, encoding="utf-8"):
        if ":" in line and not line.startswith("-"):
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    origin = [float(x) for x in re.findall(r"-?\d+\.?\d*", open(yaml_path, encoding="utf-8").read().split("origin:")[1])[:2]]
    res = float(meta["resolution"])
    data = open(os.path.join(os.path.dirname(yaml_path), meta["image"]), "rb").read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        start = i
        while not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    width, height = int(tokens[1]), int(tokens[2])
    pixels = data[i + 1:]

    def limit_at(x: float, y: float) -> float:
        col = int((x - origin[0]) / res)
        row = height - 1 - int((y - origin[1]) / res)
        if not (0 <= col < width and 0 <= row < height):
            return 0.0
        return pixels[row * width + col] * 0.01

    return limit_at


def log_lines_after(log: str, text: str, wall: float) -> int:
    """Lines containing text and stamped at or after wall. Masks arrive during bring-up: only a goal running without one counts."""
    count = 0
    for line in log.splitlines():
        if text not in line:
            continue
        stamp = re.search(r"\[(\d{9,}\.\d+)\]", line)
        if stamp and float(stamp[1]) >= wall:
            count += 1
    return count


def travel_while_gate_closed(truth: Sequence[dict], events: Iterable[dict]) -> float:
    """Metres driven with the gate closed for longer than it takes to stop."""
    changes = sorted((e["wall"], bool(e["ok"])) for e in events if e.get("e") == "gate")
    closed_since = None
    moved = 0.0
    i = 0
    prev = None
    for t in truth:
        while i < len(changes) and changes[i][0] <= t["wall"]:
            closed_since = None if changes[i][1] else (closed_since if closed_since is not None else changes[i][0])
            i += 1
        if prev is not None and closed_since is not None and t["wall"] - closed_since > GATE_CLOSE_GRACE_S:
            moved += math.hypot(t["x"] - prev["x"], t["y"] - prev["y"])
        prev = t
    return round(moved, 3)


def failures(row: dict) -> list[str]:
    """Why this run does not meet the acceptance table of the lab plan. Reaching the goal is judged by the caller."""
    out = []
    if (row.get("collisions") or 0) > 0:
        out.append(f"collisions={row['collisions']}")
    if row.get("keepout_clearance_m") == 0.0:
        out.append("body in keep-out")
    if row.get("result") == "SUCCEEDED":
        if (row.get("goal_yaw_err") or 0.0) > YAW_MAX_ERROR_RAD:
            out.append(f"yaw error {row['goal_yaw_err']}")
        if (row.get("goal_dist_m") or 0.0) > GOAL_MAX_DISTANCE_M:
            out.append(f"stopped {row['goal_dist_m']} m from the goal")
    if (row.get("travel_gate_closed_m") or 0.0) > 0.02:
        out.append(f"drove {row['travel_gate_closed_m']} m with the gate closed")
    for code, lane in (row.get("lanes") or {}).items():
        if lane["max"] > LANE_MAX_OFFSET_M:
            out.append(f"{code} offset {lane['max']}")
        if lane["wrong_way"]:
            out.append(f"{code} wrong way x{lane['wrong_way']}")
    if (row.get("max_speed_mps") or 0.0) > row.get("speed_ceiling_mps", math.inf):
        out.append(f"speed {row['max_speed_mps']}")
    for code, depth in (row.get("opposite_lane_m") or {}).items():
        if depth > OPPOSITE_LANE_MAX_DEPTH_M:
            out.append(f"corner {depth} m into opposite lane {code}")
    zone = row.get("speed_zone") or {}
    if zone.get("excess_mps", 0.0) > SPEED_ZONE_TOLERANCE_MPS:
        out.append(f"speed zone {zone['max_speed_mps']} > {zone['limit_mps']}")
    return out
