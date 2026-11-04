#!/usr/bin/env python3
"""Semantic Map Adapter — rasterize Zone/Lane policies from a read-only snapshot.

Does not write the database. JUNCTION zones are ignored for keepout/speed masks.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Optional

import yaml

# Nav2 SpeedFilter (warotrans): limit_mps ≈ mask_value * multiplier (0.01).
SPEED_MULTIPLIER = 0.01
# Keepout PGM for map_server trinary (negate=0): black(0)=occupied/keepout, white(254)=free.
# Prior FREE=0 inverted the mask → almost the entire global costmap became LETHAL.
KEEPOUT_LETHAL = 0
FREE = 254
# Speed mask uses mode: raw so encoded limit bytes (1..100) are not trinary-squashed.
SPEED_UNSET = 0


def polygon_rings(geometry: Any) -> list[list[tuple[float, float]]]:
    """GeoJSON Polygon / MultiPolygon → list of exterior rings (x,y)."""
    if not isinstance(geometry, dict):
        return []
    gtype = geometry.get("type")
    coords = geometry.get("coordinates")
    if gtype == "Polygon" and coords:
        return [[(float(p[0]), float(p[1])) for p in coords[0]]]
    if gtype == "MultiPolygon" and coords:
        return [[(float(p[0]), float(p[1])) for p in poly[0]] for poly in coords]
    return []


def point_in_ring(x: float, y: float, ring: list[tuple[float, float]]) -> bool:
    # Ray casting; ring may be open or closed.
    n = len(ring)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-15) + xi):
            inside = not inside
        j = i
    return inside


def write_pgm(path: Path, width: int, height: int, pixels: bytes) -> None:
    header = f"P5\n# warotrans semantic mask\n{width} {height}\n255\n".encode()
    path.write_bytes(header + pixels)


def world_to_pixel(
    x: float, y: float, *, origin_x: float, origin_y: float, resolution: float, height: int
) -> tuple[int, int]:
    col = int(math.floor((x - origin_x) / resolution))
    # PGM row 0 = top = max Y
    row_from_bottom = int(math.floor((y - origin_y) / resolution))
    row = height - 1 - row_from_bottom
    return col, row


def rasterize_keepout(
    zones: Iterable[dict[str, Any]],
    *,
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> bytes:
    grid = bytearray([FREE]) * (width * height)
    for z in zones:
        if str(z.get("zone_type") or z.get("zoneType") or "").upper() != "KEEP_OUT":
            continue
        if z.get("is_active") is False:
            continue
        for ring in polygon_rings(z.get("geometry")):
            # Bounding box scan
            xs = [p[0] for p in ring]
            ys = [p[1] for p in ring]
            c0, r0 = world_to_pixel(min(xs), max(ys), origin_x=origin_x, origin_y=origin_y, resolution=resolution, height=height)
            c1, r1 = world_to_pixel(max(xs), min(ys), origin_x=origin_x, origin_y=origin_y, resolution=resolution, height=height)
            for row in range(max(0, min(r0, r1)), min(height, max(r0, r1) + 1)):
                for col in range(max(0, min(c0, c1)), min(width, max(c0, c1) + 1)):
                    wx = origin_x + (col + 0.5) * resolution
                    wy = origin_y + (height - row - 0.5) * resolution
                    if point_in_ring(wx, wy, ring):
                        grid[row * width + col] = KEEPOUT_LETHAL
    return bytes(grid)


def rasterize_speed(
    zones: Iterable[dict[str, Any]],
    *,
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
    default_limit_mps: float = 1.0,
) -> bytes:
    """Most restrictive SPEED wins. Pixel = round(limit_mps / SPEED_MULTIPLIER), clipped 1..100.

    Cells with no SPEED zone stay 0 (no limit from filter).
    """
    # Store limit*100 as float then convert; 0 = unset
    limits = [0.0] * (width * height)
    for z in zones:
        if str(z.get("zone_type") or z.get("zoneType") or "").upper() != "SPEED":
            continue
        if z.get("is_active") is False:
            continue
        raw = z.get("max_speed") if z.get("max_speed") is not None else z.get("maxSpeed")
        if raw is None:
            continue
        lim = float(raw)
        if lim <= 0:
            continue
        for ring in polygon_rings(z.get("geometry")):
            xs = [p[0] for p in ring]
            ys = [p[1] for p in ring]
            c0, r0 = world_to_pixel(min(xs), max(ys), origin_x=origin_x, origin_y=origin_y, resolution=resolution, height=height)
            c1, r1 = world_to_pixel(max(xs), min(ys), origin_x=origin_x, origin_y=origin_y, resolution=resolution, height=height)
            for row in range(max(0, min(r0, r1)), min(height, max(r0, r1) + 1)):
                for col in range(max(0, min(c0, c1)), min(width, max(c0, c1) + 1)):
                    wx = origin_x + (col + 0.5) * resolution
                    wy = origin_y + (height - row - 0.5) * resolution
                    if not point_in_ring(wx, wy, ring):
                        continue
                    i = row * width + col
                    if limits[i] <= 0:
                        limits[i] = lim
                    else:
                        limits[i] = min(limits[i], lim)
    out = bytearray([SPEED_UNSET]) * (width * height)
    for i, lim in enumerate(limits):
        if lim <= 0:
            continue
        val = int(round(lim / SPEED_MULTIPLIER))
        out[i] = max(1, min(100, val))
    return bytes(out)


def export_lanes(edges: Iterable[dict[str, Any]]) -> list[dict[str, Any]]:
    lanes = []
    for e in edges:
        if e.get("is_active") is False:
            continue
        geom = e.get("geometry") or {}
        if geom.get("type") != "LineString":
            continue
        coords = [(float(p[0]), float(p[1])) for p in geom.get("coordinates") or []]
        if len(coords) < 2:
            continue
        direction = str(e.get("direction") or "BIDIRECTIONAL").upper()
        tangents = []
        tangent_samples = []
        for i in range(len(coords) - 1):
            ax, ay = coords[i]
            bx, by = coords[i + 1]
            ang = math.atan2(by - ay, bx - ax)
            tangents.append(ang)
            tangent_samples.append(
                {
                    "a": [ax, ay],
                    "b": [bx, by],
                    "tangent": ang,
                }
            )
        tangents.append(tangents[-1])
        lanes.append(
            {
                "code": str(e.get("code") or ""),
                "direction": direction,
                "width_meters": float(e.get("width_meters") or e.get("widthMeters") or 0.5),
                "coordinates": [[c[0], c[1]] for c in coords],
                "tangents_rad": tangents,
                "tangent_samples": tangent_samples,
            }
        )
    return lanes


def export_zone_list(zones: Iterable[dict[str, Any]], zone_type: str) -> list[dict[str, Any]]:
    want = zone_type.upper()
    out = []
    for z in zones:
        zt = str(z.get("zone_type") or z.get("zoneType") or "").upper()
        if zt != want:
            continue
        if z.get("is_active") is False:
            continue
        rings = polygon_rings(z.get("geometry"))
        if not rings:
            continue
        out.append(
            {
                "id": str(z.get("id") or ""),
                "code": str(z.get("code") or ""),
                "name": str(z.get("name") or ""),
                "zone_type": zt,
                "max_speed": z.get("max_speed") if z.get("max_speed") is not None else z.get("maxSpeed"),
                "rings": [[[p[0], p[1]] for p in ring] for ring in rings],
            }
        )
    return out


def dist_point_to_segment(
    px: float, py: float, ax: float, ay: float, bx: float, by: float
) -> tuple[float, float, float]:
    """Return (distance, t in [0,1], tangent_angle)."""
    abx, aby = bx - ax, by - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-18:
        return math.hypot(px - ax, py - ay), 0.0, 0.0
    t = max(0.0, min(1.0, ((px - ax) * abx + (py - ay) * aby) / ab2))
    qx, qy = ax + t * abx, ay + t * aby
    return math.hypot(px - qx, py - qy), t, math.atan2(aby, abx)


def lane_membership(
    x: float,
    y: float,
    lanes: list[dict[str, Any]],
    *,
    margin: float = 0.05,
) -> Optional[dict[str, Any]]:
    """If (x,y) is inside a lane corridor, return {direction, tangent_rad, code, dist}."""
    best = None
    best_d = float("inf")
    for lane in lanes:
        coords = lane["coordinates"]
        half = float(lane["width_meters"]) / 2.0 + margin
        for i in range(len(coords) - 1):
            ax, ay = coords[i]
            bx, by = coords[i + 1]
            d, t, ang = dist_point_to_segment(x, y, ax, ay, bx, by)
            if d <= half and d < best_d:
                best_d = d
                # Prefer stored tangent when available
                tangents = lane.get("tangents_rad") or []
                if tangents and i < len(tangents):
                    ang = float(tangents[i])
                best = {
                    "code": lane["code"],
                    "direction": lane["direction"],
                    "tangent_rad": ang,
                    "dist": d,
                }
    return best


def move_allowed(
    x0: float,
    y0: float,
    x1: float,
    y1: float,
    lanes: list[dict[str, Any]],
    *,
    margin: float = 0.05,
) -> bool:
    """Hard ONE_WAY rule: travel against lane tangent inside corridor is invalid."""
    mid_x, mid_y = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    mem = lane_membership(mid_x, mid_y, lanes, margin=margin)
    if mem is None:
        # Also check destination cell (entering corridor)
        mem = lane_membership(x1, y1, lanes, margin=margin)
    if mem is None:
        return True
    if mem["direction"] != "ONE_WAY":
        return True
    move_ang = math.atan2(y1 - y0, x1 - x0)
    # ONE_WAY: forward half-plane + near-perpendicular crossing (|along|<0.25).
    # Strict ±90° alone blocked NNW cells and funneled robots to the east pinch;
    # reverse-dominant-only allowed 8-connected zigzag reverse — reject that too.
    d = move_ang - float(mem["tangent_rad"])
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    along = math.cos(d)
    if abs(along) < 0.25:
        return True
    return along >= -1e-9


def footprint_hits_polygons(
    x: float,
    y: float,
    yaw: float,
    rings_list: list[list[list[float]]],
    *,
    half_l: float = 0.18,
    half_w: float = 0.15,
) -> bool:
    c, s = math.cos(yaw), math.sin(yaw)
    corners = [
        (half_l, half_w),
        (half_l, -half_w),
        (-half_l, half_w),
        (-half_l, -half_w),
        (0.0, 0.0),
    ]
    for lx, ly in corners:
        wx = x + c * lx - s * ly
        wy = y + s * lx + c * ly
        for rings in rings_list:
            for ring in rings:
                pts = [(float(p[0]), float(p[1])) for p in ring]
                if point_in_ring(wx, wy, pts):
                    return True
    return False


def validate_path_poses(
    poses: list[tuple[float, float, float]],
    *,
    lanes: list[dict[str, Any]],
    keepout_rings: list[list[list[float]]],
    no_stop_rings: list[list[list[float]]],
    check_no_stop_goal: bool = True,
    margin: float = 0.05,
) -> tuple[bool, str]:
    """Defense-in-depth path policy. poses = (x,y,yaw)."""
    if len(poses) < 2:
        return True, "trivial"
    for i in range(len(poses) - 1):
        x0, y0, _ = poses[i]
        x1, y1, yaw1 = poses[i + 1]
        if not move_allowed(x0, y0, x1, y1, lanes, margin=margin):
            return False, f"ONE_WAY_VIOLATION at segment {i}"
        if footprint_hits_polygons(x1, y1, yaw1, keepout_rings):
            return False, f"KEEP_OUT_VIOLATION at pose {i + 1}"
    if check_no_stop_goal:
        gx, gy, gyaw = poses[-1]
        if footprint_hits_polygons(gx, gy, gyaw, no_stop_rings):
            return False, "NO_STOP_GOAL"
    return True, "path_ok"


def plan_lane_aware_grid(
    *,
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
    lethal: set[tuple[int, int]],
    start_xy: tuple[float, float],
    goal_xy: tuple[float, float],
    lanes: list[dict[str, Any]],
    margin: float = 0.05,
) -> list[tuple[float, float]]:
    """Minimal 8-connected A* with the same ONE_WAY corridor rule as LaneAwareNavfn (unit tests)."""
    import heapq

    def w2c(x: float, y: float) -> tuple[int, int]:
        c = int(math.floor((x - origin_x) / resolution))
        r = int(math.floor((y - origin_y) / resolution))
        return c, r

    def c2w(c: int, r: int) -> tuple[float, float]:
        return origin_x + (c + 0.5) * resolution, origin_y + (r + 0.5) * resolution

    sc, sr = w2c(*start_xy)
    gc, gr = w2c(*goal_xy)
    if not (0 <= sc < width and 0 <= sr < height and 0 <= gc < width and 0 <= gr < height):
        return []
    if (sc, sr) in lethal or (gc, gr) in lethal:
        return []

    nbrs = [(1, 0, 1.0), (1, 1, 1.414), (0, 1, 1.0), (-1, 1, 1.414), (-1, 0, 1.0), (-1, -1, 1.414), (0, -1, 1.0), (1, -1, 1.414)]
    start = (sc, sr)
    goal = (gc, gr)
    open_h: list[tuple[float, float, tuple[int, int]]] = [(0.0, 0.0, start)]
    gscore = {start: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    found = False
    while open_h:
        _, g, cur = heapq.heappop(open_h)
        if cur == goal:
            found = True
            break
        if g > gscore.get(cur, float("inf")) + 1e-9:
            continue
        x0, y0 = c2w(*cur)
        for dx, dy, step in nbrs:
            nxt = (cur[0] + dx, cur[1] + dy)
            if not (0 <= nxt[0] < width and 0 <= nxt[1] < height):
                continue
            if nxt in lethal:
                continue
            x1, y1 = c2w(*nxt)
            if not move_allowed(x0, y0, x1, y1, lanes, margin=margin):
                continue
            ng = g + step
            if ng + 1e-9 < gscore.get(nxt, float("inf")):
                gscore[nxt] = ng
                parent[nxt] = cur
                h = math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                heapq.heappush(open_h, (ng + h, ng, nxt))
    if not found:
        return []
    rev = [goal]
    while rev[-1] != start:
        rev.append(parent[rev[-1]])
    rev.reverse()
    return [c2w(c, r) for c, r in rev]


def semantic_fingerprint(parts: dict[str, Any]) -> str:
    blob = json.dumps(parts, sort_keys=True, separators=(",", ":")).encode()
    return hashlib.sha256(blob).hexdigest()


def write_mask_yaml(
    path: Path,
    *,
    image: str,
    resolution: float,
    origin: list[float],
    mode: str = "trinary",
) -> None:
    meta = {
        "image": image,
        "mode": mode,
        "resolution": resolution,
        "origin": origin,
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    path.write_text(yaml.safe_dump(meta, sort_keys=False), encoding="utf-8")


def export_from_snapshot(
    snap: dict[str, Any],
    *,
    width: int,
    height: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
    out_dir: Path,
    occupancy_sha256: str,
    planner_id: str = "warotrans_lane_planner::LaneAwareNavfn",
) -> dict[str, Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    zones = snap.get("zones") or []
    edges = snap.get("edges") or []
    endpoints = snap.get("endpoints") or []

    keepout_px = rasterize_keepout(
        zones, width=width, height=height, origin_x=origin_x, origin_y=origin_y, resolution=resolution
    )
    speed_px = rasterize_speed(
        zones, width=width, height=height, origin_x=origin_x, origin_y=origin_y, resolution=resolution
    )
    keepout_pgm = out_dir / "keepout.pgm"
    speed_pgm = out_dir / "speed.pgm"
    write_pgm(keepout_pgm, width, height, keepout_px)
    write_pgm(speed_pgm, width, height, speed_px)
    origin = [origin_x, origin_y, 0.0]
    write_mask_yaml(
        out_dir / "keepout.yaml",
        image="keepout.pgm",
        resolution=resolution,
        origin=origin,
        mode="trinary",
    )
    write_mask_yaml(
        out_dir / "speed.yaml",
        image="speed.pgm",
        resolution=resolution,
        origin=origin,
        mode="raw",
    )

    lanes = export_lanes(edges)
    lanes_path = out_dir / "lanes.json"
    lanes_path.write_text(json.dumps({"lanes": lanes, "margin_meters": 0.05}, indent=2) + "\n", encoding="utf-8")

    keepout_zones = export_zone_list(zones, "KEEP_OUT")
    keepout_json_path = out_dir / "keepout.json"
    keepout_json_path.write_text(json.dumps({"zones": keepout_zones}, indent=2) + "\n", encoding="utf-8")

    no_stop = export_zone_list(zones, "NO_STOP")
    no_stop_path = out_dir / "no_stop.json"
    no_stop_path.write_text(json.dumps({"zones": no_stop}, indent=2) + "\n", encoding="utf-8")

    single = export_zone_list(zones, "SINGLE_OCCUPANCY")
    single_path = out_dir / "single_occupancy.json"
    single_path.write_text(
        json.dumps({"zones": single, "enforcement": "NOT_SUPPORTED_STAGE2"}, indent=2) + "\n",
        encoding="utf-8",
    )

    ep_canon = [
        {
            "code": e.get("code"),
            "x": e.get("x"),
            "y": e.get("y"),
            "yaw": e.get("yaw"),
        }
        for e in sorted(endpoints, key=lambda z: str(z.get("code") or ""))
        if e.get("code")
    ]
    fp = semantic_fingerprint(
        {
            "occupancySha256": occupancy_sha256,
            "lanes": lanes,
            "keepoutCodes": [z["code"] for z in export_zone_list(zones, "KEEP_OUT")],
            "speed": [
                {"code": z["code"], "max_speed": z["max_speed"]} for z in export_zone_list(zones, "SPEED")
            ],
            "no_stop": [z["code"] for z in no_stop],
            "single_occupancy": [z["code"] for z in single],
            "endpoints": ep_canon,
            "footprint": {"length": 0.32, "width": 0.26, "padding": 0.02},
            "planner_id": planner_id,
            "resolution": resolution,
            "origin": origin,
        }
    )
    manifest_path = out_dir / "semantic_manifest.json"
    prior = {}
    if manifest_path.is_file():
        try:
            prior = json.loads(manifest_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            prior = {}
    prior.update(
        {
            "semanticMapFingerprint": fp,
            "plannerId": planner_id,
            "stage": 2,
            "policies": {
                "keepout": "keepout.yaml",
                "keepout_zones": "keepout.json",
                "speed": "speed.yaml",
                "lanes": "lanes.json",
                "no_stop": "no_stop.json",
                "single_occupancy": "single_occupancy.json",
            },
            "note": "Stage-2 fingerprint includes lanes/zones/endpoints/yaw; SINGLE_OCCUPANCY not enforced",
        }
    )
    manifest_path.write_text(json.dumps(prior, indent=2) + "\n", encoding="utf-8")

    return {
        "keepout_yaml": out_dir / "keepout.yaml",
        "keepout_json": keepout_json_path,
        "speed_yaml": out_dir / "speed.yaml",
        "lanes": lanes_path,
        "no_stop": no_stop_path,
        "single_occupancy": single_path,
        "manifest": manifest_path,
    }
