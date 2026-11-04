#!/usr/bin/env python3
"""Resolve Gazebo/AMCL spawn pose from published MapVersion endpoints (PARK).

Layout may set:
  spawn_endpoint_code: EP-PARK-1
  spawn: {x,y,yaw}   # optional explicit override (debug only)

Resolution order:
  1. Explicit layout['spawn'] when spawn_endpoint_code is absent
  2. Overlay JSON endpoints (web.overlay) matching spawn_endpoint_code
  3. Optional HTTP GET Navigation endpoints when --backend / WAROTRANS_BACKEND is set
"""

from __future__ import annotations

import json
import math
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Optional

import yaml

# Default sim footprint (Traffic Footprint Length/Width + padding), metres.
DEFAULT_HALF_L = 0.16 + 0.02
DEFAULT_HALF_W = 0.13 + 0.02


def load_yaml(path: str | Path) -> dict[str, Any]:
    with open(path, encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"layout {path!r} must be a mapping")
    return data


def _as_pose(ep: dict[str, Any]) -> dict[str, float]:
    return {
        "x": float(ep["x"]),
        "y": float(ep["y"]),
        "yaw": float(ep.get("yaw") or 0.0),
    }


def load_overlay_endpoints(layout: dict[str, Any], layout_dir: Path) -> list[dict[str, Any]]:
    web = layout.get("web") or {}
    rel = web.get("overlay")
    if not rel:
        return []
    pkg_root = Path(__file__).resolve().parents[1]
    candidates = []
    if os.path.isabs(rel):
        candidates.append(Path(rel))
    else:
        # Layout field is package-relative (config/web/...), not relative to the YAML file dir.
        candidates.extend([pkg_root / rel, layout_dir.parent / rel, layout_dir / rel])
    path = next((p for p in candidates if p.is_file()), None)
    if path is None:
        return []
    data = json.loads(path.read_text(encoding="utf-8"))
    eps = data.get("endpoints") if isinstance(data, dict) else None
    return list(eps) if isinstance(eps, list) else []


def fetch_backend_endpoints(backend: str, map_version_id: str, token: Optional[str] = None) -> list[dict[str, Any]]:
    url = f"{backend.rstrip('/')}/api/navigation/maps/{map_version_id}/endpoints"
    req = urllib.request.Request(url, headers={"Accept": "application/json"})
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(req, timeout=5) as resp:
        body = json.loads(resp.read().decode("utf-8"))
    if isinstance(body, list):
        return body
    if isinstance(body, dict) and isinstance(body.get("items"), list):
        return body["items"]
    return []


def find_endpoint(endpoints: list[dict[str, Any]], code: str) -> Optional[dict[str, Any]]:
    want = code.strip()
    for ep in endpoints:
        if not isinstance(ep, dict):
            continue
        if str(ep.get("code") or "").strip() == want:
            return ep
    return None


def footprint_clear(
    pose: dict[str, float],
    *,
    occupied_cells: Optional[set[tuple[int, int]]] = None,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    resolution: float = 0.05,
    half_l: float = DEFAULT_HALF_L,
    half_w: float = DEFAULT_HALF_W,
) -> tuple[bool, str]:
    """Sample footprint corners against occupancy (row 0 = image top)."""
    if occupied_cells is None:
        return True, "no occupancy check"
    yaw = pose["yaw"]
    c, s = math.cos(yaw), math.sin(yaw)
    # Corners + edge midpoints (large half_* alone can jump over thin walls).
    local_pts = [
        (half_l, half_w),
        (half_l, -half_w),
        (-half_l, half_w),
        (-half_l, -half_w),
        (half_l, 0.0),
        (-half_l, 0.0),
        (0.0, half_w),
        (0.0, -half_w),
        (0.0, 0.0),
    ]
    for lx, ly in local_pts:
        wx = pose["x"] + c * lx - s * ly
        wy = pose["y"] + s * lx + c * ly
        col = int(math.floor((wx - origin_x) / resolution))
        # OccupancyGrid image row 0 is top; world Y increases up → row from top.
        # Without image height we only reject if cell is in occupied set keyed (col,row)
        # using ROS map convention: row from bottom. Prefer caller to pass ROS-style cells.
        row_from_bottom = int(math.floor((wy - origin_y) / resolution))
        if (col, row_from_bottom) in occupied_cells:
            return False, f"footprint hits occupied cell col={col} row={row_from_bottom} at ({wx:.3f},{wy:.3f})"
    return True, "clear"


def min_obstacle_distance(
    pose: dict[str, float],
    *,
    occupied_cells: set[tuple[int, int]],
    origin_x: float,
    origin_y: float,
    resolution: float,
    search_radius_m: float = 1.5,
) -> float:
    """Euclidean distance from pose center to nearest occupied cell center."""
    best = search_radius_m
    cx0 = int(math.floor((pose["x"] - origin_x) / resolution))
    ry0 = int(math.floor((pose["y"] - origin_y) / resolution))
    n = int(search_radius_m / resolution) + 1
    for dc in range(-n, n + 1):
        for dr in range(-n, n + 1):
            if (cx0 + dc, ry0 + dr) not in occupied_cells:
                continue
            wx = origin_x + (cx0 + dc + 0.5) * resolution
            wy = origin_y + (ry0 + dr + 0.5) * resolution
            best = min(best, math.hypot(wx - pose["x"], wy - pose["y"]))
    return best


def nudge_spawn_to_clear(
    pose: dict[str, float],
    *,
    occupied_cells: set[tuple[int, int]],
    origin_x: float,
    origin_y: float,
    resolution: float,
    max_radius_m: float = 1.0,
    step_m: float = 0.05,
    clearance_margin_m: float = 0.08,
    require_standoff: bool = True,
) -> Optional[dict[str, float]]:
    """Search a nearby clear pose (sim footprint) without changing authored endpoint goals.

    MAP-ONE parking bays are tight: full diagonal standoff (~0.31 m) drags the spawn
    south into the wall corridor where Collision Monitor permanently FootprintApproach-holds.
    When require_standoff is False (layout spawn_require_clearance=false), only require a
    footprint-clear cell and prefer staying on the authored Y / moving +X into the aisle.
    """
    base = dict(pose)

    def _foot_ok(p: dict[str, float]) -> bool:
        clear, _ = footprint_clear(
            p,
            occupied_cells=occupied_cells,
            origin_x=origin_x,
            origin_y=origin_y,
            resolution=resolution,
        )
        return clear

    # Keep authored pose when footprint is already clear (do not drag across the map).
    if _foot_ok(base):
        return base

    need = math.hypot(DEFAULT_HALF_L, DEFAULT_HALF_W) + clearance_margin_m

    def _standoff_ok(p: dict[str, float]) -> bool:
        if not _foot_ok(p):
            return False
        return (
            min_obstacle_distance(
                p,
                occupied_cells=occupied_cells,
                origin_x=origin_x,
                origin_y=origin_y,
                resolution=resolution,
            )
            >= need
        )

    def _rank(cand: dict[str, float]) -> tuple[float, float, float]:
        # Prefer: small displacement, stay on authored Y, then move +X (aisle exit).
        dx = cand["x"] - base["x"]
        dy = cand["y"] - base["y"]
        return (math.hypot(dx, dy), abs(dy), -dx)

    best: Optional[tuple[tuple[float, float, float], dict[str, float]]] = None
    n = int(max_radius_m / step_m)
    if require_standoff:
        for iy in range(-n, n + 1):
            for ix in range(-n, n + 1):
                if ix == 0 and iy == 0:
                    continue
                cand = {
                    "x": base["x"] + ix * step_m,
                    "y": base["y"] + iy * step_m,
                    "yaw": base["yaw"],
                }
                if not _standoff_ok(cand):
                    continue
                key = _rank(cand)
                if best is None or key < best[0]:
                    best = (key, cand)
        if best is not None:
            return best[1]
    # Nearest footprint-clear pose (ignore standoff) — used for tight PARK bays.
    best = None
    for iy in range(-n, n + 1):
        for ix in range(-n, n + 1):
            if ix == 0 and iy == 0:
                continue
            cand = {
                "x": base["x"] + ix * step_m,
                "y": base["y"] + iy * step_m,
                "yaw": base["yaw"],
            }
            if not _foot_ok(cand):
                continue
            key = _rank(cand)
            if best is None or key < best[0]:
                best = (key, cand)
    return None if best is None else best[1]


def load_occupied_cells_ros(map_yaml_path: Path) -> tuple[set[tuple[int, int]], dict[str, float]]:
    """Return set of (col, row_from_bottom) occupied cells + map meta.

    Trinary PGM (negate: 0): dark (~0) = occupied, mid (~205) = unknown, bright (~254) = free.
    """
    meta = load_yaml(map_yaml_path)
    image = meta.get("image")
    empty_meta = {
        "origin_x": float((meta.get("origin") or [0, 0, 0])[0]),
        "origin_y": float((meta.get("origin") or [0, 0, 0])[1]),
        "resolution": float(meta.get("resolution") or 0.05),
    }
    if not image:
        return set(), empty_meta
    pgm_path = (map_yaml_path.parent / image).resolve()
    raw = pgm_path.read_bytes()
    if not raw.startswith(b"P5"):
        return set(), empty_meta
    pos = 0
    seen = 0
    width = height = maxval = 0
    while seen < 3 and pos < len(raw):
        nl = raw.find(b"\n", pos)
        if nl < 0:
            break
        line = raw[pos:nl]
        pos = nl + 1
        if line.startswith(b"#"):
            continue
        if seen == 0:
            seen = 1
            continue
        if seen == 1:
            width, height = map(int, line.split())
            seen = 2
            continue
        maxval = int(line)
        seen = 3
    data = raw[pos : pos + width * height]
    if len(data) < width * height:
        return set(), empty_meta
    occupied: set[tuple[int, int]] = set()
    for row_top in range(height):
        for col in range(width):
            if data[row_top * width + col] < 50:
                occupied.add((col, height - 1 - row_top))
    empty_meta.update({"width": float(width), "height": float(height)})
    return occupied, empty_meta


def resolve_spawn(
    layout_path: str | Path,
    *,
    endpoint_code_override: Optional[str] = None,
    backend: Optional[str] = None,
    map_yaml: Optional[str | Path] = None,
    require_clearance: bool = True,
) -> dict[str, Any]:
    """Return {x,y,yaw,source,endpoint_code}."""
    layout_path = Path(layout_path).resolve()
    layout = load_yaml(layout_path)
    layout_dir = layout_path.parent
    code = (endpoint_code_override or layout.get("spawn_endpoint_code") or "").strip()

    if not code:
        spawn = layout.get("spawn")
        if not isinstance(spawn, dict):
            raise ValueError(f"{layout_path}: missing spawn and spawn_endpoint_code")
        pose = _as_pose(spawn)
        return {**pose, "source": "layout.spawn", "endpoint_code": None}

    endpoints = load_overlay_endpoints(layout, layout_dir)
    ep = find_endpoint(endpoints, code)
    source = f"overlay:{code}"

    # Stage-1 / offline layouts may embed endpoints directly (no web.overlay).
    if ep is None:
        layout_eps = layout.get("endpoints")
        if isinstance(layout_eps, list):
            ep = find_endpoint(layout_eps, code)
            if ep is not None:
                source = f"layout.endpoints:{code}"

    if ep is None and backend:
        web = layout.get("web") or {}
        mvid = str(web.get("map_version_id") or "")
        if mvid:
            try:
                endpoints = fetch_backend_endpoints(backend, mvid)
                ep = find_endpoint(endpoints, code)
                source = f"backend:{code}"
            except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
                raise RuntimeError(f"spawn endpoint {code!r}: backend fetch failed: {exc}") from exc

    if ep is None:
        raise RuntimeError(
            f"spawn_endpoint_code={code!r} not found in layout.endpoints / overlay "
            f"{(layout.get('web') or {}).get('overlay')!r} (and backend unavailable/miss)"
        )

    pose = _as_pose(ep)
    # Layout may disable hard fail on PARK clearance; still nudge so LiDAR/Collision Monitor
    # are not permanently in approach-stop (MAP-ONE parking footprint clips occupancy).
    layout_clearance = layout.get("spawn_require_clearance")
    do_clearance = require_clearance if layout_clearance is None else bool(layout_clearance)
    if map_yaml:
        cells, meta = load_occupied_cells_ros(Path(map_yaml))
        ok, reason = footprint_clear(
            pose,
            occupied_cells=cells,
            origin_x=meta.get("origin_x", 0.0),
            origin_y=meta.get("origin_y", 0.0),
            resolution=meta.get("resolution", 0.05),
        )
        if not ok:
            # Tight PARK (spawn_require_clearance=false): footprint-clear only — avoid
            # standoff nudge into south wall corridor that CM permanently holds.
            nudged = nudge_spawn_to_clear(
                pose,
                occupied_cells=cells,
                origin_x=meta.get("origin_x", 0.0),
                origin_y=meta.get("origin_y", 0.0),
                resolution=meta.get("resolution", 0.05),
                require_standoff=do_clearance,
            )
            if nudged is None:
                if do_clearance:
                    raise RuntimeError(f"PARK spawn clearance failed for {code}: {reason}")
            else:
                pose = nudged
                source = f"{source}+nudge"

    return {**pose, "source": source, "endpoint_code": code}


def main(argv: list[str]) -> int:
    import argparse

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--layout", required=True)
    p.add_argument("--endpoint", default="")
    p.add_argument("--backend", default=os.environ.get("WAROTRANS_BACKEND", ""))
    p.add_argument("--map", default="")
    p.add_argument("--json", action="store_true")
    args = p.parse_args(argv)
    pose = resolve_spawn(
        args.layout,
        endpoint_code_override=args.endpoint or None,
        backend=args.backend or None,
        map_yaml=args.map or None,
    )
    if args.json:
        print(json.dumps(pose))
    else:
        print(
            f"spawn x={pose['x']:.4f} y={pose['y']:.4f} yaw={pose['yaw']:.4f} "
            f"source={pose['source']} endpoint={pose.get('endpoint_code')}"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
