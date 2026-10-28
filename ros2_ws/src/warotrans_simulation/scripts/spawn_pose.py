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
    corners = [
        (half_l, half_w),
        (half_l, -half_w),
        (-half_l, half_w),
        (-half_l, -half_w),
        (0.0, 0.0),
    ]
    for lx, ly in corners:
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
            f"spawn_endpoint_code={code!r} not found in overlay "
            f"{(layout.get('web') or {}).get('overlay')!r} (and backend unavailable/miss)"
        )

    pose = _as_pose(ep)
    if require_clearance and map_yaml:
        cells, meta = load_occupied_cells_ros(Path(map_yaml))
        ok, reason = footprint_clear(
            pose,
            occupied_cells=cells,
            origin_x=meta.get("origin_x", 0.0),
            origin_y=meta.get("origin_y", 0.0),
            resolution=meta.get("resolution", 0.05),
        )
        if not ok:
            raise RuntimeError(f"PARK spawn clearance failed for {code}: {reason}")

    # #region agent log
    try:
        import json
        import time
        from pathlib import Path as _P

        _log = _P("/mnt/e/programming/my_project/warotrans-system/debug-100d90.log")
        with _log.open("a", encoding="utf-8") as _f:
            _f.write(
                json.dumps(
                    {
                        "sessionId": "100d90",
                        "runId": "spawn-resolve",
                        "hypothesisId": "B",
                        "location": "spawn_pose.py:resolve_spawn",
                        "message": "resolved spawn for Gazebo/AMCL",
                        "data": {**pose, "source": source, "endpoint_code": code},
                        "timestamp": int(time.time() * 1000),
                    }
                )
                + "\n"
            )
    except Exception:
        pass
    # #endregion

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
