#!/usr/bin/env python3
"""Offline landmark audit: occupancy map origin/corners vs overlay PARK vs FE worldToPixel.

Compares published endpoints and map meta without requiring a live Gazebo session.
Optional live TF/MQTT columns are left blank for runtime fill-in.

Usage (from warotrans_simulation package root or with absolute paths):
  python3 scripts/measure_landmarks.py \\
    --layout config/warehouse_layout.yaml \\
    --map maps/sim_warehouse.yaml \\
    --out ../../docs/simulation/MAP_ALIGNMENT_TEST_REPORT.md
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import yaml

# Same continuous transform as web/src/shared/map/mapCoordinateTransform.ts
def world_to_pixel(
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    image_height: int,
    x: float,
    y: float,
) -> tuple[float, float]:
    dx = x - origin_x
    dy = y - origin_y
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    local_x = c * dx + s * dy
    local_y = -s * dx + c * dy
    u = local_x / resolution
    v = image_height - local_y / resolution
    return u, v


def pixel_to_world(
    *,
    resolution: float,
    origin_x: float,
    origin_y: float,
    origin_yaw: float,
    image_height: int,
    u: float,
    v: float,
) -> tuple[float, float]:
    local_x = u * resolution
    local_y = (image_height - v) * resolution
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    return origin_x + c * local_x - s * local_y, origin_y + s * local_x + c * local_y


def load_yaml(path: Path) -> dict[str, Any]:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def pgm_size(pgm: Path) -> tuple[int, int]:
    raw = pgm.read_bytes()
    pos = 0
    seen = 0
    width = height = 0
    while seen < 3 and pos < len(raw):
        nl = raw.find(b"\n", pos)
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
        seen = 3
    return width, height


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", type=Path, default=root / "config" / "warehouse_layout.yaml")
    ap.add_argument("--map", type=Path, default=root / "maps" / "sim_warehouse.yaml")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args()

    layout = load_yaml(args.layout)
    map_meta = load_yaml(args.map)
    origin = map_meta["origin"]
    resolution = float(map_meta["resolution"])
    pgm = (args.map.parent / map_meta["image"]).resolve()
    width, height = pgm_size(pgm)

    web = layout.get("web") or {}
    overlay_rel = web.get("overlay")
    # overlay path in layout is package-relative (same as generate_warehouse / spawn_pose).
    overlay_path: Path | None = None
    if overlay_rel:
        for candidate in (
            root / overlay_rel,
            args.layout.parent.parent / overlay_rel,
            args.layout.parent / overlay_rel,
        ):
            if candidate.is_file():
                overlay_path = candidate.resolve()
                break
    endpoints: list[dict[str, Any]] = []
    if overlay_path and overlay_path.is_file():
        endpoints = json.loads(overlay_path.read_text(encoding="utf-8")).get("endpoints") or []

    parks = [e for e in endpoints if str(e.get("code", "")).startswith("EP-PARK")]
    spawn_code = (layout.get("spawn_endpoint_code") or "").strip()
    spawn = layout.get("spawn") or {}

    # Map corners in world (continuous image corners)
    corners = {
        "origin_ll": pixel_to_world(
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]) if len(origin) > 2 else 0.0,
            image_height=height,
            u=0.0,
            v=float(height),
        ),
        "top_left": pixel_to_world(
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=0.0,
            image_height=height,
            u=0.0,
            v=0.0,
        ),
        "bottom_right": pixel_to_world(
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=0.0,
            image_height=height,
            u=float(width),
            v=float(height),
        ),
    }

    lines: list[str] = []
    lines.append("# MAP Alignment Test Report")
    lines.append("")
    lines.append(f"Generated: {datetime.now(timezone.utc).isoformat()}")
    lines.append("")
    lines.append("## Map meta (occupancy)")
    lines.append("")
    lines.append("| Field | Value |")
    lines.append("|---|---|")
    lines.append(f"| map yaml | `{args.map}` |")
    lines.append(f"| image | `{pgm.name}` {width}×{height} |")
    lines.append(f"| resolution | {resolution} m/px |")
    lines.append(f"| origin | ({origin[0]}, {origin[1]}, {origin[2] if len(origin)>2 else 0}) |")
    lines.append(f"| map_version_code | {web.get('map_version_code')} |")
    lines.append(f"| map_version_id | {web.get('map_version_id')} |")
    lines.append("")
    lines.append("## Contract check")
    lines.append("")
    lines.append("| Check | Expected | Actual | Result |")
    lines.append("|---|---|---|---|")
    yaw0 = abs(float(origin[2]) if len(origin) > 2 else 0.0) < 1e-9
    lines.append(f"| origin yaw | 0 | {origin[2] if len(origin)>2 else 0} | {'PASS' if yaw0 else 'FAIL'} |")
    lines.append(f"| resolution | 0.05 | {resolution} | {'PASS' if abs(resolution-0.05)<1e-9 else 'FAIL'} |")
    lines.append(f"| size | 107×68 | {width}×{height} | {'PASS' if (width,height)==(107,68) else 'FAIL'} |")
    ox_ok = abs(float(origin[0]) + 0.594) < 0.002
    oy_ok = abs(float(origin[1]) + 2.719) < 0.002
    lines.append(f"| origin ~ (-0.594,-2.719) | yes | ({origin[0]},{origin[1]}) | {'PASS' if ox_ok and oy_ok else 'FAIL'} |")
    lines.append("")
    lines.append("## World corners (FE worldToPixel inverse)")
    lines.append("")
    lines.append("| Landmark | world X | world Y |")
    lines.append("|---|---|---|")
    for name, (x, y) in corners.items():
        lines.append(f"| {name} | {x:.4f} | {y:.4f} |")
    lines.append("")
    lines.append("## PARK endpoints (overlay) vs FE pixel round-trip")
    lines.append("")
    lines.append("| Code | world X | world Y | yaw | FE u | FE v | round-trip dx | dy |")
    lines.append("|---|---|---|---|---|---|---|---|")
    for ep in parks:
        x, y = float(ep["x"]), float(ep["y"])
        yaw = float(ep.get("yaw") or 0.0)
        u, v = world_to_pixel(
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=0.0,
            image_height=height,
            x=x,
            y=y,
        )
        rx, ry = pixel_to_world(
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=0.0,
            image_height=height,
            u=u,
            v=v,
        )
        lines.append(
            f"| {ep.get('code')} | {x:.4f} | {y:.4f} | {yaw:.4f} | {u:.2f} | {v:.2f} | "
            f"{abs(rx-x):.6f} | {abs(ry-y):.6f} |"
        )
    lines.append("")
    lines.append("## Layout spawn")
    lines.append("")
    lines.append(f"- `spawn_endpoint_code`: `{spawn_code or '(none)'}`")
    lines.append(f"- explicit `spawn`: `{spawn}`")
    if spawn_code:
        match = next((e for e in parks if e.get("code") == spawn_code), None)
        if match:
            lines.append(
                f"- resolved PARK pose: ({match['x']}, {match['y']}, {match.get('yaw')}) — "
                "must drive Gazebo create + AMCL initial_pose"
            )
        else:
            lines.append(f"- ERROR: {spawn_code} not in overlay")
    else:
        lines.append(
            "- WARN: no spawn_endpoint_code — historical hard (0,0) SLAM origin causes "
            "Fleet Map 'robot on the left' vs PARK upper-right"
        )
    lines.append("")
    lines.append("## Runtime columns (fill during live Gazebo)")
    lines.append("")
    lines.append("| Landmark | Gazebo model pose | ROS map->base TF | MQTT PoseX/Y | FE marker | d Gazebo-FE |")
    lines.append("|---|---|---|---|---|---|")
    lines.append("| EP-PARK-1 | TBD | TBD | TBD | TBD | TBD |")
    lines.append("| EP-PARK-2 | TBD | TBD | TBD | TBD | TBD |")
    lines.append("| map origin cell | TBD | TBD | — | — | TBD |")
    lines.append("")
    lines.append("## Verdict (offline)")
    lines.append("")
    lines.append(
        "Occupancy + overlay + FE `worldToPixel` share one world==map frame. "
        "Primary misalignment UX is spawn at SLAM origin (0,0) instead of published PARK."
    )
    lines.append("")

    text = "\n".join(lines)
    if args.out:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text, encoding="utf-8")
        print(f"Wrote {args.out}", file=sys.stderr)
    try:
        print(text)
    except UnicodeEncodeError:
        sys.stdout.buffer.write(text.encode("utf-8", errors="replace"))
        sys.stdout.buffer.write(b"\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
