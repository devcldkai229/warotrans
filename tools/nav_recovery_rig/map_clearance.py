#!/usr/bin/env python3
"""Does the robot fit where a map sends it? Geometry only: no ROS, no robot, no database writes.

    python tools/nav_recovery_rig/map_clearance.py <bundle dir> <endpoints.csv>

endpoints.csv: code,x,y,yaw per line (psql -t -A -F,). A robot stopped closer than its turning radius to a wall cannot
turn in place there, which is how it ends up stuck (docs/NAV2_STUCK_RECOVERY_EVIDENCE.md).
"""

from __future__ import annotations

import csv
import math
import re
import sys
from pathlib import Path

HALF_LENGTH, HALF_WIDTH, PADDING = 0.16, 0.13, 0.02
TURN_RADIUS = math.hypot(HALF_LENGTH + PADDING, HALF_WIDTH + PADDING)  # padded corner, as the collision monitor sees it
BACKUP = 0.10  # recovery reverse distance


def load_occupied(bundle: Path) -> tuple[list[tuple[float, float]], float]:
    text = (bundle / "map.yaml").read_text(encoding="utf-8")
    res = float(re.search(r"resolution:\s*([\d.]+)", text).group(1))
    ox, oy = [float(v) for v in re.search(r"origin:\s*\[([^\]]+)\]", text).group(1).split(",")[:2]]
    raw = (bundle / re.search(r"image:\s*(\S+)", text).group(1)).read_bytes()
    head = re.match(rb"P5\s+(?:#.*\n)?(\d+)\s+(\d+)\s+(\d+)\s", raw)
    w, h = int(head[1]), int(head[2])
    px = raw[head.end():]
    cells = []
    for row in range(h):
        for col in range(w):
            if px[row * w + col] < 50:
                cells.append((ox + (col + 0.5) * res, oy + (h - row - 0.5) * res))
    return cells, res


def clearance(cells, res: float, x: float, y: float, yaw: float) -> dict:
    """Metres from the centre to the nearest wall cell edge, and from the chassis sides at this heading."""
    half = res / 2
    centre = math.inf
    gaps = {"front": math.inf, "rear": math.inf, "left": math.inf, "right": math.inf}
    c, s = math.cos(yaw), math.sin(yaw)
    for cx, cy in cells:
        dx, dy = cx - x, cy - y
        if abs(dx) > 1.2 or abs(dy) > 1.2:
            continue
        centre = min(centre, max(math.hypot(dx, dy) - half, 0.0))
        bx, by = c * dx + s * dy, -s * dx + c * dy
        if abs(by) <= HALF_WIDTH + half:
            if bx > 0:
                gaps["front"] = min(gaps["front"], bx - HALF_LENGTH - half)
            else:
                gaps["rear"] = min(gaps["rear"], -bx - HALF_LENGTH - half)
        if abs(bx) <= HALF_LENGTH + half:
            if by > 0:
                gaps["left"] = min(gaps["left"], by - HALF_WIDTH - half)
            else:
                gaps["right"] = min(gaps["right"], -by - HALF_WIDTH - half)
    return {"centre": centre, **gaps}


def verdict(c: dict) -> str:
    if min(c["front"], c["rear"], c["left"], c["right"]) < PADDING:
        return "PADDED_FOOTPRINT_ON_WALL"
    if c["centre"] >= TURN_RADIUS:
        return "OK"
    if c["rear"] >= BACKUP + PADDING + 0.03 and c["left"] >= TURN_RADIUS - HALF_WIDTH and c["right"] >= TURN_RADIUS - HALF_WIDTH:
        return "TURN_NEEDS_BACKUP"
    return "CANNOT_TURN_IN_PLACE"


def main() -> int:
    bundle, endpoints = Path(sys.argv[1]), Path(sys.argv[2])
    cells, res = load_occupied(bundle)
    print(f"turning radius (padded) {TURN_RADIUS:.3f} m")
    print(f"{'endpoint':<24}{'centre':>8}{'front':>8}{'rear':>8}{'left':>8}{'right':>8}  verdict")
    worst = 0
    with endpoints.open(encoding="utf-8") as f:
        for row in csv.reader(f):
            if len(row) < 4:
                continue
            code, x, y, yaw = row[0], float(row[1]), float(row[2]), float(row[3])
            c = clearance(cells, res, x, y, yaw)
            v = verdict(c)
            worst += v != "OK"
            cols = "".join(f"{min(c[k], 9.99):8.3f}" for k in ("centre", "front", "rear", "left", "right"))
            print(f"{code:<24}{cols}  {v}")
    print(f"{worst} endpoint(s) need attention")
    return 0


if __name__ == "__main__":
    sys.exit(main())
