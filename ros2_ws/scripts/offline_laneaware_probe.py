#!/usr/bin/env python3
"""Offline LaneAware reachability probe on map_one_v001 occupancy + lanes."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/warotrans_simulation/scripts"))
import semantic_map_adapter as sma  # noqa: E402


def load_lethal(map_yaml: Path) -> tuple[set[tuple[int, int]], dict]:
    meta = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    raw = (map_yaml.parent / meta["image"]).read_bytes()
    assert raw.startswith(b"P5")
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
    data = raw[pos : pos + width * height]
    lethal: set[tuple[int, int]] = set()
    for r_top in range(height):
        for c in range(width):
            if data[r_top * width + c] < 50:
                lethal.add((c, height - 1 - r_top))
    return lethal, {
        "w": width,
        "h": height,
        "res": res,
        "ox": ox,
        "oy": oy,
    }


def main() -> int:
    root = Path("/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation")
    lethal, m = load_lethal(root / "maps/map_one_v001/map.yaml")
    lanes = json.loads((root / "maps/map_one_v001/lanes.json").read_text(encoding="utf-8"))["lanes"]
    eps = json.loads((root / "config/map_one_v001_endpoints.json").read_text(encoding="utf-8"))[
        "endpoints"
    ]
    print("map", m, "lethal", len(lethal), "lanes", len(lanes), "eps", len(eps))

    def plan(sx, sy, gx, gy, label):
        path = sma.plan_lane_aware_grid(
            width=m["w"],
            height=m["h"],
            origin_x=m["ox"],
            origin_y=m["oy"],
            resolution=m["res"],
            lethal=lethal,
            start_xy=(sx, sy),
            goal_xy=(gx, gy),
            lanes=lanes,
            margin=0.05,
        )
        print(f"{label}: n={len(path)}")
        return path

    plan(-1.698, -0.825, 1.747, 1.543, "nudge->INBOUND")
    plan(0.0, -0.30, 1.747, 1.543, "center->INBOUND")
    plan(0.0, -0.30, -1.848, -0.525, "center->PARKING")
    plan(1.747, 1.543, -1.325, 1.644, "INBOUND->SHELF-A01")
    plan(-1.848, -0.525, 1.793, -0.977, "PARKING->OUTBOUND")

    # All ordered pairs quick count
    ok = fail = 0
    fails = []
    for a in eps:
        for b in eps:
            if a["code"] == b["code"]:
                continue
            path = sma.plan_lane_aware_grid(
                width=m["w"],
                height=m["h"],
                origin_x=m["ox"],
                origin_y=m["oy"],
                resolution=m["res"],
                lethal=lethal,
                start_xy=(a["x"], a["y"]),
                goal_xy=(b["x"], b["y"]),
                lanes=lanes,
                margin=0.05,
            )
            if len(path) >= 2:
                ok += 1
            else:
                fail += 1
                fails.append((a["code"], b["code"]))
    print(f"pair_ok={ok} pair_fail={fail} total={ok+fail}")
    print("first_fails", fails[:30])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
