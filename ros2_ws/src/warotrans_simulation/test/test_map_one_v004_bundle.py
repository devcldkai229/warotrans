"""The MAP-ONE-V004 bundle in the repo is what the robot and the backend load: pin what it must contain."""

from __future__ import annotations

import json
import math
import re
from collections import Counter
from pathlib import Path

import pytest

BUNDLE = Path(__file__).resolve().parents[2] / "warotrans_navigation" / "maps" / "map_one_v004"
FOOTPRINT_RADIUS_M = math.hypot(0.18, 0.15)
BODY_HALF_WIDTH_M = 0.15
BODY_HALF_LENGTH_M = 0.18
ROBOT_CRUISE_MPS = 0.10
SPEED_MULTIPLIER = 0.01


def _pgm(name: str) -> tuple[int, int, bytes]:
    raw = (BUNDLE / name).read_bytes()
    m = re.match(rb"P5\s+(?:#.*\s+)*(\d+)\s+(\d+)\s+(\d+)\s", raw)
    return int(m[1]), int(m[2]), raw[m.end():]


def _meta() -> tuple[float, float, float]:
    manifest = json.loads((BUNDLE / "semantic_manifest.json").read_text(encoding="utf-8"))
    return manifest["resolution"], manifest["origin"][0], manifest["origin"][1]


def _keepout_ring() -> list[list[float]]:
    zones = json.loads((BUNDLE / "keepout.json").read_text(encoding="utf-8"))["zones"]
    assert len(zones) == 1
    return zones[0]["rings"][0]


def _distance_to_ring(x: float, y: float, ring: list[list[float]]) -> float:
    best = math.inf
    for (ax, ay), (bx, by) in zip(ring, ring[1:]):
        dx, dy = bx - ax, by - ay
        c = dx * dx + dy * dy
        t = 0.0 if c == 0 else max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / c))
        best = min(best, math.hypot(x - ax - t * dx, y - ay - t * dy))
    return best


def _lethal_cells(name: str):
    width, height, data = _pgm(name)
    res, ox, oy = _meta()
    for row in range(height):
        for col in range(width):
            if data[row * width + col] < 100:  # trinary, negate 0: dark is occupied
                yield ox + (col + 0.5) * res, oy + (height - row - 0.5) * res


def test_speed_mask_never_raises_the_robot_above_its_normal_speed():
    _, _, data = _pgm("speed.pgm")
    limits = {v * SPEED_MULTIPLIER for v in Counter(data) if v > 0}
    assert limits, "SPEED-ZONE-C must be in the mask"
    assert max(limits) <= ROBOT_CRUISE_MPS + 1e-9


def test_controller_mask_is_the_zone_as_drawn():
    ring = _keepout_ring()
    res, _, _ = _meta()
    cells = list(_lethal_cells("keepout.pgm"))
    assert cells
    # A wider mask counts the body twice: RPP checks the footprint against it and stops on the lanes beside the zone.
    assert all(_distance_to_ring(x, y, ring) <= res for x, y in cells)


def test_planning_mask_keeps_every_heading_of_the_body_out():
    ring = _keepout_ring()
    res, ox, oy = _meta()
    width, height, _ = _pgm("keepout_planning.pgm")
    lethal = set(_lethal_cells("keepout_planning.pgm"))
    assert lethal
    for row in range(height):
        for col in range(width):
            x, y = ox + (col + 0.5) * res, oy + (height - row - 0.5) * res
            if (x, y) not in lethal:
                assert _distance_to_ring(x, y, ring) > FOOTPRINT_RADIUS_M, (x, y)


def test_every_endpoint_and_lane_is_free_in_the_planning_mask():
    ring = _keepout_ring()
    snapshot_lanes = json.loads((BUNDLE / "lanes.json").read_text(encoding="utf-8"))["lanes"]
    for lane in snapshot_lanes:
        (ax, ay), (bx, by) = lane["coordinates"][0], lane["coordinates"][-1]
        steps = max(1, int(math.dist((ax, ay), (bx, by)) / 0.02))
        for i in range(steps + 1):
            x, y = ax + (bx - ax) * i / steps, ay + (by - ay) * i / steps
            assert _distance_to_ring(x, y, ring) > FOOTPRINT_RADIUS_M, (lane["code"], x, y)


@pytest.mark.parametrize("code", ["LTR-A", "RTL-PARKING"])
def test_body_on_the_lanes_beside_the_zone_clears_the_controller_mask(code):
    lanes = json.loads((BUNDLE / "lanes.json").read_text(encoding="utf-8"))["lanes"]
    lane = next(x for x in lanes if code in x["code"])
    (ax, ay), (bx, by) = lane["coordinates"][0], lane["coordinates"][-1]
    theta = math.atan2(by - ay, bx - ax)
    res, _, _ = _meta()
    cells = list(_lethal_cells("keepout.pgm"))
    half_cell = res / 2.0
    steps = max(1, int(math.dist((ax, ay), (bx, by)) / 0.05))
    for i in range(steps + 1):
        x, y = ax + (bx - ax) * i / steps, ay + (by - ay) * i / steps
        for cx, cy in cells:
            # Cell centre in the body frame; the cell overlaps the rectangle when it is within half a cell of it.
            u = (cx - x) * math.cos(theta) + (cy - y) * math.sin(theta)
            v = -(cx - x) * math.sin(theta) + (cy - y) * math.cos(theta)
            assert not (abs(u) <= BODY_HALF_LENGTH_M + half_cell and abs(v) <= BODY_HALF_WIDTH_M + half_cell), (code, x, y)


def test_manifest_lists_the_planning_mask():
    manifest = json.loads((BUNDLE / "semantic_manifest.json").read_text(encoding="utf-8"))
    assert manifest["policies"]["keepout_planning"] == "keepout_planning.yaml"
    assert (BUNDLE / "keepout_planning.yaml").is_file()
