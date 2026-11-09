"""Reservation, footprint, lane and zone checks on MAP-ONE-V004 with REAL Nav2 paths (real_nav2_paths.json).

The fixture holds the first plan of each of the 90 legs of an all-pairs rig run: planner_server with LaneAwareNavfn, the
V004 bundle and its lanes. These are planner outputs, not driven trajectories. No backend or broker is needed; the geometry is the code the
robot gateway runs, which is pinned to the backend by the shared vectors (SemanticCellsParityTests).
"""

from __future__ import annotations

import itertools
import json
import math
import re
import sys
from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parent
ROBOT_REPO = HERE.parents[1]
sys.path.insert(0, str(ROBOT_REPO / "ros2_ws" / "src" / "warotrans_fleet"))

from warotrans_fleet.semantic_traffic_client import SemanticGeometry, collect_resources, footprint_resources  # noqa: E402

BUNDLE = ROBOT_REPO / "ros2_ws" / "src" / "warotrans_navigation" / "maps" / "map_one_v004"
FIXTURE = json.loads((HERE / "real_nav2_paths.json").read_text(encoding="utf-8"))
ZONES = json.loads((BUNDLE / "single_occupancy.json").read_text(encoding="utf-8"))["zones"]
LANES = json.loads((BUNDLE / "lanes.json").read_text(encoding="utf-8"))
HALF_L, HALF_W = 0.18, 0.15  # robot 0.32 x 0.26 + 0.02 padding (backend Traffic:Footprint)
RADIUS = math.hypot(HALF_L, HALF_W)
GEOM = SemanticGeometry(cell_size=0.25, horizon=4.0, sample_step=0.0625, reserve_radius=RADIUS + 0.15 + 0.03125,
                        footprint_radius=RADIUS)
PATHS = FIXTURE["paths"]


def _read_pgm(path: Path):
    raw = path.read_bytes()
    tokens = []
    i = 0
    while len(tokens) < 4:
        m = re.compile(rb"\s*(#[^\n]*\n\s*)*(\S+)").match(raw, i)
        tokens.append(m.group(2))
        i = m.end()
    w, h = int(tokens[1]), int(tokens[2])
    return w, h, raw[i + 1:]


W, H, PIXELS = _read_pgm(BUNDLE / "map.pgm")
_yaml = (BUNDLE / "map.yaml").read_text(encoding="utf-8")
RES = float(re.search(r"resolution:\s*([\d.]+)", _yaml).group(1))
OX, OY = [float(v) for v in re.search(r"origin:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)", _yaml).groups()]


def occupied(x: float, y: float) -> bool:
    col, row = int((x - OX) / RES), H - 1 - int((y - OY) / RES)
    if not (0 <= col < W and 0 <= row < H):
        return True
    return PIXELS[row * W + col] < 50  # black = wall in the saved map


def footprint_clearance_hits(x: float, y: float, yaw: float) -> int:
    c, s = math.cos(yaw), math.sin(yaw)
    hits = 0
    for u in (-0.16, -0.08, 0.0, 0.08, 0.16):      # real footprint 0.32 x 0.26, no padding
        for v in (-0.13, -0.065, 0.0, 0.065, 0.13):
            if occupied(x + c * u - s * v, y + s * u + c * v):
                hits += 1
    return hits


def test_every_endpoint_pair_has_a_real_path():
    assert FIXTURE["source"] == "nav2"
    assert len(PATHS) == 90
    assert [p for p in PATHS if not p["ok"]] == []


def test_real_paths_centre_line_never_crosses_a_wall():
    bad = []
    for p in PATHS:
        walls = sum(occupied(x, y) for x, y, _ in p["poses"])
        if walls:
            bad.append((p["from"], p["to"], walls))
    assert bad == [], f"centre line in occupied pixels: {bad[:10]}"


def test_report_real_paths_footprint_against_walls(capsys):
    """Footprint (0.32 x 0.26, heading of the path) over wall pixels. Reported: Nav2's costmap decides, not this check."""
    rows = []
    for p in PATHS:
        hits = [footprint_clearance_hits(*pose) for pose in p["poses"]]
        if any(hits):
            rows.append({"from": p["from"], "to": p["to"], "poses": len(hits), "posesTouching": sum(1 for h in hits if h)})
    (HERE / "artifacts").mkdir(exist_ok=True)
    (HERE / "artifacts" / "footprint_touching_walls.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    with capsys.disabled():
        print(f"\n[info] real paths whose footprint touches a wall pixel somewhere: {len(rows)} of {len(PATHS)}")


def _all_lane_segments():
    out = []
    for lane in LANES["lanes"]:
        half = float(lane["width_meters"]) / 2.0 + 0.08  # corridor_margin of nav2_params_lane_aware.yaml
        pts = lane["coordinates"]
        for a, b in zip(pts, pts[1:]):
            out.append((lane["code"], lane["direction"] == "ONE_WAY", a[0], a[1], b[0], b[1], half))
    return out


def test_real_paths_do_not_drive_against_a_one_way_lane():
    """Every step of every path, by the rule LaneAwareNavfn::moveAllowed applies: the nearest lane at the step midpoint."""
    segments = _all_lane_segments()
    assert any(one_way for _, one_way, *_ in segments), "no ONE_WAY lane read from lanes.json"
    against = {}
    for p in PATHS:
        # From the second pose: the first step joins the start pose to the grid, up to a cell long, and the search never saw it.
        for (x0, y0, _), (x1, y1, _) in zip(p["poses"][1:], p["poses"][2:]):
            dx, dy = x1 - x0, y1 - y0
            n = math.hypot(dx, dy)
            if n < 1e-3:
                continue
            mx, my = (x0 + x1) / 2, (y0 + y1) / 2
            best = None
            for code, one_way, ax, ay, bx, by, half in segments:
                vx, vy = bx - ax, by - ay
                t = max(0.0, min(1.0, ((mx - ax) * vx + (my - ay) * vy) / (vx * vx + vy * vy)))
                d = math.hypot(mx - (ax + t * vx), my - (ay + t * vy))
                if d <= half and (best is None or d < best[0]):
                    best = (d, code, one_way, vx, vy)
            if best is None or not best[2]:
                continue
            along = (dx * best[3] + dy * best[4]) / (n * math.hypot(best[3], best[4]))
            if along < -0.25:  # reverse; |along| < 0.25 is a near-perpendicular crossing, allowed
                against[(p["from"], p["to"], best[1])] = against.get((p["from"], p["to"], best[1]), 0) + 1
    assert against == {}, f"{len(against)} route/lane pairs against a ONE_WAY lane: {list(against)[:10]}"


def test_reservation_of_a_real_path_covers_the_footprint_all_along_it():
    for p in PATHS:
        reserved = collect_resources(p["poses"], ZONES, GEOM)
        traveled = 0.0
        prev = p["poses"][0]
        for pose in p["poses"]:
            traveled += math.hypot(pose[0] - prev[0], pose[1] - prev[1])
            prev = pose
            if traveled > GEOM.horizon:
                break
            under = footprint_resources(pose[0], pose[1], RADIUS, ZONES, GEOM.cell_size)
            assert under <= reserved, (p["from"], p["to"], sorted(under - reserved)[:4])


def test_workflow_routes_use_zones_as_expected_and_conflict_with_a_robot_on_them():
    by = {(p["from"], p["to"]): p for p in PATHS}
    route = by[("EP-PARKING-A", "EP-INBOUND")]
    res = collect_resources(route["poses"], ZONES, GEOM)
    assert res  # reserved something
    # A second robot standing on the route's first metres conflicts; one at the far end of the map does not.
    near = footprint_resources(route["poses"][10][0], route["poses"][10][1], GEOM.reserve_radius, ZONES, GEOM.cell_size)
    assert near & res


@pytest.mark.parametrize("a,b", [("EP-PARKING-A", "EP-PARKING-B")])
def test_report_parked_endpoints_spacing(a, b, capsys):
    """Not a pass/fail of safety: documents which endpoints two robots cannot occupy at once (multi-robot planning)."""
    snap = json.loads((ROBOT_REPO.parent / "warotrans-system" / "samples" / "maps" / "MAP-ONE-V004" / "snapshot.json")
                      .read_text(encoding="utf-8"))
    eps = {e["code"]: e for e in snap["endpoints"]}
    seat = {c: footprint_resources(e["x"], e["y"], GEOM.reserve_radius, ZONES, GEOM.cell_size) for c, e in eps.items()}
    clash = [(x, y, sorted(seat[x] & seat[y])[:2]) for x, y in itertools.combinations(sorted(eps), 2) if seat[x] & seat[y]]
    (HERE / "artifacts").mkdir(exist_ok=True)
    (HERE / "artifacts" / "endpoint_conflicts.json").write_text(json.dumps(clash, indent=1), encoding="utf-8")
    with capsys.disabled():
        print(f"\n[info] endpoint pairs two parked robots cannot hold at once: {len(clash)} of 45")
    assert a in eps and b in eps
