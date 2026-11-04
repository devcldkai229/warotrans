"""Unit tests for Stage-2 Semantic Map Adapter (no Gazebo)."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import pytest

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
import semantic_map_adapter as sma  # noqa: E402


def test_junction_omitted_from_keepout_and_speed():
    zones = [
        {
            "code": "Z-J",
            "zone_type": "JUNCTION",
            "is_active": True,
            "geometry": {
                "type": "Polygon",
                "coordinates": [[[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0], [0.0, 0.0]]],
            },
        },
        {
            "code": "Z-K",
            "zone_type": "KEEP_OUT",
            "is_active": True,
            "geometry": {
                "type": "Polygon",
                "coordinates": [[[2.0, 2.0], [3.0, 2.0], [3.0, 3.0], [2.0, 3.0], [2.0, 2.0]]],
            },
        },
        {
            "code": "Z-S",
            "zone_type": "SPEED",
            "max_speed": 0.2,
            "is_active": True,
            "geometry": {
                "type": "Polygon",
                "coordinates": [[[4.0, 4.0], [5.0, 4.0], [5.0, 5.0], [4.0, 5.0], [4.0, 4.0]]],
            },
        },
    ]
    w, h, res = 100, 100, 0.1
    ox, oy = 0.0, 0.0
    keep = sma.rasterize_keepout(zones, width=w, height=h, origin_x=ox, origin_y=oy, resolution=res)
    speed = sma.rasterize_speed(zones, width=w, height=h, origin_x=ox, origin_y=oy, resolution=res)
    # JUNCTION cell ~ (0.5, 0.5) must stay free
    c, r = sma.world_to_pixel(0.5, 0.5, origin_x=ox, origin_y=oy, resolution=res, height=h)
    assert keep[r * w + c] == sma.FREE
    assert speed[r * w + c] == sma.SPEED_UNSET
    # KEEP_OUT marked (trinary occupied = 0)
    c, r = sma.world_to_pixel(2.5, 2.5, origin_x=ox, origin_y=oy, resolution=res, height=h)
    assert keep[r * w + c] == sma.KEEPOUT_LETHAL
    assert sma.KEEPOUT_LETHAL == 0
    assert sma.FREE == 254
    # SPEED encoded ~ 0.2 / 0.01 = 20
    c, r = sma.world_to_pixel(4.5, 4.5, origin_x=ox, origin_y=oy, resolution=res, height=h)
    assert speed[r * w + c] == 20


def test_speed_overlap_most_restrictive():
    zones = [
        {
            "code": "A",
            "zone_type": "SPEED",
            "max_speed": 0.5,
            "is_active": True,
            "geometry": {
                "type": "Polygon",
                "coordinates": [[[0, 0], [2, 0], [2, 2], [0, 2], [0, 0]]],
            },
        },
        {
            "code": "B",
            "zone_type": "SPEED",
            "max_speed": 0.1,
            "is_active": True,
            "geometry": {
                "type": "Polygon",
                "coordinates": [[[0.5, 0.5], [1.5, 0.5], [1.5, 1.5], [0.5, 1.5], [0.5, 0.5]]],
            },
        },
    ]
    w, h, res = 40, 40, 0.1
    speed = sma.rasterize_speed(zones, width=w, height=h, origin_x=0.0, origin_y=0.0, resolution=res)
    c, r = sma.world_to_pixel(1.0, 1.0, origin_x=0.0, origin_y=0.0, resolution=res, height=h)
    assert speed[r * w + c] == 10  # 0.1 m/s


def test_one_way_move_allowed():
    lanes = [
        {
            "code": "E1",
            "direction": "ONE_WAY",
            "width_meters": 1.0,
            "coordinates": [[0.0, 0.0], [2.0, 0.0]],
            "tangents_rad": [0.0, 0.0],
        }
    ]
    assert sma.move_allowed(0.1, 0.0, 0.5, 0.0, lanes) is True
    assert sma.move_allowed(0.5, 0.0, 0.1, 0.0, lanes) is False  # pure reverse
    # Near-perpendicular cross (north) inside eastbound corridor
    assert sma.move_allowed(0.5, -0.1, 0.5, 0.3, lanes) is True
    # Reverse-diagonal (NW 135°) must stay illegal — prevents zigzag reverse
    assert sma.move_allowed(0.5, 0.0, 0.3, 0.2, lanes) is False
    # Outside corridor: free
    assert sma.move_allowed(0.5, 5.0, 0.1, 5.0, lanes) is True


def test_validate_path_no_stop_goal():
    lanes = []
    no_stop = [[[[-0.2, -0.2], [0.2, -0.2], [0.2, 0.2], [-0.2, 0.2], [-0.2, -0.2]]]]
    poses = [(1.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    ok, reason = sma.validate_path_poses(
        poses, lanes=lanes, keepout_rings=[], no_stop_rings=no_stop, check_no_stop_goal=True
    )
    assert ok is False
    assert reason == "NO_STOP_GOAL"


def test_fingerprint_stable(tmp_path: Path):
    snap = {
        "zones": [],
        "edges": [
            {
                "code": "L1",
                "direction": "ONE_WAY",
                "width_meters": 0.8,
                "is_active": True,
                "geometry": {"type": "LineString", "coordinates": [[0, 0], [1, 0]]},
            }
        ],
        "endpoints": [{"code": "EP-A", "x": 0.0, "y": 0.0, "yaw": 0.0}],
    }
    a = sma.export_from_snapshot(
        snap,
        width=10,
        height=10,
        origin_x=0.0,
        origin_y=0.0,
        resolution=0.05,
        out_dir=tmp_path / "a",
        occupancy_sha256="abc",
    )
    b = sma.export_from_snapshot(
        snap,
        width=10,
        height=10,
        origin_x=0.0,
        origin_y=0.0,
        resolution=0.05,
        out_dir=tmp_path / "b",
        occupancy_sha256="abc",
    )
    ma = json.loads(a["manifest"].read_text(encoding="utf-8"))
    mb = json.loads(b["manifest"].read_text(encoding="utf-8"))
    assert ma["semanticMapFingerprint"] == mb["semanticMapFingerprint"]
    assert (tmp_path / "a" / "lanes.json").is_file()
    assert (tmp_path / "a" / "keepout.pgm").is_file()
    assert (tmp_path / "a" / "speed.pgm").is_file()
    assert (tmp_path / "a" / "no_stop.json").is_file()
    assert (tmp_path / "a" / "single_occupancy.json").is_file()
    lanes = json.loads((tmp_path / "a" / "lanes.json").read_text(encoding="utf-8"))
    assert lanes["lanes"][0]["tangent_samples"]


def test_lane_membership_corridor():
    lanes = [
        {
            "code": "L",
            "direction": "BIDIRECTIONAL",
            "width_meters": 0.5,
            "coordinates": [[0.0, 0.0], [3.0, 0.0]],
            "tangents_rad": [0.0, 0.0],
        }
    ]
    assert sma.lane_membership(1.0, 0.1, lanes) is not None
    assert sma.lane_membership(1.0, 1.0, lanes) is None


def test_lane_aware_astar_rejects_one_way_reverse_only_path():
    # Corridor along +x ONE_WAY; lethal walls force travel inside the corridor only.
    lanes = [
        {
            "code": "OW",
            "direction": "ONE_WAY",
            "width_meters": 0.8,
            "coordinates": [[0.0, 0.0], [5.0, 0.0]],
            "tangents_rad": [0.0, 0.0],
        }
    ]
    w, h, res = 50, 20, 0.1
    ox, oy = 0.0, -1.0
    lethal = set()
    for c in range(w):
        for r in range(h):
            y = oy + (r + 0.5) * res
            if abs(y) > 0.25:
                lethal.add((c, r))
    path = sma.plan_lane_aware_grid(
        width=w,
        height=h,
        origin_x=ox,
        origin_y=oy,
        resolution=res,
        lethal=lethal,
        start_xy=(4.0, 0.0),
        goal_xy=(0.5, 0.0),
        lanes=lanes,
    )
    assert path == []
    fwd = sma.plan_lane_aware_grid(
        width=w,
        height=h,
        origin_x=ox,
        origin_y=oy,
        resolution=res,
        lethal=lethal,
        start_xy=(0.5, 0.0),
        goal_xy=(4.0, 0.0),
        lanes=lanes,
    )
    assert len(fwd) >= 2
