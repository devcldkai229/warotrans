"""Two robots side by side: real Host + real Mosquitto + robot-gateway logic, with the settings of a two-robot run.

    Traffic:SemanticPrefixGrant=true, SemanticLaneHalfWidthMeters=0.26, SemanticCellSizeMeters=0.1

Motion-free (robots are simulated), test database and test topic tree only. Run alone, it starts its own Host:

    dotnet build backend/src/WaroTrans.Host        # in warotrans-system
    python -m pytest tools/traffic_e2e/test_dual_lane_harness.py -v -p no:cacheprovider

The lane scenarios use the x of BTT-C (1.19) and TTB-C (0.52) of MAP-ONE-V004, 0.67 m apart.
"""

from __future__ import annotations

import math
import subprocess
import sys
import time
from pathlib import Path

import pytest

from harness import Host, wait_for
from test_traffic_e2e import (  # noqa: F401 - api and warehouse_id are fixtures
    ARTIFACTS,
    HOST,
    ROBOTS,
    api,
    dist,
    holders,
    navigate,
    result_of,
    robot,
    view,
    warehouse_id,
    watch_exclusive,
)

BODY_LENGTH_M = 0.36  # padded footprint: two robots nose to tail are at least this far apart, centre to centre
ZONE = "zone:ZONE-SINGLE-CENTER"
ZONE_SOUTH_EDGE_Y = 1.04


@pytest.fixture(scope="session")
def host():
    ARTIFACTS.mkdir(exist_ok=True)
    seeder = str(Path(__file__).with_name("seed_test_db.py"))
    clean = subprocess.run([sys.executable, seeder, "--cleanup-e2e-robots"], capture_output=True, text=True)
    assert clean.returncode == 0, clean.stderr
    h = Host(ARTIFACTS, name="dual", extra_env={
        "Traffic__SemanticPrefixGrant": "true",
        "Traffic__SemanticLaneHalfWidthMeters": "0.26",
        "Traffic__SemanticCellSizeMeters": "0.1",
    })
    h.start()
    HOST.append(h)
    seed = subprocess.run([sys.executable, seeder], capture_output=True, text=True)
    assert seed.returncode == 0, seed.stderr
    yield h
    for sim in ROBOTS:
        sim.stop()
    h.stop()


def _pair(api, warehouse_id, tag, a_xy, b_xy, a_speed=0.1, b_speed=0.1, yaw_a=0.0, yaw_b=0.0):
    a, ra = robot(api, warehouse_id, f"{tag}A", *a_xy, speed=a_speed)
    b, rb = robot(api, warehouse_id, f"{tag}B", *b_xy, speed=b_speed)
    a.yaw, b.yaw = yaw_a, yaw_b
    return a, ra, b, rb


def test_opposite_lanes_run_side_by_side(api, warehouse_id):
    a, ra, b, rb = _pair(api, warehouse_id, "P", (1.19, 0.05), (0.52, 0.7), yaw_a=math.pi / 2, yaw_b=-math.pi / 2)
    ca, cb = navigate(api, ra, 1.19, 0.6), navigate(api, rb, 0.52, 0.15)
    both_authorized, blocked, closest = [], [], [99.0]

    def done():
        va, vb = view(api, ra), view(api, rb)
        if va.get("authority") and vb.get("authority"):
            both_authorized.append((a.y, b.y))
        blocked.extend(x for x in (va.get("blockingOwner"), vb.get("blockingOwner")) if x)
        closest[0] = min(closest[0], dist(a, b))
        return result_of(a, ca) and result_of(b, cb)

    watch_exclusive(api, 40, until=done)
    assert result_of(a, ca)["outcome"] == "SUCCEEDED" and result_of(b, cb)["outcome"] == "SUCCEEDED"
    assert both_authorized, "never authorized at the same time"
    assert not blocked, f"one waited for the other: {blocked}"
    assert closest[0] >= 0.6
    assert not a.moved_while_gate_closed() and not b.moved_while_gate_closed()
    # Off the map again: a parked robot rightly blocks every route through it.
    pa, pb = navigate(api, ra, 1.19, -4.0), navigate(api, rb, 0.52, -4.0)
    wait_for(lambda: result_of(a, pa) and result_of(b, pb), 90, what="P robots parked")


def test_second_robot_waits_just_before_the_single_occupancy_zone(api, warehouse_id):
    a, ra, b, rb = _pair(api, warehouse_id, "S", (0.85, 1.9), (1.19, -1.2), a_speed=0.1, b_speed=0.4, yaw_a=math.pi / 2,
                         yaw_b=math.pi / 2)
    ca = navigate(api, ra, 0.85, 3.4, timeout=120)
    wait_for(lambda: ZONE in holders(api.snapshot()) and ra in holders(api.snapshot())[ZONE], 15, what="A holds the zone")
    start_y = b.y
    cb = navigate(api, rb, 1.19, 2.0, timeout=120)

    # While A is in the zone, B drives on, well past where it started, and stops short of the zone.
    wait_for(lambda: b.y > start_y + 0.8, 15, what="B driving towards the zone")
    deepest = [b.y]

    def a_left():
        deepest[0] = max(deepest[0], b.y) if ra in holders(api.snapshot()).get(ZONE, []) else deepest[0]
        return rb in holders(api.snapshot()).get(ZONE, []) or result_of(a, ca)

    snaps = watch_exclusive(api, 60, until=a_left)
    assert deepest[0] + 0.18 < ZONE_SOUTH_EDGE_Y, f"B went to y={deepest[0]} while A held the zone"
    assert ra in [v for s in snaps for v in holders(s).get(ZONE, [])]
    wait_for(lambda: result_of(a, ca) and result_of(b, cb), 120, what="both done")
    assert result_of(a, ca)["outcome"] == "SUCCEEDED" and result_of(b, cb)["outcome"] == "SUCCEEDED"
    assert not b.moved_while_gate_closed()
    pa, pb = navigate(api, ra, 2.2, -4.0, timeout=120), navigate(api, rb, 3.5, -4.0, timeout=120)
    wait_for(lambda: result_of(a, pa) and result_of(b, pb), 120, what="S robots parked")


def test_following_on_a_lane_keeps_its_distance(api, warehouse_id):
    a, ra, b, rb = _pair(api, warehouse_id, "F", (12.0, 2.0), (12.0, 0.0), a_speed=0.1, b_speed=0.4,
                         yaw_a=math.pi / 2, yaw_b=math.pi / 2)
    ca = navigate(api, ra, 12.0, 4.0, timeout=90)
    wait_for(lambda: view(api, ra).get("authority"), 10, what="leader granted")
    cb = navigate(api, rb, 12.0, 3.0, timeout=90)
    closest = [99.0]
    moved = [0.0]

    def done():
        closest[0] = min(closest[0], dist(a, b))
        moved[0] = b.y
        return result_of(a, ca) and result_of(b, cb)

    watch_exclusive(api, 90, until=done)
    assert result_of(a, ca)["outcome"] == "SUCCEEDED" and result_of(b, cb)["outcome"] == "SUCCEEDED"
    assert moved[0] > 2.5, "the follower got nowhere"
    assert closest[0] >= BODY_LENGTH_M


def test_crossing_paths_one_waits_before_the_lane_of_the_other(api, warehouse_id):
    a, ra, b, rb = _pair(api, warehouse_id, "C", (20.0, 2.0), (22.0, 4.5), a_speed=0.2, b_speed=0.4,
                         yaw_b=-math.pi / 2)
    ca = navigate(api, ra, 25.0, 2.0, timeout=90)
    wait_for(lambda: view(api, ra).get("authority"), 10, what="A granted")
    cb = navigate(api, rb, 22.0, 0.0, timeout=90)
    closest = [99.0]

    def done():
        closest[0] = min(closest[0], dist(a, b))
        return result_of(a, ca) and result_of(b, cb)

    watch_exclusive(api, 90, until=done)
    assert result_of(a, ca)["outcome"] == "SUCCEEDED" and result_of(b, cb)["outcome"] == "SUCCEEDED"
    assert closest[0] >= BODY_LENGTH_M


def test_head_on_on_one_lane_never_drives_into_each_other(api, warehouse_id):
    a, ra, b, rb = _pair(api, warehouse_id, "H", (30.0, 2.0), (36.0, 2.0), a_speed=0.3, b_speed=0.3, yaw_b=math.pi)
    ca = navigate(api, ra, 36.0, 2.0, timeout=25)
    wait_for(lambda: view(api, ra).get("authority"), 10, what="A granted")
    cb = navigate(api, rb, 30.0, 2.0, timeout=25)
    closest = [99.0]

    def done():
        closest[0] = min(closest[0], dist(a, b))
        return result_of(a, ca) and result_of(b, cb)

    watch_exclusive(api, 60, until=done)
    # There is no way past on one lane: what matters is no contact and no robot stuck forever without a result.
    assert result_of(a, ca) and result_of(b, cb)
    assert closest[0] >= BODY_LENGTH_M
    time.sleep(0.1)
