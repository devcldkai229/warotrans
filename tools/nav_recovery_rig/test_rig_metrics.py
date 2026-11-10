"""The acceptance numbers of the rig: a metric that cannot fail proves nothing."""

from __future__ import annotations

import ast
import math
from pathlib import Path

import rig_metrics as m

RING = [[1.0, 1.0], [2.0, 1.0], [2.0, 1.05], [1.0, 1.05], [1.0, 1.0]]
LANE = {"code": "EDGE-LANE-X---1", "width_meters": 0.6, "tangent_samples": [{"a": [0.0, 0.0], "b": [4.0, 0.0], "tangent": 0.0}]}


def test_v004_all_pairs_are_the_ninety_directed_pairs():
    """Reads the table from the source: importing run_rig needs a ROS environment for nav_stuck_probe."""
    tree = ast.parse((Path(__file__).with_name("run_rig.py")).read_text(encoding="utf-8"))
    table = next(n.value for n in tree.body if isinstance(n, ast.Assign) and n.targets[0].id == "V004")
    assert len(table.keys) == 10  # 10 x 9 = 90, all checked at runtime by the summary


def test_only_warnings_during_the_goal_count():
    log = ("[planner_server-9] [WARN] [1000.5] [global_costmap]: KeepoutFilter: Filter mask was not received\n"
           "[planner_server-9] [WARN] [1002.5] [global_costmap]: KeepoutFilter: Filter mask was not received\n"
           "[planner_server-9] [INFO] [1003.5] [global_costmap]: something else\n")
    log = log.replace("1000.5", "1700000000.5").replace("1002.5", "1700000002.5").replace("1003.5", "1700000003.5")
    assert m.log_lines_after(log, "Filter mask was not received", 1700000001.0) == 1
    assert m.log_lines_after(log, "Filter mask was not received", 1700000003.0) == 0


def test_clearance_of_a_body_beside_and_on_the_zone():
    assert math.isclose(m.body_clearance(1.5, 0.7, 0.0, RING), 1.0 - (0.7 + 0.15), abs_tol=1e-9)
    assert m.body_clearance(1.5, 0.9, 0.0, RING) == 0.0


def test_a_long_thin_zone_shorter_than_the_body_still_counts_as_overlap():
    tip = [[1.0, 0.9], [1.05, 0.9], [1.05, 1.1], [1.0, 1.1], [1.0, 0.9]]
    assert m.body_clearance(1.0, 1.0, 0.0, tip) == 0.0


def test_heading_changes_the_clearance():
    across = m.body_clearance(1.5, 0.75, 0.0, RING)
    along = m.body_clearance(1.5, 0.75, math.pi / 2, RING)
    assert along < across


def test_keepout_clearance_is_the_minimum_over_the_run():
    truth = [{"x": 1.5, "y": 0.5, "yaw": 0.0}, {"x": 1.5, "y": 0.7, "yaw": 0.0}]
    assert math.isclose(m.keepout_clearance(truth, [{"rings": [RING]}]), 0.15, abs_tol=1e-9)
    assert m.keepout_clearance(truth, []) is None


def _drive(y: float, dx: float = 0.02) -> list[dict]:
    x0 = 0.0 if dx > 0 else 4.0
    return [{"x": x0 + i * dx, "y": y, "yaw": 0.0} for i in range(201)]


def test_lane_offset_while_following():
    assert math.isclose(m.lane_offsets(_drive(0.08), [LANE])["X"]["max"], 0.08, abs_tol=1e-3)


def test_driving_against_the_lane_is_counted():
    lane = m.lane_offsets(_drive(0.0, dx=-0.02), [LANE])["X"]
    assert lane["wrong_way"] > 0 and lane["samples"] == 0


def test_motion_with_the_gate_closed_counts_after_the_stopping_time():
    truth = [{"wall": 100.0 + i * 0.1, "x": i * 0.01, "y": 0.0} for i in range(40)]
    assert m.travel_while_gate_closed(truth, [{"e": "gate", "wall": 100.0, "ok": True}]) == 0.0
    moved = m.travel_while_gate_closed(truth, [{"e": "gate", "wall": 100.0, "ok": False}])
    assert math.isclose(moved, 0.31, abs_tol=0.015)
    # Stopped within the grace period, then reopened.
    assert m.travel_while_gate_closed(truth, [{"e": "gate", "wall": 100.0, "ok": False},
                                              {"e": "gate", "wall": 100.5, "ok": True}]) == 0.0


def test_failures_name_each_criterion():
    assert m.failures({"result": "SUCCEEDED", "goal_yaw_err": 0.1, "keepout_clearance_m": 0.1}) == []
    text = " ".join(m.failures({
        "result": "SUCCEEDED", "collisions": 1, "keepout_clearance_m": 0.0, "goal_yaw_err": 3.0,
        "travel_gate_closed_m": 0.2, "lanes": {"X": {"max": 0.2, "wrong_way": 1}}, "max_speed_mps": 0.2,
        "speed_ceiling_mps": 0.1,
    }))
    assert "from the goal" in " ".join(m.failures({"result": "SUCCEEDED", "goal_dist_m": 0.09}))
    for word in ("collisions", "keep-out", "yaw", "gate", "offset", "wrong way", "speed"):
        assert word in text
