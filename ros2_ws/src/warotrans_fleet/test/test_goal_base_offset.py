"""Goal offset: Nav2 drives the turning centre, endpoints are for the chassis centre (2026-11-11 lab)."""

import math

import pytest
from fleet_fakes import Harness, navigate

from warotrans_fleet.navigator import STATUS_SUCCEEDED

CMD = "22222222-2222-2222-2222-222222222222"
GOAL = (4.25, 2.10, 1.5708)  # what navigate() sends (see fleet_fakes)
OFFSET = 0.12


def _harness(pose: dict, **overrides):
    kwargs = dict(
        arrival_pose=lambda: pose["p"],
        arrival_settle_sec=0.9,
        arrival_max_refines=2,
        goal_base_offset=lambda: OFFSET,
    )
    kwargs.update(overrides)
    return Harness(**kwargs)


def _settle(h: Harness) -> None:
    for _ in range(4):
        h.tick(0.3)


def _shifted(goal=GOAL, offset=OFFSET):
    x, y, yaw = goal
    return x - offset * math.cos(yaw), y - offset * math.sin(yaw), yaw


def test_the_nav2_goal_is_the_endpoint_moved_back_along_its_heading():
    h = _harness({"p": None})
    h.send(navigate(CMD))
    sent = h.nav.goals[0][0]
    assert (sent.x, sent.y, sent.yaw) == pytest.approx(_shifted())


def test_arrival_error_is_measured_on_the_chassis_centre_against_the_endpoint():
    # The chassis centre stands on the endpoint: no error although the Nav2 goal was 0.12 m behind it.
    pose = {"p": GOAL}
    h = _harness(pose)
    h.send(navigate(CMD))
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    _settle(h)
    result = h.recorder.results[0]["result"]
    assert result["arrivalErrorXyM"] == pytest.approx(0.0, abs=1e-4)
    assert result["refineCount"] == 0


def test_a_refine_re_sends_the_shifted_goal():
    pose = {"p": (GOAL[0], GOAL[1], GOAL[2] + 0.4)}
    h = _harness(pose)
    h.send(navigate(CMD))
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    _settle(h)
    assert len(h.nav.goals) == 2
    again = h.nav.goals[1][0]
    assert (again.x, again.y, again.yaw) == pytest.approx(_shifted())


def test_no_offset_sends_the_endpoint_unchanged():
    h = _harness({"p": None}, goal_base_offset=lambda: 0.0)
    h.send(navigate(CMD))
    sent = h.nav.goals[0][0]
    assert (sent.x, sent.y, sent.yaw) == pytest.approx(GOAL)


def test_a_failing_offset_source_falls_back_to_no_offset():
    def broken() -> float:
        raise RuntimeError("tf not ready")

    h = _harness({"p": None}, goal_base_offset=broken)
    h.send(navigate(CMD))
    sent = h.nav.goals[0][0]
    assert (sent.x, sent.y) == pytest.approx(GOAL[:2])
