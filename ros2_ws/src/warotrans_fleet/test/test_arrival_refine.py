"""Arrival refine: AMCL updates in place while the robot settles, a missed goal is driven again (2026-11-10 lab)."""

import pytest
from fleet_fakes import Harness, cancel, navigate

from warotrans_fleet.localization_jump import LocalizationJumpDetector
from warotrans_fleet.navigator import STATUS_SUCCEEDED

CMD = "11111111-1111-1111-1111-111111111111"
GOAL = (4.25, 2.10, 1.5708)


def _harness(pose: dict, refreshes: list, **overrides):
    kwargs = dict(
        arrival_pose=lambda: pose["p"],
        arrival_settle_sec=0.9,
        localization_refresh=lambda: refreshes.append(1),
        arrival_max_refines=2,
    )
    kwargs.update(overrides)
    return Harness(**kwargs)


def _settle(h: Harness) -> None:
    for _ in range(4):
        h.tick(0.3)


def test_goal_within_limits_succeeds_without_refine_and_reports_it():
    pose = {"p": (GOAL[0] + 0.02, GOAL[1], GOAL[2] + 0.05)}
    refreshes: list = []
    h = _harness(pose, refreshes)
    h.send(navigate(CMD))
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    assert h.recorder.results == []
    _settle(h)
    result = h.recorder.results[0]
    assert result["outcome"] == "SUCCEEDED"
    assert result["result"]["refineCount"] == 0
    assert result["result"]["arrivalOutOfTolerance"] is False
    assert len(refreshes) == 3  # one on SUCCEEDED, two while settling
    assert len(h.nav.goals) == 1


def test_a_miss_found_after_the_amcl_update_drives_the_same_goal_again():
    pose = {"p": (GOAL[0] - 0.16, GOAL[1], GOAL[2] + 0.45)}  # the INBOUND leg of the lab run
    refreshes: list = []
    h = _harness(pose, refreshes)
    h.send(navigate(CMD))
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    _settle(h)
    assert h.recorder.results == []
    assert len(h.nav.goals) == 2
    again = h.nav.goals[1][0]
    assert (again.x, again.y, again.yaw) == pytest.approx(GOAL)

    pose["p"] = (GOAL[0] + 0.01, GOAL[1] - 0.01, GOAL[2] - 0.03)
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    _settle(h)
    result = h.recorder.results[0]
    assert result["outcome"] == "SUCCEEDED"
    assert result["result"]["refineCount"] == 1
    assert result["result"]["arrivalOutOfTolerance"] is False


def test_the_endpoints_position_tolerance_accepts_an_arrival_the_default_would_refine():
    """payload.positionTolerance 0.12 (EP-INBOUND / EP-OUTBOUND, 2026-11-13) replaces arrival_refine_xy_m (0.05)."""
    pose = {"p": (GOAL[0] - 0.10, GOAL[1], GOAL[2])}
    refreshes: list = []
    h = _harness(pose, refreshes)
    h.send(navigate(CMD, positionTolerance=0.12))
    assert h.nav.goals[0][0].position_tolerance == pytest.approx(0.12)
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    _settle(h)
    result = h.recorder.results[0]
    assert result["outcome"] == "SUCCEEDED"
    assert result["result"]["refineCount"] == 0
    assert result["result"]["arrivalOutOfTolerance"] is False
    assert len(h.nav.goals) == 1


def test_without_a_position_tolerance_the_same_miss_is_driven_again():
    pose = {"p": (GOAL[0] - 0.10, GOAL[1], GOAL[2])}
    refreshes: list = []
    h = _harness(pose, refreshes)
    h.send(navigate(CMD))
    assert h.nav.goals[0][0].position_tolerance is None
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    _settle(h)
    assert h.recorder.results == []
    assert len(h.nav.goals) == 2


def test_a_position_tolerance_that_is_not_positive_is_rejected():
    from warotrans_fleet.command_contract import CommandRejected, parse_navigate_target

    base = {"frameId": "map", "x": 1.0, "y": 2.0, "yaw": 0.0}
    assert parse_navigate_target({**base, "positionTolerance": 0.12}, map_frame="map").position_tolerance == 0.12
    assert parse_navigate_target(base, map_frame="map").position_tolerance is None
    for bad in (0, -0.1, "x"):
        with pytest.raises(CommandRejected):
            parse_navigate_target({**base, "positionTolerance": bad}, map_frame="map")


def test_refines_stop_at_the_limit_and_report_out_of_tolerance():
    pose = {"p": (GOAL[0], GOAL[1], GOAL[2] + 0.4)}
    h = _harness(pose, [])
    h.send(navigate(CMD))
    for _ in range(3):
        h.nav.accept()
        h.nav.finish(STATUS_SUCCEEDED)
        _settle(h)
    assert len(h.nav.goals) == 3
    result = h.recorder.results[0]
    assert result["outcome"] == "SUCCEEDED"
    assert result["result"]["refineCount"] == 2
    assert result["result"]["arrivalOutOfTolerance"] is True


def test_no_refine_while_motion_is_held():
    pose = {"p": (GOAL[0] - 0.2, GOAL[1], GOAL[2])}
    hold = {"on": False}
    h = _harness(pose, [], motion_hold_active=lambda: hold["on"])
    h.send(navigate(CMD))
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    hold["on"] = True
    _settle(h)
    assert len(h.nav.goals) == 1
    assert h.recorder.results[0]["result"]["arrivalOutOfTolerance"] is True


def test_cancel_during_a_refine_cancels_the_refine_goal():
    pose = {"p": (GOAL[0] - 0.2, GOAL[1], GOAL[2])}
    h = _harness(pose, [])
    h.send(navigate(CMD))
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    _settle(h)
    h.nav.accept()
    h.send(cancel(CMD))
    assert h.recorder.kinds().count("nav2_cancel") == 1


def test_a_failing_refresh_never_changes_the_result():
    def boom():
        raise RuntimeError("no amcl")

    pose = {"p": GOAL}
    h = _harness(pose, [], localization_refresh=boom)
    h.send(navigate(CMD))
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    _settle(h)
    assert h.recorder.results[0]["outcome"] == "SUCCEEDED"


def test_default_handler_does_not_refine():
    pose = {"p": (GOAL[0] - 0.3, GOAL[1], GOAL[2] + 0.9)}
    h = Harness(arrival_pose=lambda: pose["p"], arrival_settle_sec=0.9)
    h.send(navigate(CMD))
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    _settle(h)
    result = h.recorder.results[0]
    assert result["outcome"] == "SUCCEEDED"
    assert "refineCount" not in result["result"]


class _Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


def test_jump_detector_flags_amcl_corrections_but_not_driving_or_turning():
    clock = _Clock()
    d = LocalizationJumpDetector(clock=clock)
    assert d.observe(0.0, 0.0, 0.0) is None
    # Driving 0.10 m/s and turning 0.5 rad/s at 5 Hz is normal motion.
    for i in range(1, 6):
        clock.t = 0.2 * i
        assert d.observe(0.02 * i, 0.0, 0.1 * i) is None
    clock.t = 1.2
    jump = d.observe(0.10 - 0.16, 0.0, 0.5 + 0.45)  # the INBOUND correction
    assert jump is not None and jump.dxy == pytest.approx(0.16, abs=1e-6)
    assert d.count == 1
    clock.t = 3.0  # long gap: only resets the reference
    assert d.observe(5.0, 5.0, 2.0) is None
