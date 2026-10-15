"""Drive Assist: lidar corridor gaps, speed scaling, and its use inside TeleopController."""

import math

import pytest

from warotrans_fleet.drive_assist import Footprint, Gaps, assisted_linear, corridor_gaps, scan_to_base_points
from warotrans_fleet.teleop import TeleopController

FOOT = Footprint(half_length=0.16, half_width=0.13, margin=0.05)


def test_corridor_keeps_only_points_in_robot_wide_lane():
    gaps = corridor_gaps([(1.0, 0.0), (0.5, 0.5), (-0.66, 0.1), (0.4, -0.17)], FOOT)
    assert gaps.front == pytest.approx(0.4 - 0.16)  # (0.5, 0.5) is beside the lane
    assert gaps.rear == pytest.approx(0.66 - 0.16)


def test_points_inside_body_are_self_hits_and_ignored():
    gaps = corridor_gaps([(0.1, 0.0), (-0.1, 0.05)], FOOT)
    assert gaps == Gaps(front=None, rear=None)


@pytest.mark.parametrize(
    "linear, gaps, expected",
    [
        (0.3, Gaps(front=None, rear=None), 0.3),  # nothing ahead
        (0.3, Gaps(front=1.0, rear=0.0), 0.3),  # far ahead; rear blocked does not matter forward
        (0.3, Gaps(front=0.375, rear=None), 0.15),  # halfway between stop (0.15) and slow (0.6)
        (0.3, Gaps(front=0.1, rear=None), 0.0),  # too close: hold
        (-0.3, Gaps(front=0.0, rear=1.0), -0.3),  # reversing away from a wall is allowed
        (-0.3, Gaps(front=1.0, rear=0.1), 0.0),  # wall behind
        (0.3, None, 0.0),  # no fresh scan: cannot see, no linear motion
        (0.0, None, 0.0),
    ],
)
def test_assisted_linear(linear, gaps, expected):
    assert assisted_linear(linear, gaps, stop_gap=0.15, slow_gap=0.6) == pytest.approx(expected)


def test_scan_to_base_applies_laser_mount_rotated_left():
    # Real robot: laser yaw +pi/2. A hit at laser angle -pi/2 (laser's right) is straight ahead of the robot.
    points = scan_to_base_points([1.0], -math.pi / 2, 0.0, 0.15, 12.0, 0.01, 0.0, math.pi / 2)
    assert points[0] == pytest.approx((1.01, 0.0), abs=1e-9)


def test_scan_drops_out_of_range_and_non_finite():
    points = scan_to_base_points([0.05, math.inf, math.nan, 20.0, 2.0], 0.0, 0.1, 0.15, 12.0, 0.0, 0.0, 0.0)
    assert len(points) == 1


class Clock:
    def __init__(self):
        self.t = 0.0


def _teleop(gaps_ref: dict, clock: Clock, sent: list):
    return TeleopController(
        robot_code="R",
        publish_twist=lambda lin, ang: sent.append((round(lin, 4), ang)),
        monotonic=lambda: clock.t,
        navigation_busy=lambda: False,
        max_linear=0.3,
        max_angular=1.0,
        deadman_sec=0.5,
        gaps_provider=lambda: gaps_ref["g"],
    )


def test_teleop_assist_on_by_default_and_rechecked_on_tick():
    gaps = {"g": Gaps(front=1.0, rear=None)}
    clock, sent = Clock(), []
    teleop = _teleop(gaps, clock, sent)
    teleop.handle({"linear": 0.3, "angular": 0.0})
    assert sent == [(0.3, 0.0)]
    gaps["g"] = Gaps(front=0.1, rear=None)  # obstacle appears while driving, before the next message
    clock.t = 0.1
    teleop.tick()
    assert sent[-1] == (0.0, 0.0)
    teleop.tick()
    assert len(sent) == 2  # no duplicate publish while nothing changes


def test_teleop_assist_off_ignores_obstacles():
    gaps = {"g": Gaps(front=0.05, rear=None)}
    clock, sent = Clock(), []
    _teleop(gaps, clock, sent).handle({"linear": 0.3, "angular": 0.0, "assist": False})
    assert sent == [(0.3, 0.0)]


def test_teleop_assist_still_turns_in_place_when_blocked():
    gaps = {"g": Gaps(front=0.0, rear=0.0)}
    clock, sent = Clock(), []
    _teleop(gaps, clock, sent).handle({"linear": 0.3, "angular": 0.5})
    assert sent == [(0.0, 0.5)]
