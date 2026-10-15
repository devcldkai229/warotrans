"""TeleopController: clamp, deadman stop, blocked while navigating, malformed / foreign messages dropped."""

import math
from datetime import datetime, timedelta, timezone

import pytest

from warotrans_fleet.teleop import TeleopController

ROBOT = "RBT-SIM-01"
NOW = datetime(2026, 10, 15, 12, 0, 0, tzinfo=timezone.utc)


class Harness:
    def __init__(self, *, busy: bool = False, max_age: float = 0.0) -> None:
        self.t = 0.0
        self.busy = busy
        self.twists: list[tuple[float, float]] = []
        self.teleop = TeleopController(
            robot_code=ROBOT,
            publish_twist=lambda lin, ang: self.twists.append((lin, ang)),
            monotonic=lambda: self.t,
            navigation_busy=lambda: self.busy,
            max_linear=0.3,
            max_angular=1.0,
            deadman_sec=0.5,
            max_message_age_sec=max_age,
            wall_clock=lambda: NOW,
        )


def _msg(linear: float, angular: float, **extra) -> dict:
    return {"robotCode": ROBOT, "linear": linear, "angular": angular, **extra}


def test_forward_publishes_twist_and_clamps_to_limits():
    h = Harness()
    assert h.teleop.handle(_msg(5.0, -9.0)) is None
    assert h.twists == [(0.3, -1.0)]
    assert h.teleop.is_moving


def test_release_sends_zero_and_stops():
    h = Harness()
    h.teleop.handle(_msg(0.2, 0.0))
    h.teleop.handle(_msg(0.0, 0.0))
    assert h.twists[-1] == (0.0, 0.0)
    assert not h.teleop.is_moving


def test_deadman_stops_once_when_messages_stop():
    h = Harness()
    h.teleop.handle(_msg(0.2, 0.0))
    h.t = 0.4
    h.teleop.tick()
    assert h.twists == [(0.2, 0.0)]  # still inside the deadman window
    h.t = 0.6
    h.teleop.tick()
    h.t = 1.5
    h.teleop.tick()
    assert h.twists == [(0.2, 0.0), (0.0, 0.0)]  # exactly one stop
    assert not h.teleop.is_moving


def test_held_button_keeps_robot_moving_past_deadman():
    h = Harness()
    for step in range(10):  # every 200 ms for 2 s
        h.t = step * 0.2
        h.teleop.handle(_msg(0.2, 0.0))
        h.teleop.tick()
    assert (0.0, 0.0) not in h.twists


def test_ignored_while_navigating_but_stop_still_allowed():
    h = Harness(busy=True)
    assert h.teleop.handle(_msg(0.2, 0.0)) == "NAVIGATION_ACTIVE"
    assert h.twists == []
    assert h.teleop.handle(_msg(0.0, 0.0)) is None
    assert h.twists == [(0.0, 0.0)]


@pytest.mark.parametrize(
    "body, reason",
    [
        ({"robotCode": "RBT-OTHER", "linear": 0.1, "angular": 0.0}, "WRONG_ROBOT"),
        ({"robotCode": ROBOT, "linear": "fast", "angular": 0.0}, "INVALID_PAYLOAD"),
        ({"robotCode": ROBOT, "angular": 0.0}, "INVALID_PAYLOAD"),
        ({"robotCode": ROBOT, "linear": math.nan, "angular": 0.0}, "INVALID_PAYLOAD"),
        ({"robotCode": ROBOT, "linear": 0.1, "angular": math.inf}, "INVALID_PAYLOAD"),
    ],
)
def test_malformed_or_foreign_messages_are_dropped(body, reason):
    h = Harness()
    assert h.teleop.handle(body) == reason
    assert h.twists == []


def test_stale_message_dropped_only_when_age_check_enabled():
    old = (NOW - timedelta(seconds=5)).isoformat().replace("+00:00", "Z")
    assert Harness(max_age=2.0).teleop.handle(_msg(0.1, 0.0, sentAt=old)) == "STALE"
    assert Harness(max_age=0.0).teleop.handle(_msg(0.1, 0.0, sentAt=old)) is None


def test_stop_on_shutdown_only_when_moving():
    h = Harness()
    h.teleop.stop()
    assert h.twists == []
    h.teleop.handle(_msg(0.1, 0.2))
    h.teleop.stop()
    assert h.twists[-1] == (0.0, 0.0)


def test_rejects_non_positive_limits():
    with pytest.raises(ValueError):
        TeleopController(
            robot_code=ROBOT, publish_twist=lambda *_: None, monotonic=lambda: 0.0,
            navigation_busy=lambda: False, max_linear=0.0, max_angular=1.0, deadman_sec=0.5,
        )
