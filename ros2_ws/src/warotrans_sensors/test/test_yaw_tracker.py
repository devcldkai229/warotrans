import math

import pytest

from warotrans_sensors.yaw_tracker import YawTracker, quaternion_ok, quaternion_to_rpy, rpy_to_quaternion


def _spin(tracker, start, end, steps=200):
    out = None
    for i in range(steps + 1):
        raw = start + (end - start) * i / steps
        out = tracker.update(math.atan2(math.sin(raw), math.cos(raw)))  # chip output is wrapped
    return out


def test_unwraps_across_pi():
    t = YawTracker()
    assert _spin(t, 0.0, 3 * math.pi) == pytest.approx(3 * math.pi, abs=1e-9)


def test_scale_applies_to_the_unwrapped_angle():
    t = YawTracker(1.0075)
    assert _spin(t, 0.0, -2 * math.pi) == pytest.approx(-2 * math.pi * 1.0075, abs=1e-9)


def test_first_sample_matches_the_previous_behaviour():
    # Before YawTracker: unwrapped started at the raw yaw and the output was raw * scale.
    t = YawTracker(0.5)
    assert t.update(1.0) == pytest.approx(0.5)


def test_chip_reset_keeps_yaw_continuous():
    t = YawTracker(1.0075)
    before = _spin(t, 0.0, 2.5)
    t.on_chip_reset()
    # The restarted fusion reports ~0 again; the output must continue from `before`.
    assert t.update(0.0) == pytest.approx(before)
    assert t.update(0.1) == pytest.approx(before + 0.1 * 1.0075)
    assert t.resets == 1


def test_reset_before_any_sample_starts_normally():
    t = YawTracker()
    t.on_chip_reset()
    assert t.update(0.3) == pytest.approx(0.3)


@pytest.mark.parametrize("q,ok", [
    ((1.0, 0.0, 0.0, 0.0), True),
    ((0.0, 0.0, 0.0, 0.0), False),   # CONFIG mode / resetting chip
    ((0.7, 0.0, 0.0, 0.0), False),
    ((1.0, 1.0, 0.0, 0.0), False),
])
def test_quaternion_ok(q, ok):
    assert quaternion_ok(*q) is ok


def test_rpy_round_trip():
    q = rpy_to_quaternion(0.02, -0.01, 2.0)
    r, p, y = quaternion_to_rpy(*q)
    assert (r, p, y) == pytest.approx((0.02, -0.01, 2.0), abs=1e-9)
