import math

import pytest

from warotrans_hardware.wheel_odom_math import SlipWindow, metres_per_tick, side_distance, slip_ratio


def test_metres_per_tick():
    assert metres_per_tick(0.0334, 2481.9) == pytest.approx(8.45555e-05, rel=1e-5)
    with pytest.raises(ValueError):
        metres_per_tick(0.0, 2481.9)


def test_side_distance():
    assert side_distance(0.10, 0.12, use_rear=True) == pytest.approx(0.11)
    assert side_distance(0.10, 0.12, use_rear=False) == pytest.approx(0.10)
    assert side_distance(0.10, None, use_rear=True) == pytest.approx(0.10)  # front-only firmware


def test_slip_ratio():
    assert slip_ratio(0.20, 0.20) == pytest.approx(0.0)
    assert slip_ratio(0.20, 0.10) == pytest.approx(0.5)
    assert slip_ratio(0.20, -0.20) == pytest.approx(2.0)   # wrong rear sign
    assert slip_ratio(0.20, 0.0) == pytest.approx(1.0)     # rear unplugged
    assert slip_ratio(0.004, 0.0) == 0.0                   # too little motion to judge


def test_slip_window():
    w = SlipWindow(0.5)
    out = None
    for _ in range(25):  # 25 x 20 ms
        out = w.add(0.02, 0.007, 0.0063, -0.007, -0.007) or out
    left, right = out
    assert left == pytest.approx(0.1)
    assert right == pytest.approx(0.0)
    assert w.add(0.02, 1.0, 1.0, 1.0, 1.0) is None  # a new window started
    assert math.isfinite(left)
