"""Pure helpers for four-wheel encoder odometry on the 2-channel skid-steer (no ROS imports).

The MDD10A drives the front and rear wheel of a side from one channel, so each side has one commanded speed but
(since 2026-11-20) two encoders. With ``use_rear_encoders`` the side distance is the mean of both wheels; the
front/rear difference of a side is reported as slip.
"""

from __future__ import annotations

import math
from typing import Optional


def metres_per_tick(radius_m: float, ticks_per_rev: float) -> float:
    if not (math.isfinite(radius_m) and math.isfinite(ticks_per_rev)) or radius_m <= 0.0 or ticks_per_rev <= 0.0:
        raise ValueError("radius and ticks_per_rev must be finite and > 0")
    return 2.0 * math.pi * radius_m / ticks_per_rev


def side_distance(front_m: float, rear_m: Optional[float], use_rear: bool) -> float:
    """Distance of one side: the mean of front and rear wheel when the rear is used and available."""
    if use_rear and rear_m is not None:
        return 0.5 * (front_m + rear_m)
    return front_m


def slip_ratio(front_m: float, rear_m: float, min_distance_m: float = 0.01) -> float:
    """(front - rear) / the larger magnitude, in [-2, 2]; 0 while both moved less than ``min_distance_m``.

    ~0: both wheels of the side turn alike. Large values: one wheel slips, is lifted, or its encoder is
    unplugged / has the wrong sign (opposite directions give |ratio| > 1)."""
    larger = max(abs(front_m), abs(rear_m))
    if larger < min_distance_m:
        return 0.0
    return (front_m - rear_m) / larger


def slip_variance_scale(left_ratio: float, right_ratio: float, threshold: float, scale: float) -> float:
    """Factor for the /odom twist variance: ``scale`` while either side slips more than ``threshold``, else 1.

    A slipping side's encoders no longer measure ground speed, so the EKF should lean on the IMU and its model."""
    return scale if max(abs(left_ratio), abs(right_ratio)) > threshold else 1.0


class SlipWindow:
    """Accumulates front and rear distances of both sides and yields slip ratios once per window."""

    def __init__(self, window_s: float = 0.5) -> None:
        if window_s <= 0.0:
            raise ValueError("window_s must be > 0")
        self._window_s = window_s
        self._elapsed = 0.0
        self._sums = [0.0, 0.0, 0.0, 0.0]  # front_left, rear_left, front_right, rear_right

    def add(self, dt_s: float, front_left: float, rear_left: float, front_right: float,
            rear_right: float) -> Optional[tuple[float, float]]:
        """Add one sample; returns (left_ratio, right_ratio) when a window completes, else None."""
        for i, d in enumerate((front_left, rear_left, front_right, rear_right)):
            self._sums[i] += d
        self._elapsed += max(0.0, dt_s)
        if self._elapsed < self._window_s:
            return None
        fl, rl, fr, rr = self._sums
        self._sums = [0.0, 0.0, 0.0, 0.0]
        self._elapsed = 0.0
        return slip_ratio(fl, rl), slip_ratio(fr, rr)
