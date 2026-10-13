"""Pure geometry helpers for fleet telemetry."""

from __future__ import annotations

import math


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert quaternion to yaw (radians) about Z — ROS REP-103."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)
