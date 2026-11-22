"""BNO055 yaw post-processing, kept free of ROS so it can be unit tested.

YawTracker turns the chip's wrapped fusion yaw into a continuous, scaled yaw that survives a chip reset:
after a reset (brown-out, I2C glitch) the BNO055 restarts in CONFIG mode and, once reconfigured, its IMUPLUS
heading starts again from 0. With robot_localization imu0_relative=true that would be a heading jump, so the
first sample after a reset is joined onto the last published yaw instead.
"""

from __future__ import annotations

import math
from typing import Optional


def quaternion_ok(w: float, x: float, y: float, z: float) -> bool:
    """A fusion quaternion has norm 1; all zeros (CONFIG mode / resetting chip) or garbage does not."""
    n = math.sqrt(w * w + x * x + y * y + z * z)
    return 0.9 < n < 1.1


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class YawTracker:
    """out = offset + scale * unwrapped(raw yaw)."""

    def __init__(self, scale: float = 1.0) -> None:
        self.scale = float(scale)
        self._last_raw: Optional[float] = None
        self._unwrapped = 0.0
        self._offset = 0.0
        self._last_out: Optional[float] = None
        self._rejoin = False
        self.resets = 0

    @property
    def last_out(self) -> Optional[float]:
        return self._last_out

    def on_chip_reset(self) -> None:
        """The next raw yaw comes from a restarted fusion: join it onto the last output."""
        self._rejoin = True
        self._last_raw = None
        self.resets += 1

    def update(self, raw_yaw: float) -> float:
        if self._last_raw is None:
            self._unwrapped = raw_yaw
            if self._rejoin and self._last_out is not None:
                self._offset = self._last_out - self.scale * raw_yaw
            self._rejoin = False
        else:
            self._unwrapped += wrap(raw_yaw - self._last_raw)
        self._last_raw = raw_yaw
        out = self._offset + self.scale * self._unwrapped
        self._last_out = out
        return out


def quaternion_to_rpy(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)
