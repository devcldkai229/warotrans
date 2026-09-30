"""Skid-steer mixing and PWM command calculation mirroring ESP32 firmware.

Open-loop map: duty ∝ |wheel_speed| / commissioning_full_scale_linear, clamp 40%.
ESP32 may optionally run a closed-loop wheel-velocity scaffold
(USE_WHEEL_VELOCITY_CLOSED_LOOP in warotrans_low_level.ino); that loop is
DEFAULT OFF and is NOT implemented on the Pi. This module stays a UI/debug
mirror of the mix equation only:

  left  = linear - angular * wheel_separation / 2
  right = linear + angular * wheel_separation / 2
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class MixResult:
    """Computed wheel PWM command values (not measured actuator feedback)."""

    left_wheel_speed: float
    right_wheel_speed: float
    left_pwm_raw: int
    right_pwm_raw: int
    left_pwm_percent: float
    right_pwm_percent: float


def _duty_from_wheel_speed(
    wheel_speed: float,
    *,
    full_scale_linear: float,
    pwm_raw_max: int,
    pwm_limit_duty: int,
) -> int:
    if not math.isfinite(wheel_speed) or full_scale_linear <= 0.0:
        return 0

    ratio = abs(wheel_speed) / full_scale_linear
    if not math.isfinite(ratio) or ratio <= 0.0:
        return 0
    if ratio > 1.0:
        ratio = 1.0

    duty = int(ratio * pwm_raw_max + 0.5)
    if duty >= pwm_limit_duty:
        return pwm_limit_duty
    return int(duty)


def mix_to_pwm(
    linear: float,
    angular: float,
    *,
    wheel_separation: float,
    full_scale_linear: float,
    pwm_raw_max: int = 255,
    pwm_limit_percent: float = 40.0,
) -> MixResult | None:
    """Mirror firmware mixing and PWM clamping.

    Returns ``None`` when inputs are non-finite.
    """

    if not math.isfinite(linear) or not math.isfinite(angular):
        return None
    if wheel_separation <= 0.0 or full_scale_linear <= 0.0:
        return None
    if pwm_raw_max <= 0 or pwm_limit_percent <= 0.0:
        return None

    linear = max(-full_scale_linear, min(full_scale_linear, linear))
    full_scale_angular = 2.0 * full_scale_linear / wheel_separation
    angular = max(-full_scale_angular, min(full_scale_angular, angular))

    left_speed = linear - angular * wheel_separation / 2.0
    right_speed = linear + angular * wheel_separation / 2.0

    peak = max(abs(left_speed), abs(right_speed))
    if peak > full_scale_linear:
        scale = full_scale_linear / peak
        left_speed *= scale
        right_speed *= scale

    pwm_limit_duty = int(round(pwm_raw_max * pwm_limit_percent / 100.0))
    left_raw = _duty_from_wheel_speed(
        left_speed,
        full_scale_linear=full_scale_linear,
        pwm_raw_max=pwm_raw_max,
        pwm_limit_duty=pwm_limit_duty,
    )
    right_raw = _duty_from_wheel_speed(
        right_speed,
        full_scale_linear=full_scale_linear,
        pwm_raw_max=pwm_raw_max,
        pwm_limit_duty=pwm_limit_duty,
    )

    def signed_percent(wheel_speed: float, pwm_raw: int) -> float:
        if pwm_raw == 0:
            return 0.0
        magnitude = pwm_raw / pwm_raw_max * 100.0
        return magnitude if wheel_speed >= 0.0 else -magnitude

    return MixResult(
        left_wheel_speed=left_speed,
        right_wheel_speed=right_speed,
        left_pwm_raw=left_raw,
        right_pwm_raw=right_raw,
        left_pwm_percent=signed_percent(left_speed, left_raw),
        right_pwm_percent=signed_percent(right_speed, right_raw),
    )
