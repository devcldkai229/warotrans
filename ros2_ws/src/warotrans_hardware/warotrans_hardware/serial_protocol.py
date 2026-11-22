"""Pure helpers for the ESP32 text telemetry protocol."""

from __future__ import annotations

import math
from typing import NamedTuple, Optional

INT64_MIN = -(2**63)
INT64_MAX = 2**63 - 1
UINT32_MAX = 2**32 - 1


class EncoderFrame(NamedTuple):
    """One ``O`` telemetry line. ``rear_left`` / ``rear_right`` are None for firmware without rear encoders."""

    left: int
    right: int
    mcu_millis: int
    rear_left: Optional[int] = None
    rear_right: Optional[int] = None


def parse_encoder_frame(line: bytes | str) -> Optional[EncoderFrame]:
    """Parse ``O <left> <right> <millis>`` (front encoders only) or, since 2026-11-20,
    ``O <front_left> <front_right> <rear_left> <rear_right> <millis>``. ``None`` if malformed / out of range."""

    if isinstance(line, bytes):
        try:
            text = line.decode("ascii").strip()
        except UnicodeDecodeError:
            return None
    else:
        text = line.strip()

    fields = text.split()
    if len(fields) not in (4, 6) or fields[0] != "O":
        return None

    try:
        values = [int(f, 10) for f in fields[1:]]
    except ValueError:
        return None

    *ticks, mcu_millis = values
    if not all(INT64_MIN <= t <= INT64_MAX for t in ticks):
        return None
    if not 0 <= mcu_millis <= UINT32_MAX:
        return None

    if len(ticks) == 2:
        return EncoderFrame(ticks[0], ticks[1], mcu_millis)
    return EncoderFrame(ticks[0], ticks[1], mcu_millis, ticks[2], ticks[3])


def parse_encoder_line(
    line: bytes | str,
) -> Optional[tuple[int, int, int]]:
    """Parse one ESP32 telemetry line.

    Returns ``(left_ticks, right_ticks, mcu_millis)`` (front wheels) for a valid line and
    ``None`` for malformed or out-of-range input. ``parse_encoder_frame`` also returns the rear wheels.
    """

    frame = parse_encoder_frame(line)
    if frame is None:
        return None
    return frame.left, frame.right, frame.mcu_millis


def format_velocity_command(linear_mps: float, angular_rps: float) -> str:
    """Format a Pi -> ESP32 velocity command line."""

    if not math.isfinite(linear_mps) or not math.isfinite(angular_rps):
        raise ValueError("linear and angular must be finite")
    return f"V {linear_mps:.6f} {angular_rps:.6f}\n"


# Runtime drive tuning understood by warotrans_low_level.ino ("C <name> <value>").
# mpt_l / mpt_r / mpt_rl / mpt_rr: wheel metres per encoder tick (PI speed measurement); sep: effective wheel
# separation (mixer); rinv_l / rinv_r: 1 = flip the rear encoder sign of that side; rear_fb: 1 = the PI measures
# each side as the mean of its front and rear wheel (firmware falls back with "K FAULT rear_fb" on a mismatch).
CONFIG_NAMES = ("min_duty", "closed_loop", "kp", "ki", "mpt_l", "mpt_r", "sep",
                "mpt_rl", "mpt_rr", "rinv_l", "rinv_r", "rear_fb")


def format_config_command(name: str, value: float) -> str:
    """Format a Pi -> ESP32 drive-tuning line, e.g. ``C min_duty 60``."""

    if name not in CONFIG_NAMES:
        raise ValueError(f"unknown config name {name!r}; expected one of {CONFIG_NAMES}")
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{name} must be a finite non-negative number")
    # 9 significant digits: metres per tick (~8.4e-05) keeps its precision; integers print as before.
    return f"C {name} {value:.9g}\n"


def wheel_geometry_commands(
    ticks_per_rev_left: float,
    ticks_per_rev_right: float,
    wheel_radius_left: float,
    wheel_radius_right: float,
    wheel_separation: float,
    ticks_per_rev_rear_left: float = 0.0,
    ticks_per_rev_rear_right: float = 0.0,
    wheel_radius_rear_left: float = 0.0,
    wheel_radius_rear_right: float = 0.0,
) -> list[str]:
    """``C mpt_l / mpt_r / sep`` lines from the wheel_odom_node values, so the ESP32 PI measures wheel speed
    with the same calibration as the odometry. A value <= 0 (not configured) sends nothing for that item and
    the firmware keeps its built-in default."""

    lines = []
    for name, ticks, radius in (("mpt_l", ticks_per_rev_left, wheel_radius_left),
                                ("mpt_r", ticks_per_rev_right, wheel_radius_right),
                                ("mpt_rl", ticks_per_rev_rear_left, wheel_radius_rear_left),
                                ("mpt_rr", ticks_per_rev_rear_right, wheel_radius_rear_right)):
        if math.isfinite(ticks) and math.isfinite(radius) and ticks > 0.0 and radius > 0.0:
            lines.append(format_config_command(name, 2.0 * math.pi * radius / ticks))
    if math.isfinite(wheel_separation) and wheel_separation > 0.0:
        lines.append(format_config_command("sep", wheel_separation))
    return lines


def parse_config_ack(line: bytes | str) -> Optional[tuple[str, str]]:
    """Parse an ESP32 ``K <name> <value>`` / ``K ERR <name>`` acknowledgement.

    Returns ``(name, value)`` (``("ERR", name)`` on rejection) or ``None`` for other lines.
    """

    if isinstance(line, bytes):
        try:
            text = line.decode("ascii").strip()
        except UnicodeDecodeError:
            return None
    else:
        text = line.strip()

    fields = text.split()
    if len(fields) != 3 or fields[0] != "K":
        return None
    return fields[1], fields[2]


# Battery voltage sample from the 0-25 V divider module on ESP32 ADC1 (firmware BATTERY_ADC_ENABLED):
#   "B <avg_pin_millivolts> <sample_count> <mcu_millis>"
BATTERY_PIN_MV_MAX = 3300  # ESP32-S3 ADC pin can never exceed its 3.3 V rail


def parse_battery_line(line: bytes | str) -> Optional[tuple[int, int, int]]:
    """Parse one ``B`` battery line.

    Returns ``(pin_millivolts, sample_count, mcu_millis)`` or ``None`` for any other or malformed line.
    """

    if isinstance(line, bytes):
        try:
            text = line.decode("ascii").strip()
        except UnicodeDecodeError:
            return None
    else:
        text = line.strip()

    fields = text.split()
    if len(fields) != 4 or fields[0] != "B":
        return None

    try:
        pin_mv = int(fields[1], 10)
        samples = int(fields[2], 10)
        mcu_millis = int(fields[3], 10)
    except ValueError:
        return None

    if not 0 <= pin_mv <= BATTERY_PIN_MV_MAX:
        return None
    if not 1 <= samples <= 65535:
        return None
    if not 0 <= mcu_millis <= UINT32_MAX:
        return None

    return pin_mv, samples, mcu_millis
