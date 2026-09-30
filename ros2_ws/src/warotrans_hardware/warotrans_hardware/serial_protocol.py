"""Pure helpers for the ESP32 text telemetry protocol."""

from __future__ import annotations

import math
from typing import Optional

INT64_MIN = -(2**63)
INT64_MAX = 2**63 - 1
UINT32_MAX = 2**32 - 1


def parse_encoder_line(
    line: bytes | str,
) -> Optional[tuple[int, int, int]]:
    """Parse one ESP32 telemetry line.

    Returns ``(left_ticks, right_ticks, mcu_millis)`` for a valid line and
    ``None`` for malformed or out-of-range input.
    """

    if isinstance(line, bytes):
        try:
            text = line.decode("ascii").strip()
        except UnicodeDecodeError:
            return None
    else:
        text = line.strip()

    fields = text.split()
    if len(fields) != 4 or fields[0] != "O":
        return None

    try:
        left_ticks = int(fields[1], 10)
        right_ticks = int(fields[2], 10)
        mcu_millis = int(fields[3], 10)
    except ValueError:
        return None

    if not INT64_MIN <= left_ticks <= INT64_MAX:
        return None
    if not INT64_MIN <= right_ticks <= INT64_MAX:
        return None
    if not 0 <= mcu_millis <= UINT32_MAX:
        return None

    return left_ticks, right_ticks, mcu_millis


def format_velocity_command(linear_mps: float, angular_rps: float) -> str:
    """Format a Pi -> ESP32 velocity command line."""

    if not math.isfinite(linear_mps) or not math.isfinite(angular_rps):
        raise ValueError("linear and angular must be finite")
    return f"V {linear_mps:.6f} {angular_rps:.6f}\n"
