"""Lane corridor geometry helpers (map metres)."""

from __future__ import annotations

import math
from typing import List, Tuple


def lane_corridor_corners(
    start_x: float,
    start_y: float,
    end_x: float,
    end_y: float,
    width_m: float,
) -> List[Tuple[float, float]]:
    """Return 4 corners of the corridor rectangle (CCW) in map frame."""
    dx = end_x - start_x
    dy = end_y - start_y
    length = math.hypot(dx, dy)
    if length < 1e-9:
        hw = width_m * 0.5
        return [
            (start_x - hw, start_y - hw),
            (start_x + hw, start_y - hw),
            (start_x + hw, start_y + hw),
            (start_x - hw, start_y + hw),
        ]
    ux, uy = dx / length, dy / length
    # Left perpendicular
    px, py = -uy, ux
    hw = width_m * 0.5
    return [
        (start_x + px * hw, start_y + py * hw),
        (end_x + px * hw, end_y + py * hw),
        (end_x - px * hw, end_y - py * hw),
        (start_x - px * hw, start_y - py * hw),
    ]
