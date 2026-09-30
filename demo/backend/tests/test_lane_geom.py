"""Unit tests for lane corridor geometry."""

from __future__ import annotations

import math

from app.lane_geom import lane_corridor_corners


def test_corridor_width_perpendicular() -> None:
    corners = lane_corridor_corners(0.0, 0.0, 2.0, 0.0, 1.0)
    # Along +X, half-width 0.5 in ±Y
    assert corners[0] == (0.0, 0.5) or math.isclose(corners[0][1], 0.5)
    ys = [c[1] for c in corners]
    assert max(ys) == 0.5
    assert min(ys) == -0.5


def test_zero_length_degenerate() -> None:
    corners = lane_corridor_corners(1.0, 1.0, 1.0, 1.0, 0.4)
    assert len(corners) == 4
