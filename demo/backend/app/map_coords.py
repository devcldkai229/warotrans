"""Map metadata and occupancy ↔ ROS map frame conversion.

Công thức khớp frontend MapCoordinateTransformer.ts.

PNG / browser: hàng 0 = đỉnh ảnh (Y tăng xuống).
ROS OccupancyGrid: index 0 = góc origin (thường dưới-trái), Y tăng lên.

Khi origin_yaw ≠ 0, cell (col, row_from_bottom) được xoay quanh origin
trong frame map trước khi cộng origin_x/y.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import cos, sin
from typing import Tuple


@dataclass(frozen=True)
class MapMeta:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0

    def to_dict(self) -> dict:
        return {
            "width": self.width,
            "height": self.height,
            "resolution": self.resolution,
            "origin_x": self.origin_x,
            "origin_y": self.origin_y,
            "origin_yaw": self.origin_yaw,
        }


class MapCoordinateTransformer:
    """Convert between ROS map metres and browser image pixels."""

    def __init__(self, meta: MapMeta) -> None:
        if meta.width < 1 or meta.height < 1:
            raise ValueError("map width/height must be >= 1")
        if meta.resolution <= 0.0:
            raise ValueError("map resolution must be > 0")
        self.meta = meta
        self._c = cos(meta.origin_yaw)
        self._s = sin(meta.origin_yaw)

    def map_to_pixel(self, x: float, y: float) -> Tuple[float, float]:
        """ROS map (x, y) metres → image (col, row_from_top) pixels."""
        m = self.meta
        dx = x - m.origin_x
        dy = y - m.origin_y
        # Inverse rotation of origin yaw
        local_x = self._c * dx + self._s * dy
        local_y = -self._s * dx + self._c * dy
        col = local_x / m.resolution
        row_from_bottom = local_y / m.resolution
        row_from_top = (m.height - 1) - row_from_bottom
        return col, row_from_top

    def pixel_to_map(self, col: float, row_from_top: float) -> Tuple[float, float]:
        """Image (col, row_from_top) pixels → ROS map (x, y) metres."""
        m = self.meta
        row_from_bottom = (m.height - 1) - row_from_top
        local_x = col * m.resolution
        local_y = row_from_bottom * m.resolution
        x = m.origin_x + self._c * local_x - self._s * local_y
        y = m.origin_y + self._s * local_x + self._c * local_y
        return x, y

    def tip_in_map(
        self, x: float, y: float, yaw: float, length_m: float
    ) -> Tuple[float, float]:
        """Point length_m ahead of (x,y) along yaw in map frame."""
        return x + length_m * cos(yaw), y + length_m * sin(yaw)
