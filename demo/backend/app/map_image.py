"""Encode OccupancyGrid data to PNG for the browser."""

from __future__ import annotations

from io import BytesIO
from typing import Optional

from PIL import Image

from .map_coords import MapMeta


def occupancy_to_png(
    data: list[int] | bytes,
    meta: MapMeta,
) -> bytes:
    """Build PNG: free=white, occupied=black, unknown=gray.

    OccupancyGrid row-major, row 0 = bottom (origin). PNG row 0 = top,
    so we flip vertically when writing pixels.
    """
    w, h = meta.width, meta.height
    if len(data) < w * h:
        raise ValueError(f"occupancy length {len(data)} < {w * h}")

    img = Image.new("L", (w, h))
    pixels = img.load()
    assert pixels is not None

    for row_from_bottom in range(h):
        row_from_top = h - 1 - row_from_bottom
        base = row_from_bottom * w
        for col in range(w):
            v = int(data[base + col])
            if v < 0:
                gray = 128  # unknown
            elif v >= 50:
                gray = 0  # occupied
            else:
                gray = 255  # free
            pixels[col, row_from_top] = gray

    buf = BytesIO()
    img.save(buf, format="PNG")
    return buf.getvalue()
