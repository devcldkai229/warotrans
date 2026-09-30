"""Unit tests for MapCoordinateTransformer (no ROS required)."""

from __future__ import annotations

import math

import pytest

from app.map_coords import MapCoordinateTransformer, MapMeta
from app.map_image import occupancy_to_png


@pytest.fixture
def meta() -> MapMeta:
    return MapMeta(
        width=60,
        height=62,
        resolution=0.05,
        origin_x=-0.409,
        origin_y=-2.357,
        origin_yaw=0.0,
    )


@pytest.fixture
def xf(meta: MapMeta) -> MapCoordinateTransformer:
    return MapCoordinateTransformer(meta)


def test_origin_maps_to_bottom_left_pixel(xf: MapCoordinateTransformer, meta: MapMeta) -> None:
    col, row = xf.map_to_pixel(meta.origin_x, meta.origin_y)
    assert col == pytest.approx(0.0)
    assert row == pytest.approx(meta.height - 1)


def test_opposite_corner(xf: MapCoordinateTransformer, meta: MapMeta) -> None:
    x = meta.origin_x + (meta.width - 1) * meta.resolution
    y = meta.origin_y + (meta.height - 1) * meta.resolution
    col, row = xf.map_to_pixel(x, y)
    assert col == pytest.approx(meta.width - 1)
    assert row == pytest.approx(0.0)


def test_y_inverted_increasing_y_decreases_row(
    xf: MapCoordinateTransformer, meta: MapMeta
) -> None:
    x = meta.origin_x + 1.0
    _, row_low = xf.map_to_pixel(x, meta.origin_y)
    _, row_high = xf.map_to_pixel(x, meta.origin_y + 1.0)
    assert row_high < row_low


def test_roundtrip_pixel_map_pixel(xf: MapCoordinateTransformer) -> None:
    for col, row in [(0.0, 0.0), (10.5, 20.25), (59.0, 61.0), (30.0, 31.0)]:
        x, y = xf.pixel_to_map(col, row)
        c2, r2 = xf.map_to_pixel(x, y)
        assert c2 == pytest.approx(col, abs=0.5)
        assert r2 == pytest.approx(row, abs=0.5)


def test_roundtrip_map_pixel_map(xf: MapCoordinateTransformer, meta: MapMeta) -> None:
    points = [
        (meta.origin_x, meta.origin_y),
        (meta.origin_x + 1.0, meta.origin_y + 0.5),
        (0.0, 0.0),
    ]
    for x, y in points:
        col, row = xf.map_to_pixel(x, y)
        x2, y2 = xf.pixel_to_map(col, row)
        assert x2 == pytest.approx(x, abs=meta.resolution / 2)
        assert y2 == pytest.approx(y, abs=meta.resolution / 2)


def test_roundtrip_with_origin_yaw() -> None:
    yaw = math.pi / 6
    meta = MapMeta(40, 40, 0.05, -1.0, -1.0, origin_yaw=yaw)
    xf = MapCoordinateTransformer(meta)
    for x, y in [(0.0, 0.0), (-0.5, 0.25), (0.8, -0.3)]:
        col, row = xf.map_to_pixel(x, y)
        x2, y2 = xf.pixel_to_map(col, row)
        assert x2 == pytest.approx(x, abs=1e-9)
        assert y2 == pytest.approx(y, abs=1e-9)


def test_occupancy_png_flip(meta: MapMeta) -> None:
    data = [-1] * (meta.width * meta.height)
    data[0] = 100
    png = occupancy_to_png(data, meta)
    assert png[:8] == b"\x89PNG\r\n\x1a\n"
    from io import BytesIO

    from PIL import Image

    img = Image.open(BytesIO(png))
    assert img.getpixel((0, meta.height - 1)) == 0
    assert img.getpixel((0, 0)) == 128
