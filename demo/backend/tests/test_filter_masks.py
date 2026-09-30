"""Unit tests for KEEP_OUT / SPEED_LIMIT mask rasterization."""

from pathlib import Path

from app.filter_masks import (
    rasterize_keepout,
    rasterize_speed,
    write_mask_pgm_yaml,
)
from app.map_coords import MapMeta
from app.session import TrafficZone, ZonePoint, ZoneType


def _meta() -> MapMeta:
    return MapMeta(
        width=20,
        height=20,
        resolution=0.05,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
    )


def test_keepout_marks_polygon_cells(tmp_path: Path):
    meta = _meta()
    z = TrafficZone(
        id="k",
        name="KO",
        type=ZoneType.KEEP_OUT,
        polygon=[
            ZonePoint(x=0.1, y=0.1),
            ZonePoint(x=0.4, y=0.1),
            ZonePoint(x=0.4, y=0.4),
            ZonePoint(x=0.1, y=0.4),
        ],
    )
    data = rasterize_keepout(meta, [z])
    assert any(v == 100 for v in data)
    assert any(v == 0 for v in data)
    yaml_path = write_mask_pgm_yaml(meta, data, tmp_path / "keepout.yaml")
    assert yaml_path.is_file()
    assert yaml_path.with_suffix(".pgm").is_file()


def test_speed_encodes_mps():
    meta = _meta()
    z = TrafficZone(
        id="s",
        name="SL",
        type=ZoneType.SPEED_LIMIT,
        maxSpeedMps=0.05,
        polygon=[
            ZonePoint(x=0.1, y=0.1),
            ZonePoint(x=0.3, y=0.1),
            ZonePoint(x=0.3, y=0.3),
            ZonePoint(x=0.1, y=0.3),
        ],
    )
    data = rasterize_speed(meta, [z])
    vals = {v for v in data if v > 0}
    assert 5 in vals  # 0.05 / 0.01
