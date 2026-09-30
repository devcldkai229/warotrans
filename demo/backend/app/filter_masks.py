"""Semantic TrafficZone → Nav2 costmap filter masks (PGM + YAML).

KEEP_OUT: OccupancyGrid 100 inside polygon (lethal for KeepoutFilter).
SPEED_LIMIT: OccupancyGrid encodes absolute m/s via base/multiplier on info server.
  With type=2 (absolute), base=0, multiplier=0.01 → occupancy N ⇒ N*0.01 m/s.
"""

from __future__ import annotations

import re
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

from .map_coords import MapCoordinateTransformer, MapMeta


def _point_in_poly(x: float, y: float, poly: Sequence) -> bool:
    n = len(poly)
    if n < 3:
        return False
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = float(poly[i].x), float(poly[i].y)
        xj, yj = float(poly[j].x), float(poly[j].y)
        if ((yi > y) != (yj > y)) and (
            x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
        ):
            inside = not inside
        j = i
    return inside


def _empty_grid(meta: MapMeta) -> List[int]:
    return [0] * (meta.width * meta.height)


def rasterize_keepout(meta: MapMeta, zones: Iterable) -> List[int]:
    from .session import ZoneType

    xf = MapCoordinateTransformer(meta)
    data = _empty_grid(meta)
    keepouts = [
        z
        for z in zones
        if z.enabled and z.type == ZoneType.KEEP_OUT and len(z.polygon) >= 3
    ]
    if not keepouts:
        return data
    for row in range(meta.height):
        for col in range(meta.width):
            x, y = xf.pixel_to_map(col + 0.5, row + 0.5)
            for z in keepouts:
                if _point_in_poly(x, y, z.polygon):
                    data[row * meta.width + col] = 100
                    break
    return data


def rasterize_speed(meta: MapMeta, zones: Iterable) -> List[int]:
    """Encode maxSpeedMps as occupancy = round(mps / 0.01), clamp 1..100."""
    from .session import ZoneType

    xf = MapCoordinateTransformer(meta)
    data = _empty_grid(meta)
    speeds = [
        z
        for z in zones
        if z.enabled and z.type == ZoneType.SPEED_LIMIT and len(z.polygon) >= 3
    ]
    if not speeds:
        return data
    for row in range(meta.height):
        for col in range(meta.width):
            x, y = xf.pixel_to_map(col + 0.5, row + 0.5)
            best: Optional[float] = None
            for z in speeds:
                if _point_in_poly(x, y, z.polygon):
                    v = float(z.maxSpeedMps) if z.maxSpeedMps and z.maxSpeedMps > 0 else 0.05
                    best = v if best is None else min(best, v)
            if best is not None:
                occ = int(round(best / 0.01))
                data[row * meta.width + col] = max(1, min(100, occ))
    return data


def write_mask_pgm_yaml(
    meta: MapMeta,
    data: Sequence[int],
    yaml_path: Path,
    *,
    mode: str = "raw",
) -> Path:
    """Write PGM (raw occupancy lightness) + YAML next to it. Returns yaml path."""
    yaml_path = Path(yaml_path)
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    pgm_path = yaml_path.with_suffix(".pgm")
    # OccupancyGrid row 0 = bottom in ROS; PGM row 0 = top.
    # Our raster used row_from_top via pixel_to_map — data[0] is top row.
    # map_server raw mode: lightness maps to occupancy; write top-first PGM.
    w, h = meta.width, meta.height
    if len(data) != w * h:
        raise ValueError("mask size mismatch")
    header = f"P5\n{w} {h}\n255\n".encode("ascii")
    body = bytearray(w * h)
    for i, v in enumerate(data):
        # raw: 0..100 used; clamp
        body[i] = max(0, min(100, int(v)))
    pgm_path.write_bytes(header + bytes(body))
    yaml_path.write_text(
        "\n".join(
            [
                f"image: {pgm_path.name}",
                f"mode: {mode}",
                f"resolution: {meta.resolution}",
                f"origin: [{meta.origin_x}, {meta.origin_y}, 0.0]",
                "negate: false",
                "occupied_thresh: 0.65",
                "free_thresh: 0.25",
                "",
            ]
        ),
        encoding="utf-8",
    )
    return yaml_path


def _parse_map_yaml(map_yaml: Path) -> Tuple[Path, float, float, float]:
    text = map_yaml.read_text(encoding="utf-8")
    image_m = re.search(r"^image:\s*(.+)$", text, re.M)
    res_m = re.search(r"^resolution:\s*([0-9.eE+-]+)", text, re.M)
    ori_m = re.search(
        r"^origin:\s*\[\s*([0-9.eE+-]+)\s*,\s*([0-9.eE+-]+)", text, re.M
    )
    if not image_m or not res_m or not ori_m:
        raise ValueError(f"cannot parse map yaml: {map_yaml}")
    image = Path(image_m.group(1).strip().strip("'\""))
    if not image.is_absolute():
        image = map_yaml.parent / image
    return (
        image,
        float(res_m.group(1)),
        float(ori_m.group(1)),
        float(ori_m.group(2)),
    )


def ensure_empty_masks_from_map_yaml(map_yaml: Path, out_dir: Path) -> Tuple[Path, Path]:
    """Create empty keepout/speed masks matching an occupancy map YAML."""
    map_yaml = Path(map_yaml)
    image, resolution, origin_x, origin_y = _parse_map_yaml(map_yaml)
    with image.open("rb") as f:
        magic = f.readline().strip()
        if magic != b"P5":
            raise ValueError(f"unsupported map image {image}")
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        wh = line.split()
        width, height = int(wh[0]), int(wh[1])
        _ = f.readline()
    meta = MapMeta(
        width=width,
        height=height,
        resolution=resolution,
        origin_x=origin_x,
        origin_y=origin_y,
        origin_yaw=0.0,
    )
    out_dir = Path(out_dir)
    keepout = write_mask_pgm_yaml(
        meta, _empty_grid(meta), out_dir / "warotrans_keepout.yaml"
    )
    speed = write_mask_pgm_yaml(
        meta, _empty_grid(meta), out_dir / "warotrans_speed.yaml"
    )
    return keepout, speed
