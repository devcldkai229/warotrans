#!/usr/bin/env python3
"""Build an isolated Gazebo/Nav2 sim bundle from a read-only map snapshot.

Does NOT modify the source snapshot or any database.

Outputs (under warotrans_simulation), keyed by --out-slug (default map_one_v001):
  maps/<slug>/map.pgm|.yaml + semantic policy artifacts
  config/layouts/<slug>.yaml
  worlds/<slug>.sdf
  config/<slug>_endpoints.json

Usage:
  python3 scripts/build_map_one_sim_bundle.py \\
    --snapshot /path/to/warotrans-system/samples/maps/MAP-ONE-V001
  python3 scripts/build_map_one_sim_bundle.py \\
    --snapshot /path/to/samples/maps/MAP-XXX --out-slug map_xxx
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import shutil
import sys
from pathlib import Path

import yaml

PKG_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(Path(__file__).resolve().parent))
import generate_warehouse as gw  # noqa: E402
import semantic_map_adapter as sma  # noqa: E402

FOCUS_CODES = ("EP-PARKING-A", "EP-INBOUND", "EP-SHELFA01", "EP-OUTBOUND")


def _sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    h.update(path.read_bytes())
    return h.hexdigest()


def _find_pgm(src: Path, meta_image: str) -> Path:
    candidates = [
        src / "map.pgm",
        src / meta_image,
        src / Path(meta_image).name,
    ]
    for c in candidates:
        if c.is_file():
            return c
    raise FileNotFoundError(f"PGM not found in {src} (tried map.pgm and {meta_image})")


def _slugify(code: str) -> str:
    s = re.sub(r"[^a-zA-Z0-9]+", "_", code.strip().lower()).strip("_")
    return s or "map_bundle"


def _pick_spawn(endpoints: list[dict]) -> dict:
    for code in FOCUS_CODES:
        hit = next((e for e in endpoints if e["code"] == code), None)
        if hit:
            return hit
    for e in endpoints:
        et = str(e.get("endpointType") or e.get("type") or "").upper()
        if "PARK" in et or "PARKING" in e["code"].upper():
            return e
    if not endpoints:
        raise RuntimeError("snapshot has no endpoints for spawn")
    return endpoints[0]


def build(
    snapshot_dir: Path,
    pkg_root: Path = PKG_ROOT,
    *,
    out_slug: str | None = None,
    require_focus: bool = False,
) -> dict[str, Path]:
    snapshot_dir = snapshot_dir.resolve()
    snap_path = snapshot_dir / "snapshot.json"
    yaml_src = snapshot_dir / "map.yaml"
    if not snap_path.is_file():
        raise FileNotFoundError(f"missing {snap_path}")
    if not yaml_src.is_file():
        raise FileNotFoundError(f"missing {yaml_src}")

    snap = json.loads(snap_path.read_text(encoding="utf-8"))
    mv = snap.get("mapVersion") or {}
    meta = yaml.safe_load(yaml_src.read_text(encoding="utf-8"))
    pgm_src = _find_pgm(snapshot_dir, str(meta.get("image") or "map.pgm"))

    map_code = str(mv.get("code") or snapshot_dir.name)
    slug = out_slug or (
        "map_one_v001" if map_code.upper() == "MAP-ONE-V001" else _slugify(map_code)
    )

    out_map = pkg_root / "maps" / slug
    out_map.mkdir(parents=True, exist_ok=True)
    pgm_dst = out_map / "map.pgm"
    yaml_dst = out_map / "map.yaml"
    shutil.copy2(pgm_src, pgm_dst)

    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    oyaw = float(meta["origin"][2]) if len(meta["origin"]) > 2 else 0.0
    if abs(oyaw) > 1e-9:
        raise ValueError(f"{map_code} origin yaw must be 0 (world≡map contract)")

    map_meta = {
        "image": "map.pgm",
        "mode": meta.get("mode", "trinary"),
        "resolution": res,
        "origin": [ox, oy, 0.0],
        "negate": int(meta.get("negate", 0)),
        "occupied_thresh": float(meta.get("occupied_thresh", 0.65)),
        "free_thresh": float(meta.get("free_thresh", 0.25)),
    }
    yaml_dst.write_text(yaml.safe_dump(map_meta, sort_keys=False), encoding="utf-8")

    endpoints = []
    for e in snap.get("endpoints") or []:
        if not isinstance(e, dict) or not e.get("code"):
            continue
        endpoints.append(
            {
                "code": str(e["code"]),
                "name": str(e.get("name") or e["code"]),
                "type": str(e.get("endpointType") or e.get("type") or "WAYPOINT"),
                "endpointType": str(e.get("endpointType") or e.get("type") or "WAYPOINT"),
                "x": float(e["x"]),
                "y": float(e["y"]),
                "yaw": float(e.get("yaw") or 0.0),
                "isEnabled": bool(e.get("isEnabled", True)),
            }
        )

    focus = [e for e in endpoints if e["code"] in FOCUS_CODES]
    missing = [c for c in FOCUS_CODES if not any(e["code"] == c for e in endpoints)]
    if require_focus and missing:
        raise RuntimeError(f"snapshot missing focus endpoints: {missing}")

    spawn = _pick_spawn(endpoints)
    layout = {
        "name": slug,
        "source_map": f"maps/{slug}/map.yaml",
        "wall_height": 0.8,
        "spawn_endpoint_code": spawn["code"],
        "spawn_require_clearance": False,
        "endpoints": endpoints,
        "shelves": [],
        "web": {
            "map_version_code": map_code,
            "map_version_id": str(mv.get("id") or ""),
            "note": f"Offline sim bundle from {snapshot_dir.name} (DB read-only)",
        },
    }
    layout_path = pkg_root / "config" / "layouts" / f"{slug}.yaml"
    layout_path.parent.mkdir(parents=True, exist_ok=True)
    layout_path.write_text(yaml.safe_dump(layout, sort_keys=False), encoding="utf-8")

    rendered = gw.render_from_source_map(layout, layout_path.parent)
    world_bytes = rendered[gw.WORLD_OUT]
    world_path = pkg_root / "worlds" / f"{slug}.sdf"
    world_path.parent.mkdir(parents=True, exist_ok=True)
    world_path.write_bytes(world_bytes)

    endpoints_out = pkg_root / "config" / f"{slug}_endpoints.json"
    endpoints_out.write_bytes(
        gw.endpoints_json(
            slug,
            {"x": spawn["x"], "y": spawn["y"], "yaw": spawn["yaw"]},
            [
                {
                    "code": e["code"],
                    "name": e["name"],
                    "endpointType": e["endpointType"],
                    "x": e["x"],
                    "y": e["y"],
                    "yaw": e["yaw"],
                }
                for e in endpoints
            ],
        )
    )

    occ_fp = _sha256_file(pgm_dst)
    width, height, _pixels = gw.read_pgm(pgm_dst.read_bytes())
    payload = {
        "mapCode": map_code,
        "mapVersionId": str(mv.get("id") or ""),
        "frameId": str(mv.get("frame_id") or "map"),
        "resolution": res,
        "origin": [ox, oy, 0.0],
        "occupancySha256": occ_fp,
        "imageWidth": int(mv.get("image_width_pixels") or width),
        "imageHeight": int(mv.get("image_height_pixels") or height),
        "contentHash": str(mv.get("content_hash") or ""),
        "focusEndpoints": {
            e["code"]: {"x": e["x"], "y": e["y"], "yaw": e["yaw"]}
            for e in (focus or [spawn])
        },
        "footprintMeters": {"length": 0.32, "width": 0.26, "padding": 0.02},
        "navRuntime": "nav2_semantic",
        "sourceSnapshot": str(snapshot_dir),
        "world": f"worlds/{slug}.sdf",
        "mapYaml": f"maps/{slug}/map.yaml",
        "layout": f"config/layouts/{slug}.yaml",
        "stage": 2,
        "outSlug": slug,
        "spawnEndpointCode": spawn["code"],
    }
    manifest_path = out_map / "semantic_manifest.json"
    manifest_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    policy_paths = sma.export_from_snapshot(
        snap,
        width=width,
        height=height,
        origin_x=ox,
        origin_y=oy,
        resolution=res,
        out_dir=out_map,
        occupancy_sha256=occ_fp,
    )

    return {
        "layout": layout_path,
        "map_yaml": yaml_dst,
        "map_pgm": pgm_dst,
        "world": world_path,
        "endpoints": endpoints_out,
        "manifest": manifest_path,
        **policy_paths,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--snapshot",
        type=Path,
        required=True,
        help="Path to samples/maps/<MapCode> (read-only)",
    )
    parser.add_argument(
        "--pkg-root",
        type=Path,
        default=PKG_ROOT,
        help="warotrans_simulation package root",
    )
    parser.add_argument(
        "--out-slug",
        type=str,
        default=None,
        help="Output directory/world slug (default: map_one_v001 for MAP-ONE-V001 else slugified code)",
    )
    parser.add_argument(
        "--require-focus",
        action="store_true",
        help="Fail if classic MAP-ONE focus endpoint codes are missing",
    )
    args = parser.parse_args()
    written = build(
        args.snapshot,
        args.pkg_root,
        out_slug=args.out_slug,
        require_focus=args.require_focus,
    )
    for key, path in written.items():
        print(f"wrote {key}: {path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
