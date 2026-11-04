#!/usr/bin/env python3
"""Export the Semantic traffic bundle of one published map for the REAL robot (no Gazebo world).

Input is a read-only DB snapshot made with
    powershell -File warotrans-system/samples/maps/export-map-snapshot.ps1 -MapCode MAP-ONE-V002

Output (warotrans_navigation/maps/<slug>/, installed to share/ by CMake):
    map.pgm, map.yaml              occupancy, byte-identical to the published map
    lanes.json                     LaneAwareNavfn lanes (ONE_WAY / BIDIRECTIONAL)
    single_occupancy.json, no_stop.json, keepout.json
    keepout.yaml/.pgm, speed.yaml/.pgm   costmap filter masks
    semantic_manifest.json         mapCode, mapVersionId, semanticMapFingerprint, ...

The backend (Traffic:SemanticMapDirectory) and the robot gateway (semantic_manifest launch arg) must read this
SAME manifest: the fingerprint is how a robot grant is tied to the map the backend reserves against.

    python3 export_semantic_bundle.py --snapshot <samples/maps/MAP-ONE-V002> [--out-slug map_one_v002]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

PKG_ROOT = Path(__file__).resolve().parent.parent
# The adapter is the single implementation of lanes/zones/fingerprint (also used by the sim bundle).
ADAPTER_DIR = PKG_ROOT.parent / "warotrans_simulation" / "scripts"
sys.path.insert(0, str(ADAPTER_DIR))
import semantic_map_adapter as sma  # noqa: E402


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _slugify(code: str) -> str:
    return re.sub(r"[^a-zA-Z0-9]+", "_", code.strip().lower()).strip("_") or "map_bundle"


def read_pgm_size(data: bytes) -> tuple[int, int]:
    """Width/height of a binary (P5) PGM, skipping header comments."""
    tokens: list[bytes] = []
    i = 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        start = i
        while not data[i:i + 1].isspace():
            i += 1
        tokens.append(data[start:i])
    if tokens[0] != b"P5":
        raise ValueError(f"expected a P5 PGM, got {tokens[0]!r}")
    return int(tokens[1]), int(tokens[2])


def _read_map_yaml(path: Path) -> dict:
    """map.yaml of a snapshot: flat `key: value` lines (origin is a [x, y, yaw] list)."""
    meta: dict = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        if ":" not in line or line.lstrip().startswith("#"):
            continue
        key, value = (s.strip() for s in line.split(":", 1))
        meta[key] = [float(v) for v in value.strip("[]").split(",")] if value.startswith("[") else value
    return meta


def export(snapshot_dir: Path, out_dir: Path) -> dict[str, Path]:
    snapshot_dir = snapshot_dir.resolve()
    snap = json.loads((snapshot_dir / "snapshot.json").read_text(encoding="utf-8"))
    mv = snap.get("mapVersion") or {}
    if str(mv.get("status") or "").upper() != "PUBLISHED":
        raise ValueError(f"{mv.get('code')} is {mv.get('status')}; only a PUBLISHED map can be dispatched")

    meta = _read_map_yaml(snapshot_dir / "map.yaml")
    pgm_src = snapshot_dir / "map.pgm"
    width, height = read_pgm_size(pgm_src.read_bytes())
    resolution = float(meta["resolution"])
    ox, oy, oyaw = (meta["origin"] + [0.0])[:3]
    if abs(oyaw) > 1e-9:
        raise ValueError("map origin yaw must be 0 (lanes/zones are authored in the map frame)")

    # The DB row is the contract; a snapshot whose image does not match it was exported from something else.
    if int(mv.get("image_width_pixels") or width) != width or int(mv.get("image_height_pixels") or height) != height:
        raise ValueError(f"PGM {width}x{height} != map_versions row {mv.get('image_width_pixels')}x{mv.get('image_height_pixels')}")
    if abs(float(mv.get("origin_x", ox)) - ox) > 1e-6 or abs(float(mv.get("origin_y", oy)) - oy) > 1e-6:
        raise ValueError("map.yaml origin != map_versions origin")

    out_dir.mkdir(parents=True, exist_ok=True)
    pgm_dst = out_dir / "map.pgm"
    pgm_dst.write_bytes(pgm_src.read_bytes())
    (out_dir / "map.yaml").write_text(
        "image: map.pgm\n"
        f"mode: {meta.get('mode', 'trinary')}\n"
        f"resolution: {resolution}\n"
        f"origin: [{ox}, {oy}, 0.0]\n"
        f"negate: {int(float(meta.get('negate', 0)))}\n"
        f"occupied_thresh: {float(meta.get('occupied_thresh', 0.65))}\n"
        f"free_thresh: {float(meta.get('free_thresh', 0.25))}\n",
        encoding="utf-8",
    )

    occupancy_sha = _sha256(pgm_dst)
    # Written first: export_from_snapshot merges its policy fields into the existing manifest.
    manifest = out_dir / "semantic_manifest.json"
    manifest.write_text(
        json.dumps(
            {
                "mapCode": str(mv["code"]),
                "mapVersionId": str(mv["id"]),
                "frameId": str(mv.get("frame_id") or "map"),
                "resolution": resolution,
                "origin": [ox, oy, 0.0],
                "occupancySha256": occupancy_sha,
                "imageWidth": width,
                "imageHeight": height,
                "contentHash": str(mv.get("content_hash") or ""),
                "footprintMeters": {"length": 0.32, "width": 0.26, "padding": 0.02},
                "navRuntime": "nav2_semantic",
                "target": "real_robot",
                "sourceSnapshot": snapshot_dir.name,
                "exportedAt": str(snap.get("exportedAt") or ""),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )

    paths = sma.export_from_snapshot(
        snap,
        width=width,
        height=height,
        origin_x=ox,
        origin_y=oy,
        resolution=resolution,
        out_dir=out_dir,
        occupancy_sha256=occupancy_sha,
    )
    return {"map_pgm": pgm_dst, "map_yaml": out_dir / "map.yaml", **paths}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--snapshot", type=Path, required=True, help="samples/maps/<MapCode> (read-only)")
    parser.add_argument("--out-slug", default=None, help="maps/<slug> (default: slugified map code)")
    args = parser.parse_args()

    snap = json.loads((args.snapshot / "snapshot.json").read_text(encoding="utf-8"))
    slug = args.out_slug or _slugify(str((snap.get("mapVersion") or {}).get("code") or args.snapshot.name))
    paths = export(args.snapshot, PKG_ROOT / "maps" / slug)
    manifest = json.loads(paths["manifest"].read_text(encoding="utf-8"))
    lanes = json.loads(paths["lanes"].read_text(encoding="utf-8"))["lanes"]
    print(f"{manifest['mapCode']} ({manifest['mapVersionId']}) -> {paths['manifest'].parent}")
    print(f"  fingerprint {manifest['semanticMapFingerprint']}")
    print(f"  lanes {len(lanes)}: " + ", ".join(f"{l.get('code')}[{l.get('direction')}]" for l in lanes))
    return 0


if __name__ == "__main__":
    sys.exit(main())
