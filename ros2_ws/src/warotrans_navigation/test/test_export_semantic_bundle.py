"""export_semantic_bundle: the robot and the backend must read one manifest for one published map."""

import importlib.util
import json
from pathlib import Path

import pytest

_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "export_semantic_bundle.py"
_spec = importlib.util.spec_from_file_location("export_semantic_bundle", _SCRIPT)
esb = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(esb)

MAP_ID = "797ac828-7c92-494f-8715-c15903b48ac3"


def _snapshot(tmp_path: Path, *, status="PUBLISHED", direction="ONE_WAY", origin=(-1.623, -1.456)) -> Path:
    snap_dir = tmp_path / "snap"
    snap_dir.mkdir(parents=True)
    (snap_dir / "map.pgm").write_bytes(b"P5\n# comment\n4 3\n255\n" + bytes([254] * 12))
    (snap_dir / "map.yaml").write_text(
        "image: other_name.pgm\nmode: trinary\nresolution: 0.050\n"
        "origin: [-1.623, -1.456, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n",
        encoding="utf-8",
    )
    snapshot = {
        "exportedAt": "2026-11-04T00:00:00Z",
        "mapVersion": {
            "id": MAP_ID, "code": "MAP-ONE-V002", "status": status, "frame_id": "map",
            "image_width_pixels": 4, "image_height_pixels": 3,
            "origin_x": origin[0], "origin_y": origin[1], "content_hash": "abc",
        },
        "endpoints": [{"code": "EP-INBOUND", "x": 0.0, "y": 0.0, "yaw": -1.5708}],
        "edges": [{
            "code": "EDGE-LANE-BTT-B---2", "direction": direction, "width_meters": 0.7, "is_active": True,
            "geometry": {"type": "LineString", "coordinates": [[1.338, 0.293], [1.331, 1.183]]},
        }],
        "zones": [{
            "code": "ZONE-SINGLE-B", "zone_type": "SINGLE_OCCUPANCY", "is_active": True,
            "geometry": {"type": "Polygon", "coordinates": [[[0, 0], [1, 0], [1, 1], [0, 1], [0, 0]]]},
        }],
    }
    (snap_dir / "snapshot.json").write_text(json.dumps(snapshot), encoding="utf-8")
    return snap_dir


def test_manifest_names_the_published_map_and_keeps_the_pgm_byte_identical(tmp_path):
    snap = _snapshot(tmp_path)
    paths = esb.export(snap, tmp_path / "out")

    manifest = json.loads(paths["manifest"].read_text(encoding="utf-8"))
    assert manifest["mapCode"] == "MAP-ONE-V002"
    assert manifest["mapVersionId"] == MAP_ID           # backend refuses a Job on any other map
    assert len(manifest["semanticMapFingerprint"]) == 64
    assert paths["map_pgm"].read_bytes() == (snap / "map.pgm").read_bytes()
    assert "image: map.pgm" in paths["map_yaml"].read_text(encoding="utf-8")
    assert "free_thresh: 0.196" in paths["map_yaml"].read_text(encoding="utf-8")

    lanes = json.loads(paths["lanes"].read_text(encoding="utf-8"))["lanes"]
    assert [(l["code"], l["direction"]) for l in lanes] == [("EDGE-LANE-BTT-B---2", "ONE_WAY")]
    zones = json.loads(paths["single_occupancy"].read_text(encoding="utf-8"))["zones"]
    assert [z["code"] for z in zones] == ["ZONE-SINGLE-B"]


def test_fingerprint_is_stable_and_follows_lane_direction(tmp_path):
    a = esb.export(_snapshot(tmp_path / "a"), tmp_path / "a" / "out")
    b = esb.export(_snapshot(tmp_path / "b"), tmp_path / "b" / "out")
    c = esb.export(_snapshot(tmp_path / "c", direction="BIDIRECTIONAL"), tmp_path / "c" / "out")

    fp = lambda p: json.loads(p["manifest"].read_text(encoding="utf-8"))["semanticMapFingerprint"]  # noqa: E731
    assert fp(a) == fp(b)
    # Flipping ONE_WAY changes what the planner may do, so a stale robot bundle must not match.
    assert fp(a) != fp(c)


def test_draft_map_is_refused(tmp_path):
    with pytest.raises(ValueError, match="PUBLISHED"):
        esb.export(_snapshot(tmp_path, status="DRAFT"), tmp_path / "out")


def test_origin_disagreeing_with_the_db_row_is_refused(tmp_path):
    with pytest.raises(ValueError, match="origin"):
        esb.export(_snapshot(tmp_path, origin=(-2.929, -1.415)), tmp_path / "out")
