"""fleet.launch.py: launch args must reach the gateway and the semantic bundle must agree with its map code."""

import importlib.util
import json
from pathlib import Path

import pytest

_LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "fleet.launch.py"
pytest.importorskip("launch_ros")
_spec = importlib.util.spec_from_file_location("fleet_launch", _LAUNCH)
fleet_launch = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(fleet_launch)


def _manifest(tmp_path: Path, code="MAP-ONE-V002", fp="f" * 64) -> str:
    path = tmp_path / "semantic_manifest.json"
    path.write_text(json.dumps({
        "mapCode": code, "semanticMapFingerprint": fp,
        "policies": {"single_occupancy": "single_occupancy.json", "lanes": "lanes.json"},
    }), encoding="utf-8")
    return str(path)


def _args(**values):
    return lambda name: values.get(name, "")


def test_launch_args_reach_the_node():
    # Was: declared but never passed, so robot_code:=/mqtt_host:= on the command line did nothing.
    overrides, _ = fleet_launch.gateway_overrides(
        _args(robot_code="RBT-001", mqtt_host="192.168.1.10", mqtt_port="1883"), {})
    assert overrides == {"robot_code": "RBT-001", "mqtt_host": "192.168.1.10", "mqtt_port": 1883}


def test_semantic_manifest_sets_fingerprint_map_code_and_zone_files(tmp_path):
    overrides, log = fleet_launch.gateway_overrides(_args(semantic_manifest=_manifest(tmp_path)), {})
    assert overrides["semantic_map_fingerprint"] == "f" * 64
    assert overrides["map_version_code"] == "MAP-ONE-V002"
    assert overrides["single_occupancy_json"] == str(tmp_path / "single_occupancy.json")
    assert "MAP-ONE-V002" in log


@pytest.mark.parametrize("args,environ", [
    ({"map_version_code": "MAP-ONE-V001"}, {}),
    ({}, {"WARO_MAP_VERSION_CODE": "MAP-ONE-V001"}),     # env wins inside the node
])
def test_map_code_disagreeing_with_the_bundle_is_refused(tmp_path, args, environ):
    with pytest.raises(RuntimeError, match="MAP-ONE-V002"):
        fleet_launch.gateway_overrides(_args(semantic_manifest=_manifest(tmp_path), **args), environ)


def test_manifest_without_fingerprint_is_refused(tmp_path):
    with pytest.raises(RuntimeError, match="semanticMapFingerprint"):
        fleet_launch.gateway_overrides(_args(semantic_manifest=_manifest(tmp_path, fp="")), {})


def test_real_v002_bundle_is_loadable():
    manifest = Path(__file__).resolve().parents[2] / "warotrans_navigation" / "maps" / "map_one_v002" / "semantic_manifest.json"
    if not manifest.is_file():
        pytest.skip("V002 bundle not exported in this checkout")
    overrides, _ = fleet_launch.gateway_overrides(_args(semantic_manifest=str(manifest)), {})
    assert overrides["map_version_code"] == "MAP-ONE-V002"
    assert len(overrides["semantic_map_fingerprint"]) == 64
