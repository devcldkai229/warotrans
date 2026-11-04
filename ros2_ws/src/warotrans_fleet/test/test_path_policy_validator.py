"""Regression: path_policy_validator must not shadow pathlib with nav_msgs.Path."""

from __future__ import annotations

import ast
import importlib
import json
import sys
from pathlib import Path

import pytest

REPO_SIM_SCRIPTS = (
    Path(__file__).resolve().parents[2] / "warotrans_simulation" / "scripts"
)
VALIDATOR_SRC = (
    Path(__file__).resolve().parents[1] / "warotrans_fleet" / "path_policy_validator.py"
)


def test_source_does_not_shadow_pathlib_path():
    """Static guard: bare name Path must not be rebound to nav_msgs after pathlib import."""
    tree = ast.parse(VALIDATOR_SRC.read_text(encoding="utf-8"))
    pathlib_names: set[str] = set()
    nav_path_names: set[str] = set()
    for node in tree.body:
        if isinstance(node, ast.ImportFrom):
            if node.module == "pathlib":
                for alias in node.names:
                    pathlib_names.add(alias.asname or alias.name)
            if node.module == "nav_msgs.msg":
                for alias in node.names:
                    if alias.name == "Path":
                        nav_path_names.add(alias.asname or alias.name)
    assert "FsPath" in pathlib_names or (
        "Path" in pathlib_names and "Path" not in nav_path_names
    ), f"pathlib aliases={pathlib_names}, nav aliases={nav_path_names}"
    assert "NavPath" in nav_path_names or "Path" not in nav_path_names
    assert not ({"Path"} & pathlib_names & nav_path_names)


def test_module_imports_and_main_exists():
    pytest.importorskip("rclpy")
    if str(REPO_SIM_SCRIPTS) not in sys.path:
        sys.path.insert(0, str(REPO_SIM_SCRIPTS))
    module = importlib.import_module("warotrans_fleet.path_policy_validator")
    assert callable(module.main)
    dirs = module._adapter_dirs()  # noqa: SLF001
    assert dirs
    for d in dirs:
        assert hasattr(d, "is_file")
        assert not hasattr(d, "poses")


def test_path_policy_validator_constructs_with_json_paths(tmp_path: Path):
    rclpy = pytest.importorskip("rclpy")
    if str(REPO_SIM_SCRIPTS) not in sys.path:
        sys.path.insert(0, str(REPO_SIM_SCRIPTS))

    lanes = {
        "lanes": [
            {
                "id": "L1",
                "direction": "ONE_WAY",
                "width": 0.8,
                "polyline": [[0.0, 0.0], [2.0, 0.0]],
                "tangent_samples": [[1.0, 0.0]],
            }
        ]
    }
    no_stop = {
        "zones": [
            {
                "code": "NS1",
                "rings": [[[10.0, 10.0], [10.5, 10.0], [10.5, 10.5], [10.0, 10.5]]],
            }
        ]
    }
    keepout = {
        "zones": [
            {
                "code": "KO1",
                "rings": [[[20.0, 20.0], [20.5, 20.0], [20.5, 20.5], [20.0, 20.5]]],
            }
        ]
    }
    lanes_p = tmp_path / "lanes.json"
    no_stop_p = tmp_path / "no_stop.json"
    keepout_p = tmp_path / "keepout.json"
    lanes_p.write_text(json.dumps(lanes), encoding="utf-8")
    no_stop_p.write_text(json.dumps(no_stop), encoding="utf-8")
    keepout_p.write_text(json.dumps(keepout), encoding="utf-8")

    from rclpy.parameter import Parameter
    from warotrans_fleet.path_policy_validator import PathPolicyValidator

    rclpy.init(
        args=[
            "--ros-args",
            "-p",
            f"lanes_json:={lanes_p}",
            "-p",
            f"no_stop_json:={no_stop_p}",
            "-p",
            f"keepout_json:={keepout_p}",
        ]
    )
    try:
        node = PathPolicyValidator()
        assert len(node.lanes) == 1
        assert len(node.no_stop_rings) == 1
        assert len(node.keepout_rings) == 1
        node.set_parameters(
            [
                Parameter("lanes_json", value=str(lanes_p)),
                Parameter("no_stop_json", value=str(no_stop_p)),
                Parameter("keepout_json", value=str(keepout_p)),
            ]
        )
        node._reload()  # noqa: SLF001
        assert len(node.lanes) == 1
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()
