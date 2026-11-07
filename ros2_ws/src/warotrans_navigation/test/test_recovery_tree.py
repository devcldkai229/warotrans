"""Static guarantees of the real-robot recovery: nothing here may weaken a safety layer to make the robot move."""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

yaml = pytest.importorskip("yaml")

PKG = Path(__file__).resolve().parents[1]
TREE = ET.parse(PKG / "behavior_trees" / "navigate_to_pose_w_replanning_resilient.xml").getroot()
PARAMS = yaml.safe_load((PKG / "config" / "nav2_params.yaml").read_text(encoding="utf-8"))
LAUNCH = (PKG / "launch" / "navigation.launch.py").read_text(encoding="utf-8")


def test_recovery_motion_only_through_the_guard():
    motions = [n for n in TREE.iter() if n.tag in ("BackUp", "Spin", "DriveOnHeading", "AssistedTeleop")]
    assert {n.tag for n in motions} == {"BackUp", "Spin"}
    for node in motions:
        assert node.attrib.get("server_name") == f"guarded_{node.tag.lower()}"


def test_order_is_back_up_then_both_turns_then_clearing():
    round_robin = next(TREE.iter("RoundRobin"))
    names = [c.attrib.get("name", c.tag) for c in round_robin]
    assert names == ["GuardedBackUp", "GuardedSpinLeft", "GuardedSpinRight", "ClearingActions", "Wait"]


def test_back_up_stays_inside_the_guaranteed_grant_margin():
    # Reserve radius minus gate radius at cell 0.25 m: 0.131 m. The guard and gateway check 0.12 m.
    node = next(TREE.iter("BackUp"))
    assert float(node.attrib["backup_dist"]) <= 0.12
    assert 0 < float(node.attrib["backup_speed"]) <= 0.06


def test_turns_go_both_ways():
    assert sorted(float(n.attrib["spin_dist"]) > 0 for n in TREE.iter("Spin")) == [False, True]


def test_recovery_attempts_are_bounded():
    for node in TREE.iter("RecoveryNode"):
        assert 1 <= int(node.attrib["number_of_retries"]) <= 5


def test_collision_monitor_is_not_weakened():
    cm = PARAMS["collision_monitor"]["ros__parameters"]
    polygon = cm["FootprintApproach"]
    assert cm["polygons"] == ["FootprintApproach"]
    assert polygon["enabled"] is True
    assert polygon["action_type"] == "approach"
    assert polygon["time_before_collision"] == 1.2
    assert polygon["min_points"] == 4
    assert polygon["footprint_topic"] == "/local_costmap/published_footprint"
    assert cm["scan"]["enabled"] is True


@pytest.mark.parametrize("costmap", ["local_costmap", "global_costmap"])
def test_footprint_margin_is_kept(costmap):
    params = PARAMS[costmap][costmap]["ros__parameters"]
    assert params["footprint_padding"] == 0.02
    assert params["footprint"] == "[[0.16, 0.13], [0.16, -0.13], [-0.16, -0.13], [-0.16, 0.13]]"


def test_wheel_only_amcl_differs_from_the_ekf_one_only_in_motion_noise():
    loc = PKG.parent / "warotrans_localization" / "config"
    if not loc.is_dir():
        pytest.skip("warotrans_localization is not in this workspace")
    ekf = yaml.safe_load((loc / "amcl.yaml").read_text(encoding="utf-8"))
    wheel = yaml.safe_load((loc / "amcl_wheel_only.yaml").read_text(encoding="utf-8"))
    a, b = ekf["amcl"]["ros__parameters"], wheel["amcl"]["ros__parameters"]
    assert [b[f"alpha{i}"] for i in (1, 2, 3, 4)] == [0.3, 0.3, 0.2, 0.2]
    assert [a[f"alpha{i}"] for i in (1, 2, 3, 4)] == [0.15] * 4
    for params in (a, b):
        for i in (1, 2, 3, 4):
            params.pop(f"alpha{i}")
    assert ekf == wheel


def test_primary_controller_still_forward_only():
    assert PARAMS["controller_server"]["ros__parameters"]["FollowPath"]["allow_reversing"] is False


def test_guard_runs_with_the_pipeline_and_needs_traffic_when_the_gate_is_on():
    assert re.search(r'executable="recovery_guard"', LAUNCH)
    assert '"require_traffic_authorization": enable_authority_gate' in LAUNCH
    # Behaviors keep publishing into the pipeline: the guard forwards, it has no velocity output of its own.
    guard = (PKG.parent / "warotrans_fleet" / "warotrans_fleet" / "recovery_guard.py").read_text(encoding="utf-8")
    assert "Twist" not in guard and "cmd_vel" not in guard
