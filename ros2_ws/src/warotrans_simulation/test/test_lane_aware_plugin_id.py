"""Regression: Nav2 LaneAware plugin string must match pluginlib XML type."""

from __future__ import annotations

import re
from pathlib import Path

import yaml

_SIM = Path(__file__).resolve().parents[1]
_WS = _SIM.parent
_YAML = _WS / "warotrans_navigation" / "config" / "nav2_params_lane_aware.yaml"
_XML = _WS / "warotrans_lane_planner" / "lane_aware_navfn_plugin.xml"


def test_lane_aware_yaml_plugin_matches_pluginlib_type():
    assert _YAML.is_file(), f"missing {_YAML}"
    assert _XML.is_file(), f"missing {_XML}"
    cfg = yaml.safe_load(_YAML.read_text(encoding="utf-8"))
    plugin = cfg["planner_server"]["ros__parameters"]["GridBased"]["plugin"]
    xml = _XML.read_text(encoding="utf-8")
    m = re.search(r'type="([^"]+)"', xml)
    assert m, "plugin XML missing type="
    declared = m.group(1)
    assert plugin == declared, f"YAML plugin={plugin!r} != XML type={declared!r}"
    assert "::" in plugin, "Jazzy Nav2 expects package::Class plugin ids"
    assert "/" not in plugin.split("::")[0] or plugin.count("/") == 0


def test_wrong_slash_form_is_not_used():
    text = _YAML.read_text(encoding="utf-8")
    assert "warotrans_lane_planner/LaneAwareNavfn" not in text
