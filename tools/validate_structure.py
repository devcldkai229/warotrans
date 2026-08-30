"""Offline structural validation for the WaroTrans repo.

This does not replace `colcon build` on Ubuntu with ROS 2 installed.
"""
from pathlib import Path
import ast
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent.parent   # tools/ -> repo root
SRC = ROOT / "ros2_ws" / "src"

expected = {
    "warotrans_msgs",
    "warotrans_description",
    "warotrans_hardware",
    "warotrans_sensors",
    "warotrans_localization",
    "warotrans_slam",
    "warotrans_navigation",
    "warotrans_perception",
    "warotrans_fleet",
    "warotrans_simulation",
    "warotrans_bringup",
}

found = {p.name for p in SRC.iterdir() if p.is_dir()}
assert expected == found, f"Package mismatch: expected={expected}, found={found}"

for pkg in sorted(expected):
    px = SRC / pkg / "package.xml"
    assert px.exists(), f"Missing {px}"
    ET.parse(px)

# Contract guard: base_frame must be base_footprint everywhere.
# The realistic failure is in a launch file, not in YAML, so scan both.
import re
BAD_BASE_FRAME = re.compile(r"""base_frame["']?\s*[:=,]\s*["']?base_link""")
for f in list(SRC.rglob("*.py")) + list(SRC.rglob("*.yaml")) + list(SRC.rglob("*.yml")):
    text = f.read_text(encoding="utf-8")
    assert not BAD_BASE_FRAME.search(text), (
        f"TF contract violation in {f}: base_frame must be base_footprint, not base_link"
    )

for py in ROOT.rglob("*.py"):
    if py.name == "validate_structure.py":
        continue
    ast.parse(py.read_text(encoding="utf-8"), filename=str(py))

print("PASS: package tree, XML, and Python syntax are structurally valid.")
