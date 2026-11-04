"""Ensure vendored paho-mqtt is importable before mqtt_publisher loads."""
from __future__ import annotations

import sys
from pathlib import Path

_vendor = Path(__file__).resolve().parent / "_vendor"
if _vendor.is_dir() and str(_vendor) not in sys.path:
    sys.path.insert(0, str(_vendor))
