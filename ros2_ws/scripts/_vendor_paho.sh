#!/usr/bin/env bash
# Vendor paho-mqtt into warotrans_fleet so robot_gateway works without system pip.
set -euo pipefail
ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
DEST=$ROOT/src/warotrans_fleet/warotrans_fleet/_vendo
mkdir -p "$DEST" /tmp/paho_wheel
cd /tmp/paho_wheel
rm -rf ./*

curl -fsSL https://pypi.org/pypi/paho-mqtt/json -o pypi.json
URL=$(python3 - <<'PY'
import json
from pathlib import Path
data = json.loads(Path("pypi.json").read_text(encoding="utf-8"))
for ver, files in sorted(data["releases"].items(), reverse=True):
    for f in files:
        if f["packagetype"] == "bdist_wheel" and "py3" in f["python_version"] and f["filename"].endswith("none-any.whl"):
            print(f["url"])
            raise SystemExit(0)
raise SystemExit("no wheel found")
PY
)
echo "WHEEL_URL=$URL"
curl -fsSL -o paho.whl "$URL"
python3 - <<'PY'
import zipfile
from pathlib import Path
dest = Path("/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_fleet/warotrans_fleet/_vendor")
dest.mkdir(parents=True, exist_ok=True)
with zipfile.ZipFile("paho.whl") as z:
    names = z.namelist()
    z.extractall(dest)
print("extracted_top", sorted({Path(n).parts[0] for n in names if n and not n.startswith(".")}))
PY

BOOT=$ROOT/src/warotrans_fleet/warotrans_fleet/_vendor_path.py
cat > "$BOOT" <<'PY'
"""Ensure vendored paho-mqtt is importable before mqtt_publisher loads."""
from __future__ import annotations

import sys
from pathlib import Path

_vendor = Path(__file__).resolve().parent / "_vendor"
if _vendor.is_dir() and str(_vendor) not in sys.path:
    sys.path.insert(0, str(_vendor))
PY

PUB=$ROOT/src/warotrans_fleet/warotrans_fleet/mqtt_publisher.py
if ! grep -q '_vendor_path' "$PUB"; then
  # Insert after module docstring / first line
  python3 - <<'PY'
from pathlib import Path
p = Path("/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_fleet/warotrans_fleet/mqtt_publisher.py")
t = p.read_text(encoding="utf-8")
needle = "from warotrans_fleet import _vendor_path"
if needle not in t:
    lines = t.splitlines(keepends=True)
    # after docstring if present
    i = 0
    if lines and lines[0].startswith('"""'):
        i = 1
        while i < len(lines) and '"""' not in lines[i]:
            i += 1
        i += 1
    lines.insert(i, "from warotrans_fleet import _vendor_path  # noqa: F401\n")
    p.write_text("".join(lines), encoding="utf-8")
print("patched mqtt_publisher")
PY
fi

python3 - <<'PY'
import sys
sys.path.insert(0, "/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_fleet")
from warotrans_fleet import _vendor_path  # noqa: F401
import paho.mqtt.client as c
print("PAHO_VENDORED_OK", c.__file__)
PY

# Rebuild fleet so install tree picks up vendo
source /opt/ros/jazzy/setup.bash
cd "$ROOT"
colcon build --packages-select warotrans_fleet --symlink-install 2>&1 | tail -20
