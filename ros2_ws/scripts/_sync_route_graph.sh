#!/usr/bin/env bash
set -euo pipefail
cp /mnt/e/programming/my_project/warotrans/ros2_ws/route_graph_v019.geojson /home/ser/maps/route_graph_v019.geojson
python3 - <<'PY'
import json
d = json.load(open("/home/ser/maps/route_graph_v019.geojson"))
print("hash", d.get("properties", {}).get("graphHash"))
print("features", len(d.get("features", [])))
PY
