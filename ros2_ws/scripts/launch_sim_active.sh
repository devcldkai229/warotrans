#!/usr/bin/env bash
# Gazebo + Nav2 lane-graph + Active traffic gateway (WSL).
# Defaults: MAP-WH42CFE65F-V003, spawn EP-PARKING-A---5. Override via env.
# Graph file must be the backend export of the same map (graphHash checked).
set -eo pipefail
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
export PYTHONPATH="/home/ser/pydeps${PYTHONPATH:+:$PYTHONPATH}"

SIM_SRC=/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation
MAP_VERSION_ID="${MAP_VERSION_ID:-1143c8a3-ae62-4660-b684-bc462450f30f}"
MAP_VERSION_CODE="${MAP_VERSION_CODE:-MAP-WH42CFE65F-V003}"
SPAWN_ENDPOINT="${SPAWN_ENDPOINT:-EP-PARKING-A---5}"
GRAPH_PATH="${GRAPH_PATH:-/home/ser/maps/route_graph_wh42cfe65f_v003.geojson}"
BACKEND_URL="${BACKEND_URL:-http://127.0.0.1:5090}"
GATEWAY_OVERLAY="${GATEWAY_OVERLAY:-$SIM_SRC/config/robot_gateway_sim_active_overlay.yaml}"

mkdir -p /tmp/waro-sim-logs "$(dirname "$GRAPH_PATH")"
rm -f /tmp/warotrans_authority_state.json

# Refresh route graph from backend (no graphHash bypass).
bash "$SIM_SRC/scripts/fetch_route_graph.sh" \
  "$BACKEND_URL" "$MAP_VERSION_ID" "$GRAPH_PATH"

# Keep gateway overlay path + map code aligned with this launch.
python3 - <<PY
from pathlib import Path
import re
p = Path(r"""$GATEWAY_OVERLAY""")
text = p.read_text(encoding="utf-8")
text = re.sub(r'(map_version_code:\s*")[^"]*(")', r'\1'"$MAP_VERSION_CODE"r'\2', text, count=1)
text = re.sub(r'(route_graph:\s*")[^"]*(")', r'\1'"$GRAPH_PATH"r'\2', text, count=1)
p.write_text(text, encoding="utf-8")
print(f"gateway overlay -> map_version_code={'$MAP_VERSION_CODE'} route_graph={'$GRAPH_PATH'}")
PY

python3 "$SIM_SRC/scripts/sim_init_gate.py" \
  --layout "$SIM_SRC/config/warehouse_layout.yaml" \
  --map "$SIM_SRC/maps/sim_warehouse.yaml" \
  --expected-map-version "$MAP_VERSION_CODE" \
  --skip-live \
  --authority-state /tmp/warotrans_authority_state.json

exec ros2 launch warotrans_simulation sim_bringup.launch.py \
  gui:="${GUI:-true}" \
  rviz:=false \
  sim_speeds:=true \
  enable_authority_gate:=true \
  spawn_endpoint_code:="$SPAWN_ENDPOINT" \
  map_version_code:="$MAP_VERSION_CODE" \
  route_graph:="$GRAPH_PATH" \
  gateway_params_file:="$GATEWAY_OVERLAY"
