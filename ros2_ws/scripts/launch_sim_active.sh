#!/usr/bin/env bash
# Gazebo + Nav2 lane-graph + Active traffic gateway for MAP-WHSIM-V019 (WSL). GUI=false for headless.
# Robot spawns at EP-PARK-1. Graph file must be the backend export of the same map (graphHash checked).
set -eo pipefail
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH PYTHONPATH
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
export PYTHONPATH="/home/ser/pydeps${PYTHONPATH:+:$PYTHONPATH}"
SIM_SRC=/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation
GRAPH=/home/ser/maps/route_graph_v019.geojson
mkdir -p /tmp/waro-sim-logs
rm -f /tmp/warotrans_authority_state.json
python3 "$SIM_SRC/scripts/sim_init_gate.py" \
  --layout "$SIM_SRC/config/warehouse_layout.yaml" \
  --map "$SIM_SRC/maps/sim_warehouse.yaml" \
  --expected-map-version MAP-WHSIM-V019 \
  --skip-live \
  --authority-state /tmp/warotrans_authority_state.json
exec ros2 launch warotrans_simulation sim_bringup.launch.py \
  gui:="${GUI:-true}" \
  rviz:=false \
  sim_speeds:=true \
  enable_authority_gate:=true \
  spawn_endpoint_code:=EP-PARK-1 \
  map_version_code:=MAP-WHSIM-V019 \
  route_graph:=$GRAPH \
  gateway_params_file:=$SIM_SRC/config/robot_gateway_sim_active_overlay.yaml
