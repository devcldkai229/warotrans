#!/usr/bin/env bash
# Stage-2: SIM-01 Nav2 semantic policy on MAP-ONE-V001 (LaneAware + keepout/speed + validator).
# Prerequisites: build_map_one_sim_bundle.py already run; colcon build; Gazebo/ROS Jazzy.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SIM_SRC="${ROOT}/src/warotrans_simulation"
LAYOUT="${SIM_SRC}/config/layouts/map_one_v001.yaml"
MAP="${SIM_SRC}/maps/map_one_v001/map.yaml"
WORLD="${SIM_SRC}/worlds/map_one_v001.sdf"
GATEWAY="${SIM_SRC}/config/robot_gateway_sim_semantic.yaml"
KEEPOUT="${SIM_SRC}/maps/map_one_v001/keepout.yaml"
SPEED="${SIM_SRC}/maps/map_one_v001/speed.yaml"
LANES="${SIM_SRC}/maps/map_one_v001/lanes.json"

if [[ ! -f "$WORLD" || ! -f "$MAP" ]]; then
  echo "MAP-ONE bundle missing. Run:"
  echo "  python3 ${SIM_SRC}/scripts/build_map_one_sim_bundle.py --snapshot <path>/MAP-ONE-V001"
  exit 1
fi
if [[ ! -f "$KEEPOUT" || ! -f "$SPEED" || ! -f "$LANES" ]]; then
  echo "Stage-2 policy artifacts missing (keepout/speed/lanes). Rebuild the MAP-ONE bundle."
  exit 1
fi

exec ros2 launch warotrans_simulation sim_bringup.launch.py \
  layout:="$LAYOUT" \
  map:="$MAP" \
  world:="$WORLD" \
  spawn_endpoint_code:=EP-PARKING-A \
  nav_runtime:=nav2_semantic \
  route_graph:= \
  enable_authority_gate:=true \
  stage1_open_grant:=true \
  gateway:=true \
  gateway_params_file:="$GATEWAY" \
  map_version_code:=MAP-ONE-V001 \
  robot_code:=RBT-SIM-01 \
  "$@"
