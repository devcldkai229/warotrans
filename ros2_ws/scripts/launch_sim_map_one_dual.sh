#!/usr/bin/env bash
# Stage-3: SIM-01 + SIM-02 on MAP-ONE (namespaced Nav2 + semantic traffic grants).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SIM_SRC="${ROOT}/src/warotrans_simulation"
LAYOUT="${SIM_SRC}/config/layouts/map_one_v001.yaml"
MAP="${SIM_SRC}/maps/map_one_v001/map.yaml"
WORLD="${SIM_SRC}/worlds/map_one_v001.sdf"

if [[ ! -f "$WORLD" || ! -f "$MAP" || ! -f "${SIM_SRC}/maps/map_one_v001/lanes.json" ]]; then
  echo "MAP-ONE Stage-3 bundle missing. Run build_map_one_sim_bundle.py first."
  exit 1
fi

exec ros2 launch warotrans_simulation multi_sim_map_one.launch.py \
  layout:="$LAYOUT" \
  map:="$MAP" \
  world:="$WORLD" \
  "$@"
