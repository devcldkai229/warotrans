#!/usr/bin/env bash
# Autonomous SIM-01 planning matrix + motion tour (Nav2 semantic, no RouteGraph).
set +e
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u

ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM=$ROOT/src/warotrans_simulation
AUD=/mnt/e/programming/my_project/warotrans-system/docs/audits/autonomous_gazebo_acceptance
ART=$AUD/runtime
EP_JSON=$SIM/config/map_one_v001_endpoints.json
LOG=$ART/orchestrator_bringup.log
MATRIX=$AUD/ENDPOINT_MATRIX.csv
MOTION=$AUD/MOTION_RESULTS.csv
mkdir -p "$ART"

bash "$ROOT/scripts/kill_sim.sh"
pkill -9 -f ros2-daemon 2>/dev/null || true

ros2 launch warotrans_simulation sim_bringup.launch.py \
  layout:=$SIM/config/layouts/map_one_v001.yaml \
  map:=$SIM/maps/map_one_v001/map.yaml \
  world:=$SIM/worlds/map_one_v001.sdf \
  spawn_endpoint_code:=EP-OUTBOUND \
  nav_runtime:=nav2_semantic \
  enable_authority_gate:=true \
  stage1_open_grant:=true \
  gateway:=false gui:=false >"$LOG" 2>&1 &
LP=$!
cleanup() { kill "$LP" 2>/dev/null; bash "$ROOT/scripts/kill_sim.sh"; }
trap cleanup EXIT

for i in $(seq 1 120); do
  grep -q 'Managed nodes are active' "$LOG" && grep -q 'LaneAwareNavfn' "$LOG" && break
  sleep 1
done
for i in $(seq 1 40); do
  W1=$(grep -c 'Filter mask was not received' "$LOG" 2>/dev/null || true)
  sleep 2
  W2=$(grep -c 'Filter mask was not received' "$LOG" 2>/dev/null || true)
  if [ "$W1" = "$W2" ] && [ "$i" -gt 3 ]; then echo "MASK_STABLE warn=$W2"; break; fi
done
sleep 5

# Recompute matrix when ONE_WAY semantics change (FORCE_MATRIX=1) or missing.
if [ "${FORCE_MATRIX:-0}" = "1" ] || [ ! -f "$MATRIX" ]; then
  python3 "$ROOT/scripts/run_endpoint_plan_matrix.py" \
    --endpoints "$EP_JSON" \
    --out "$MATRIX" \
    --timeout 20 | tee "$ART/plan_matrix_summary.txt"
else
  echo "SKIP_MATRIX existing $MATRIX"
fi

python3 "$ROOT/scripts/run_sim01_motion_tour.py" \
  --endpoints "$EP_JSON" \
  --out "$MOTION" \
  --timeout 240 \
  --codes EP-WAIT-POINT,EP-INBOUND,EP-OUTBOUND,EP-SHELF-C-01,EP-OUTBOUND,EP-SHELF-B-01,EP-OUTBOUND,EP-SHELF-A-01,EP-OUTBOUND,EP-PARKING-A,EP-CHARGING-A,EP-PARKING-B,EP-OUTBOUND,EP-INSPECTION,EP-SHELF-C-02,EP-OUTBOUND \
  | tee "$ART/motion_tour_summary.txt"

# SPEED zone velocity evidence (OUTBOUND→WAIT crosses mask≈30).
python3 "$ROOT/scripts/run_speed_zone_velocity_probe.py" \
  --out "$AUD/SPEED_VELOCITY_SAMPLES.csv" \
  --timeout 180 | tee "$ART/speed_probe_summary.txt"

echo ORCH_DONE
