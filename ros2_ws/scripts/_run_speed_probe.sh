#!/usr/bin/env bash
set +e
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM=$ROOT/src/warotrans_simulation
AUD=/mnt/e/programming/my_project/warotrans-system/docs/audits/autonomous_gazebo_acceptance
bash "$ROOT/scripts/kill_sim.sh"
pkill -9 -f ros2-daemon 2>/dev/null || true
LOG=$AUD/runtime/speed_probe_bringup.log
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
sleep 8
python3 "$ROOT/scripts/run_speed_zone_velocity_probe.py" \
  --out "$AUD/SPEED_VELOCITY_SAMPLES.csv" \
  --timeout 180 | tee "$AUD/runtime/speed_probe_summary.txt"
echo SPEED_PROBE_DONE
