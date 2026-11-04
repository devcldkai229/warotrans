#!/usr/bin/env bash
set +e
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM=$ROOT/src/warotrans_simulation
AUD=/mnt/e/programming/my_project/warotrans-system/docs/audits/autonomous_gazebo_acceptance/runtime
mkdir -p "$AUD"
bash "$ROOT/scripts/kill_sim.sh"
pkill -9 -f ros2-daemon 2>/dev/null || true
LOG=$AUD/diag_wait_bringup.log
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
# Confirm BT + reversing params
ros2 param get /bt_navigator default_nav_to_pose_bt_xml | tee "$AUD/diag_bt.txt"
ros2 param get /controller_server FollowPath.allow_reversing | tee -a "$AUD/diag_bt.txt"
ros2 param get /collision_monitor FootprintApproach.time_before_collision | tee -a "$AUD/diag_bt.txt"
# Single WAIT goal + alternate via C-01 then WAIT
python3 "$ROOT/scripts/run_sim01_motion_tour.py" \
  --endpoints "$SIM/config/map_one_v001_endpoints.json" \
  --out "$AUD/diag_wait_motion.csv" \
  --timeout 180 \
  --codes EP-WAIT-POINT,EP-OUTBOUND,EP-SHELF-C-01,EP-WAIT-POINT,EP-INBOUND \
  | tee "$AUD/diag_wait_summary.txt"
echo DIAG_WAIT_DONE
grep -c 'ONE_WAY_VIOLATION' "$LOG" || true
grep -c 'Failed to make progress' "$LOG" || true
grep -c 'Goal succeeded' "$LOG" || true
