#!/usr/bin/env bash
# Single Parking(nudge) -> Inbound NavigateToPose with motion evidence.
set +e
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u
ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM=$ROOT/src/warotrans_simulation
ART=/tmp/nav2_live_artifacts
LOG=$ART/single_leg_bringup.log
mkdir -p "$ART"
bash "$ROOT/scripts/kill_sim.sh"
pkill -9 -f ros2-daemon 2>/dev/null || true
ros2 launch warotrans_simulation sim_bringup.launch.py \
  layout:=$SIM/config/layouts/map_one_v001.yaml \
  map:=$SIM/maps/map_one_v001/map.yaml \
  world:=$SIM/worlds/map_one_v001.sdf \
  spawn_endpoint_code:=EP-INBOUND \
  nav_runtime:=nav2_semantic \
  enable_authority_gate:=true \
  stage1_open_grant:=true \
  gateway:=false gui:=false >"$LOG" 2>&1 &
LP=$!
for i in $(seq 1 120); do
  grep -q 'Managed nodes are active' "$LOG" && grep -q 'LaneAwareNavfn' "$LOG" && break
  sleep 1
done
# Wait until filter masks are actually received (not just lifecycle ACTIVE).
for i in $(seq 1 60); do
  WARN=$(grep -c 'Filter mask was not received' "$LOG" 2>/dev/null || true)
  OK=$(grep -c 'Filter mask received\|KeepoutFilter: Subscribing' "$LOG" 2>/dev/null || true)
  # Prefer: no new mask warnings in last few seconds — heuristic: total warn stopped growing after warm-up
  sleep 2
  WARN2=$(grep -c 'Filter mask was not received' "$LOG" 2>/dev/null || true)
  if [ "$WARN2" = "$WARN" ] && [ "$i" -gt 5 ]; then
    echo "MASK_STABLE_$i warn=$WARN2"
    break
  fi
done
grep 'spawn=' "$LOG" | head -2
# WAIT-POINT -> INBOUND (open free-space start; parking footprint still hostile)
timeout 180 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.292, y: 0.83, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}" \
  | tee "$ART/nav_single_inbound.txt"
grep -E 'empty path|LETHAL|INSCRIBED|approach|BLOCKING|progress' "$LOG" | tail -40 | tee "$ART/single_leg_grep.txt"
kill "$LP" 2>/dev/null
bash "$ROOT/scripts/kill_sim.sh"
echo DONE
