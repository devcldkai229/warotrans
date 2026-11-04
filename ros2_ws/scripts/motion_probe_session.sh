#!/usr/bin/env bash
# Short bringup + cmd_vel pipeline probe after STAGE1 open-grant wall-clock fix.
set +e
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u
ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM=$ROOT/src/warotrans_simulation
ART=/tmp/nav2_live_artifacts
LOG=$ART/motion_probe_bringup.log
mkdir -p "$ART"
bash "$ROOT/scripts/kill_sim.sh"
pkill -9 -f ros2-daemon 2>/dev/null || true
ros2 launch warotrans_simulation sim_bringup.launch.py \
  layout:=$SIM/config/layouts/map_one_v001.yaml \
  map:=$SIM/maps/map_one_v001/map.yaml \
  world:=$SIM/worlds/map_one_v001.sdf \
  spawn_endpoint_code:=EP-PARKING-A \
  nav_runtime:=nav2_semantic \
  enable_authority_gate:=true \
  stage1_open_grant:=true \
  gateway:=false gui:=false >"$LOG" 2>&1 &
LP=$!
for i in $(seq 1 100); do
  grep -q 'Managed nodes are active' "$LOG" && grep -q 'STAGE1_OPEN_GRANT' "$LOG" && break
  sleep 1
done
sleep 15
grep -E 'STAGE1_OPEN_GRANT|authority gate BLOCKING|wall-clock' "$LOG" | tail -20 | tee "$ART/motion_probe_grep.txt"
python3 "$ROOT/scripts/motion_cmd_vel_probe.py" | tee "$ART/motion_cmd_vel_probe_out.txt"
# also one short nav to charging
timeout 90 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -1.02, y: -0.529, z: 0.0}, orientation: {z: -0.7071, w: 0.7071}}}}" \
  | tee "$ART/nav_to_charging_short.txt"
kill "$LP" 2>/dev/null
bash "$ROOT/scripts/kill_sim.sh"
echo DONE
