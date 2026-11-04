#!/usr/bin/env bash
set +e
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM=$ROOT/src/warotrans_simulation
ART=/tmp/nav2_live_artifacts
LOG=$ART/quick_bringup.log
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
  grep -q 'Managed nodes are active' "$LOG" && grep -q 'LaneAwareNavfn' "$LOG" && break
  sleep 1
done
sleep 25
python3 "$ROOT/scripts/dump_costmap_hist.py" | tee "$ART/costmap_hist.txt"
timeout 30 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: -1.02, y: -0.529}, orientation: {z: -0.7071, w: 0.7071}}}, planner_id: GridBased, use_start: false}" \
  | tee "$ART/plan_to_charging.txt"
grep -E 'start/goal|INSCRIBED|LETHAL|NO_INFORMATION|empty path|Filter mask was not' "$LOG" | tail -30 | tee "$ART/quick_grep.txt"
kill $LP 2>/dev/null
bash "$ROOT/scripts/kill_sim.sh"
echo DONE
