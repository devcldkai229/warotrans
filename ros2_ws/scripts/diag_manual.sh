#!/usr/bin/env bash
set +e
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u

ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM=$ROOT/src/warotrans_simulation
ART=/tmp/nav2_live_artifacts
LOG=/tmp/diag_bringup.log
mkdir -p "$ART"

bash "$ROOT/scripts/kill_sim.sh"

ros2 launch warotrans_simulation sim_bringup.launch.py \
  layout:=$SIM/config/layouts/map_one_v001.yaml \
  map:=$SIM/maps/map_one_v001/map.yaml \
  world:=$SIM/worlds/map_one_v001.sdf \
  spawn_endpoint_code:=EP-PARKING-A \
  nav_runtime:=nav2_semantic \
  enable_authority_gate:=true \
  stage1_open_grant:=true \
  gateway:=false \
  gui:=false \
  >"$LOG" 2>&1 &
LAUNCH_PID=$!
echo LAUNCH_PID=$LAUNCH_PID

for i in $(seq 1 90); do
  if grep -q 'LaneAwareNavfn' "$LOG" && grep -q 'Managed nodes are active' "$LOG"; then
    echo "ACTIVE_$i"
    break
  fi
  sleep 1
done
sleep 15

ros2 topic list >"$ART/topic_list.txt"
grep -E 'costmap|keepout|map$|clock' "$ART/topic_list.txt" | tee "$ART/topics_filtered.txt"

for t in /map /global_costmap/costmap /global_costmap/costmap_raw /keepout_filter_mask; do
  echo "INFO $t" >>"$ART/topic_info.txt"
  ros2 topic info -v "$t" >>"$ART/topic_info.txt" 2>&1 || echo "missing $t" >>"$ART/topic_info.txt"
done

# Sample without TF
python3 "$ROOT/scripts/diag_costmap_lethal.py" >"$ART/diag_summary.txt" 2>"$ART/diag_stderr_small.txt"
# Truncate stderr
head -c 2000 "$ART/diag_stderr_small.txt" >"$ART/diag_stderr_head.txt" || true

timeout 25 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.747, y: 1.543}, orientation: {z: -0.7071, w: 0.7071}}}, planner_id: GridBased, use_start: false}" \
  >"$ART/plan_laneaware.txt" 2>&1

grep -c 'Filter mask was not received' "$LOG" >"$ART/filter_warn.txt" || echo 0 >"$ART/filter_warn.txt"
grep -E 'start/goal lethal|empty path|Created global planner|Filter mask' "$LOG" | tail -40 >"$ART/bringup_grep.txt"

echo DONE
cat "$ART/diag_summary.txt" || true
head -30 "$ART/plan_laneaware.txt" || true
cat "$ART/filter_warn.txt"
# leave sim running for follow-up? kill for cleanliness
kill $LAUNCH_PID 2>/dev/null || true
bash "$ROOT/scripts/kill_sim.sh"
