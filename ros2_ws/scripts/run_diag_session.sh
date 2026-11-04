#!/usr/bin/env bash
# Long-lived sim + costmap dump + dual planner probe.
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u

ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM=$ROOT/src/warotrans_simulation
LOG=/tmp/diag_bringup.log
ART=/tmp/nav2_live_artifacts
mkdir -p "$ART"

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  pkill -f 'ros2 launch warotrans_simulation sim_bringup' 2>/dev/null || true
}
trap cleanup EXIT

rm -f "$LOG"
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

echo "waiting bringup..."
for i in $(seq 1 120); do
  if grep -q 'Created global planner plugin GridBased of type warotrans_lane_planner::LaneAwareNavfn' "$LOG" \
     && grep -q 'lifecycle_manager_navigation.*Managed nodes are active' "$LOG"; then
    echo "ACTIVE at ${i}s"
    break
  fi
  sleep 1
done
# Allow costmap/filters to settle
sleep 15
FILTER_WARN=$(grep -c 'Filter mask was not received' "$LOG" || true)
echo "filter_mask_warn_count=$FILTER_WARN" | tee "$ART/filter_warn.txt"

python3 "$ROOT/scripts/diag_costmap_lethal.py" 2>"$ART/diag_stderr.txt" | tee "$ART/diag_summary.txt"

# LaneAware plan from robot (use_start false)
timeout 25 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.747, y: 1.543, z: 0.0}, orientation: {z: -0.70710678118, w: 0.70710678118}}}, planner_id: GridBased, use_start: false}" \
  | tee "$ART/plan_laneaware_parking_to_inbound.txt" || true

grep -E 'start/goal lethal|Filter mask|LaneAware|Created global planner|empty path' "$LOG" | tail -40 | tee "$ART/bringup_grep.txt"
echo DONE
