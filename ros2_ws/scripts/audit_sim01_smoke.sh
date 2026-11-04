#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u

ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM_SRC="$ROOT/src/warotrans_simulation"
LOG=/tmp/sim01_fix.log

echo "=== rclpy pytest ==="
export PYTHONPATH="$ROOT/src/warotrans_fleet:$SIM_SRC/scripts:${PYTHONPATH:-}"
python3 -c 'import rclpy; print("rclpy", rclpy.__file__)'
python3 -m pytest \
  "$ROOT/src/warotrans_fleet/test/test_path_policy_validator.py" \
  "$ROOT/src/warotrans_simulation/test/test_lane_aware_plugin_id.py" \
  -q --tb=short

echo "=== plugin strings ==="
PREF="$(ros2 pkg prefix warotrans_lane_planner)"
NPREF="$(ros2 pkg prefix warotrans_navigation)"
grep type= "$PREF/share/warotrans_lane_planner/lane_aware_navfn_plugin.xml"
grep 'plugin:' "$NPREF/share/warotrans_navigation/config/nav2_params_lane_aware.yaml"

echo "=== sim_bringup 75s ==="
rm -f "$LOG"
set +e
timeout 75 ros2 launch warotrans_simulation sim_bringup.launch.py \
  layout:="$SIM_SRC/config/layouts/map_one_v001.yaml" \
  map:="$SIM_SRC/maps/map_one_v001/map.yaml" \
  world:="$SIM_SRC/worlds/map_one_v001.sdf" \
  spawn_endpoint_code:=EP-PARKING-A \
  nav_runtime:=nav2_semantic \
  enable_authority_gate:=true \
  stage1_open_grant:=true \
  gateway:=false \
  gui:=false \
  >"$LOG" 2>&1
echo "LAUNCH_EXIT:$?"
set -e

grep -E 'LaneAware|path_policy|Failed to create|FATAL|Managed nodes are active|STAGE1_OPEN_GRANT|Created global planner|GridBased|process has died|ERROR|Activating planner|Configuring planner' "$LOG" | head -80
echo "=== lifecycle grep ==="
grep -E 'lifecycle_manager_navigation|planner_server|path_policy_validator|cmd_vel_authority' "$LOG" | tail -40
