#!/usr/bin/env bash
set -eo pipefail
unset PYTHONPATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
export PYTHONPATH="/home/ser/pydeps${PYTHONPATH:+:$PYTHONPATH}"
OUT=/mnt/e/programming/my_project/warotrans/docs/simulation/traffic-hardening/phase_b_params.txt
{
  echo "timestamp=$(date -Iseconds)"
  echo "merge_order=nav2_params -> outdoor -> route -> sim (from controller cmdline)"
  ros2 param get /controller_server FollowPath.lookahead_dist || true
  ros2 param get /controller_server FollowPath.min_lookahead_dist || true
  ros2 param get /controller_server FollowPath.max_lookahead_dist || true
  ros2 param get /controller_server FollowPath.curvature_lookahead_dist || true
  ros2 param get /controller_server FollowPath.desired_linear_vel || true
  ros2 param get /controller_server FollowPath.use_fixed_curvature_lookahead || true
  ros2 param get /controller_server general_goal_checker.xy_goal_tolerance || true
  ros2 param get /controller_server general_goal_checker.yaw_goal_tolerance || true
  echo "--- MQTT / robot ---"
  grep -E "MQTT connected|MQTT disconnected|LOCALIZED|graphHash" \
    /mnt/e/programming/my_project/warotrans/docs/simulation/traffic-hardening/logs/sim_active.log | tail -20 || true
} | tee "$OUT"
