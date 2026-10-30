#!/usr/bin/env bash
# Kill every Gazebo / Nav2 / bridge leftover. Must include parameter_bridge (duplicate /clock).
set +e
patterns=(
  parameter_bridge
  'gz sim'
  sim_bringup.launch
  robot_gateway_node
  controller_server
  planner_server
  bt_navigator
  route_server
  behavior_server
  velocity_smoother
  collision_monitor
  cmd_vel_authority
  map_server
  '/amcl'
  lifecycle_manager
  robot_state_publisher
)
for p in "${patterns[@]}"; do
  pkill -9 -f "$p" 2>/dev/null
done
sleep 3
echo "AFTER_KILL:"
ps -eo pid,cmd | grep -E 'parameter_bridge|gz sim|sim_bringup|robot_gateway|controller_server' | grep -v grep || echo CLEAR
