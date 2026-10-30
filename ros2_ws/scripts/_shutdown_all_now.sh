#!/bin/bash
pkill -9 -f sim_bringup || true
pkill -9 -f robot_gateway || true
pkill -9 -f 'gz sim' || true
pkill -9 -f gazebo || true
pkill -9 -f nav2_ || true
pkill -9 -f '/amcl' || true
pkill -9 -f map_server || true
pkill -9 -f bt_navigator || true
pkill -9 -f controller_server || true
pkill -9 -f planner_server || true
pkill -9 -f route_server || true
pkill -9 -f lifecycle_manager || true
pkill -9 -f velocity_smoother || true
pkill -9 -f collision_monitor || true
pkill -9 -f behavior_server || true
pkill -9 -f 'ruby /opt/ros' || true
sleep 2
LEFT=$(pgrep -af 'sim_bringup|robot_gateway|gz sim|nav2_|/amcl' | grep -v pgrep | wc -l)
echo COUNT=$LEFT
pgrep -af 'sim_bringup|robot_gateway|gz sim' | grep -v pgrep || echo WSL_SIM_CLEAR
