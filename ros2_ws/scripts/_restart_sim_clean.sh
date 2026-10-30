#!/usr/bin/env bash
set -eo pipefail
LOGDIR=/mnt/e/programming/my_project/warotrans/docs/simulation/traffic-hardening/logs
mkdir -p "$LOGDIR"
# Tear down every leftover sim/Nav2/gateway process from prior runs.
pkill -f 'sim_bringup.launch.py' 2>/dev/null || true
pkill -f 'robot_gateway_node' 2>/dev/null || true
pkill -f 'parameter_bridge' 2>/dev/null || true
pkill -f 'ros_gz_bridge' 2>/dev/null || true
pkill -f 'robot_state_publisher' 2>/dev/null || true
pkill -f 'gz sim' 2>/dev/null || true
pkill -f 'ruby.*gz' 2>/dev/null || true
pkill -f 'controller_server' 2>/dev/null || true
pkill -f 'planner_server' 2>/dev/null || true
pkill -f 'bt_navigator' 2>/dev/null || true
pkill -f 'route_server' 2>/dev/null || true
pkill -f 'amcl' 2>/dev/null || true
pkill -f 'map_server' 2>/dev/null || true
pkill -f 'lifecycle_manager' 2>/dev/null || true
pkill -f 'collision_monitor' 2>/dev/null || true
pkill -f 'cmd_vel_authority_gate' 2>/dev/null || true
pkill -f 'velocity_smoother' 2>/dev/null || true
sleep 5
# Ensure no stale bridges remain (duplicate /clock+/tf publishers cause TF jumps).
pkill -9 -f 'parameter_bridge' 2>/dev/null || true
pkill -9 -f 'gz sim' 2>/dev/null || true
sleep 2
rm -f /tmp/warotrans_authority_state.json
: >"$LOGDIR/sim_active.log"
nohup bash /mnt/e/programming/my_project/warotrans/ros2_ws/scripts/_launch_sim_active_headless.sh \
  >"$LOGDIR/launcher.out" 2>&1 &
echo $! >"$LOGDIR/sim.pid"
echo "restarted pid=$(cat "$LOGDIR/sim.pid")"
exit 0
