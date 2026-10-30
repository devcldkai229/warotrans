#!/usr/bin/env bash
set -eo pipefail
LOGDIR=/mnt/e/programming/my_project/warotrans/docs/simulation/traffic-hardening/logs
mkdir -p "$LOGDIR"
# Kill by process group / name aggressively
for pat in parameter_bridge 'sim_bringup.launch' 'robot_gateway_node' 'gz sim' 'ruby .*/gz' \
           controller_server planner_server bt_navigator route_server \
           behavior_server velocity_smoother collision_monitor \
           cmd_vel_authority map_server amcl lifecycle_manager \
           robot_state_publisher spawner; do
  pkill -9 -f "$pat" 2>/dev/null || true
done
sleep 5
# Remaining?
left=$(ps -eo pid,cmd | grep -E 'parameter_bridge|sim_bringup|gz sim |robot_gateway|controller_server' | grep -v grep || true)
if [ -n "$left" ]; then
  echo "STILL_ALIVE:"
  echo "$left"
  exit 1
fi
rm -f /tmp/warotrans_authority_state.json
: >"$LOGDIR/sim_active.log"
nohup bash /mnt/e/programming/my_project/warotrans/ros2_ws/scripts/_launch_sim_active_headless.sh \
  >"$LOGDIR/launcher.out" 2>&1 &
echo $! >"$LOGDIR/sim.pid"
echo "hard_reset_ok pid=$(cat "$LOGDIR/sim.pid")"
