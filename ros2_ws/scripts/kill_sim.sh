#!/usr/bin/env bash
set +e
# Prefer pgrep+kill over pkill -f: pkill can match the kill script/command line itself.
_kill_pat() {
  local pat="$1"
  local pid
  while read -r pid; do
    [ -n "$pid" ] || continue
    kill -9 "$pid" 2>/dev/null
  done < <(pgrep -f "$pat" || true)
}
for pat in \
  'ros2 launch warotrans' sim_bringup 'gz sim' 'ign gazebo' \
  planner_server controller_server bt_navigator 'nav2_amcl' map_server \
  diag_costmap robot_state_publisher parameter_bridge \
  '/warotrans_fleet/'
do
  _kill_pat "$pat"
done
sleep 2
echo "remaining:"
pgrep -af 'gz sim|planner_server|sim_bringup|/warotrans_fleet/' || echo none
exit 0
