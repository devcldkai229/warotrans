#!/usr/bin/env bash
# Kill ALL WaroTrans / Nav2 / teleop / ultrasonic / zenoh stacks on Pi.
set -uo pipefail

echo "=== STOP ALL warotrans stacks ==="
sudo systemctl stop warotrans-teleop 2>/dev/null || true
sudo systemctl stop warotrans-demo 2>/dev/null || true

# Soft then hard — never abort on pkill "no process" (exit 1)
_pkill() { pkill "$@" 2>/dev/null || true; }
_killall9() { killall -9 "$@" 2>/dev/null || true; }

_pkill -f 'uvicorn app.main:app'
_pkill -f 'ros2 launch warotrans_'
_pkill -f 'ros2 launch .*navigation'
_pkill -f 'ros2 launch .*lidar'
_pkill -f 'navigation.launch.py'
_pkill -f 'lidar.launch.py'
_pkill -f 'web_teleop_node|esp32_bridge_node|wheel_odom_node|ultrasonic_node'
_pkill -f 'rplidar|slam_toolbox|async_slam|foxglove_bridge'
_pkill -f 'fastdds discovery|fast-discovery'
_pkill -f 'rmw_zenohd|zenohd'
_pkill -f 'ros2 run rmw_zenoh'
_pkill -f 'controller_server|planner_server|behavior_server|bt_navigator'
_pkill -f 'nav2_amcl|/amcl |map_server|lifecycle_manager|costmap_filter'
_pkill -f 'robot_state_publisher|joint_state_publisher'

sleep 1

_killall9 rplidar_composition controller_server planner_server \
  behavior_server bt_navigator map_server amcl lifecycle_manager \
  costmap_filter_info_server esp32_bridge_node wheel_odom_node \
  foxglove_bridge robot_state_publisher rmw_zenohd

# SIGKILL any leftover launch / zenoh by pattern
pkill -9 -f 'ros2 launch warotrans_' 2>/dev/null || true
pkill -9 -f 'navigation.launch.py' 2>/dev/null || true
pkill -9 -f 'lidar.launch.py' 2>/dev/null || true
pkill -9 -f 'rmw_zenohd' 2>/dev/null || true
pkill -9 -f 'ros2 run rmw_zenoh_cpp' 2>/dev/null || true

# Free Zenoh TCP port if still held
ZENOH_PORT="${ZENOH_PORT:-7447}"
if command -v fuser >/dev/null 2>&1; then
  fuser -k "${ZENOH_PORT}/tcp" 2>/dev/null || true
fi
# Fallback: kill by ss/lsof pid
if command -v ss >/dev/null 2>&1; then
  for pid in $(ss -tlnp "( sport = :${ZENOH_PORT} )" 2>/dev/null \
    | sed -n 's/.*pid=\([0-9]*\).*/\1/p' | sort -u); do
    kill -9 "$pid" 2>/dev/null || true
  done
fi

sleep 2
echo "=== Remaining (should be empty of ROS / zenoh) ==="
left="$(pgrep -af 'warotrans_|nav2_|rplidar|slam_toolbox|web_teleop|esp32_bridge|ultrasonic|foxglove|uvicorn|rmw_zenoh|navigation.launch|lidar.launch' \
  | grep -v 'warotrans-stop-all\|pgrep' || true)"
if [[ -n "${left}" ]]; then
  echo "$left"
  echo "WARN: some processes remain — try: pkill -9 -f navigation.launch; fuser -k ${ZENOH_PORT}/tcp"
else
  echo "(clean)"
fi
if ss -tln 2>/dev/null | grep -q ":${ZENOH_PORT} "; then
  echo "WARN: port ${ZENOH_PORT} still LISTEN"
else
  echo "port ${ZENOH_PORT}: free"
fi
echo "STOP_ALL_DONE"
