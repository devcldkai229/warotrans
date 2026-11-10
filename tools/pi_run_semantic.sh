#!/usr/bin/env bash
# Start / stop the real-robot stack for Semantic traffic on the Pi (bringup + gateway), logs in ~/wt_logs.
#   BROKER=192.168.137.1 ~/warotrans_tools/pi_run_semantic.sh start
#   ~/warotrans_tools/pi_run_semantic.sh status | stop
# MOTORS defaults to false: odometry live, the ESP32 bridge writes no velocity. MOTORS=true is for a supervised run
# only, with a tested hardware E-stop in reach. BUNDLE selects the map (semantic bundle directory).
# EKF defaults to false until the BNO055 passes the turn test in docs/NAV2_STUCK_RECOVERY_EVIDENCE.md; AMCL's motion
# noise follows it, so the odometry source and what AMCL assumes about it cannot disagree.
set -uo pipefail
ACTION="${1:-status}"
BUNDLE="${BUNDLE:-$HOME/ros2_ws/src/warotrans_navigation/maps/map_one_v004}"
SRC="$HOME/ros2_ws/src"
LOGS="$HOME/wt_logs"
MOTORS="${MOTORS:-false}"
EKF="${EKF:-false}"
# Dual-robot lab: ROBOT_CODE=RBT-002 ROS_DOMAIN_ID=31 (and a second Pi / namespace).
ROBOT_CODE="${ROBOT_CODE:-RBT-001}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

case "$ACTION" in
  start)
    [[ -n "${BROKER:-}" ]] || { echo "BROKER=<backend IP> is required"; exit 2; }
    [[ -f "$BUNDLE/semantic_manifest.json" ]] || { echo "no bundle at $BUNDLE"; exit 2; }
    if [[ "$(systemctl is-active warotrans-teleop 2>/dev/null)" == "active" ]]; then
      echo "warotrans-teleop is running (owns the ESP32 port, drives /cmd_vel): sudo systemctl stop warotrans-teleop"
      exit 2
    fi
    if pgrep -f 'warotrans_bringup navigation.launch.py' >/dev/null; then echo "already running"; exit 2; fi
    mkdir -p "$LOGS"
    set +u; source /opt/ros/jazzy/setup.bash; source "$HOME/ros2_ws/install/setup.bash"; set -u
    case "$EKF" in
      true) AMCL_PARAMS="$SRC/warotrans_localization/config/amcl.yaml" ;;
      false) AMCL_PARAMS="$SRC/warotrans_localization/config/amcl_wheel_only.yaml" ;;
      *) echo "EKF must be true or false"; exit 2 ;;
    esac
    echo "robot_code=$ROBOT_CODE domain=$ROS_DOMAIN_ID motors_enabled=$MOTORS ekf=$EKF bundle=$BUNDLE broker=$BROKER"
    setsid nohup ros2 launch warotrans_bringup navigation.launch.py \
      map:="$BUNDLE/map.yaml" lanes_json:="$BUNDLE/lanes.json" \
      extra_params_file:="$SRC/warotrans_navigation/config/nav2_params_lane_aware.yaml" \
      enable_costmap_filters:=true keepout_mask:="$BUNDLE/keepout.yaml" speed_mask:="$BUNDLE/speed.yaml" \
      enable_authority_gate:=true motors_enabled:="$MOTORS" start_foxglove:=false \
      enable_ekf:="$EKF" amcl_params_file:="$AMCL_PARAMS" \
      > "$LOGS/bringup_${ROBOT_CODE}.log" 2>&1 < /dev/null &
    setsid nohup ros2 launch warotrans_fleet fleet.launch.py \
      params_file:="$SRC/warotrans_fleet/config/robot_gateway_semantic.yaml" \
      semantic_manifest:="$BUNDLE/semantic_manifest.json" mqtt_host:="$BROKER" \
      robot_code:="$ROBOT_CODE" \
      > "$LOGS/gateway_${ROBOT_CODE}.log" 2>&1 < /dev/null &
    echo started
    ;;
  stop)
    pkill -INT -f 'ros2 launch warotrans_' 2>/dev/null
    sleep 6
    pkill -f 'esp32_bridge_node|wheel_odom_node|robot_gateway_node|cmd_vel_authority_gate|collision_monitor|velocity_smoother|controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|nav2_amcl|/amcl|map_server|lifecycle_manager|costmap_filter|rplidar|robot_state_publisher' 2>/dev/null
    sleep 1
    pgrep -af 'warotrans|nav2|rplidar|amcl' | grep -v pi_run_semantic | grep -v pgrep | cut -c1-100 || true
    echo stopped
    ;;
  status)
    pgrep -af 'ros2 launch|esp32_bridge|robot_gateway|authority_gate|amcl' | grep -v pgrep | cut -c1-110
    ;;
  *) echo "usage: $0 start|stop|status"; exit 2;;
esac
