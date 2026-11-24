#!/usr/bin/env bash
# Start / stop the real-robot stack for Semantic traffic on the Pi (bringup + gateway), logs in ~/wt_logs.
#   ~/warotrans_tools/pi_run_semantic.sh start              BROKER=auto (default): the backend laptop is found by
#                                                           name / ICS address / LAN scan (broker_discovery.py)
#   BROKER=<ip|name> ~/warotrans_tools/pi_run_semantic.sh start   pin it
#   ~/warotrans_tools/pi_run_semantic.sh status | stop
#   MODE=cloud ~/warotrans_tools/pi_run_semantic.sh start   AWS: MQTT over TLS to AWS IoT Core with this robot's thing
#                                                           certificate, evidence to the cloud Recorder. Reads
#                                                           ~/.config/warotrans/cloud.env (docs in tools/cloud.env.example);
#                                                           no broker discovery, no laptop.
# MOTORS defaults to false: odometry live, the ESP32 bridge writes no velocity. MOTORS=true is for a supervised run
# only, with a tested hardware E-stop in reach. BUNDLE selects the map (semantic bundle directory).
# EKF defaults to true since the 2026-11-11 lab (docs/LAB_CALIBRATION_2026-11-11.md): the BNO055 yaw is within ~1 % of
# the scan-match truth on ±90/180 deg turns, wheel yaw is off by up to 25 %. EKF=false falls back to wheel-only AMCL.
# AMCL's motion noise follows it, so the odometry source and what AMCL assumes about it cannot disagree.
set -uo pipefail
ACTION="${1:-status}"
BUNDLE="${BUNDLE:-$HOME/ros2_ws/src/warotrans_navigation/maps/map_one_v022}"
SRC="$HOME/ros2_ws/src"
LOGS="$HOME/wt_logs"
MOTORS="${MOTORS:-false}"
EKF="${EKF:-true}"
# Dual-robot lab: ROBOT_CODE=RBT-002 ROS_DOMAIN_ID=31 (and a second Pi / namespace).
ROBOT_CODE="${ROBOT_CODE:-RBT-001}"
# Evidence (camera recording during trips, uploads to the Recorder on the backend laptop, port 5095). CAMERA=false
# leaves the camera off; the credential WARO_EVIDENCE_TOKEN comes from ~/.config/warotrans/evidence.env (never a repo).
CAMERA="${CAMERA:-true}"
EVIDENCE_ENV="$HOME/.config/warotrans/evidence.env"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
MODE="${MODE:-lan}"
CLOUD_ENV="$HOME/.config/warotrans/cloud.env"

case "$ACTION" in
  start)
    # The laptop's address changes with the network; a name is kept as a name (MQTT and the agent re-resolve it).
    case "$MODE" in
      cloud)
        [[ -f "$CLOUD_ENV" ]] || { echo "MODE=cloud needs $CLOUD_ENV (see tools/cloud.env.example)"; exit 2; }
        set -a; source "$CLOUD_ENV"; set +a
        IOT_DIR="${WARO_IOT_DIR:-$HOME/.config/warotrans/iot}"
        export WARO_MQTT_TLS=true WARO_MQTT_PORT="${WARO_MQTT_PORT:-8883}"
        export WARO_MQTT_CERT_FILE="${WARO_MQTT_CERT_FILE:-$IOT_DIR/$ROBOT_CODE.cert.pem}"
        export WARO_MQTT_KEY_FILE="${WARO_MQTT_KEY_FILE:-$IOT_DIR/$ROBOT_CODE.private.key}"
        export WARO_MQTT_CA_FILE="${WARO_MQTT_CA_FILE:-$IOT_DIR/AmazonRootCA1.pem}"
        for f in "$WARO_MQTT_CERT_FILE" "$WARO_MQTT_KEY_FILE" "$WARO_MQTT_CA_FILE"; do
          [[ -f "$f" ]] || { echo "MODE=cloud: missing $f"; exit 2; }
        done
        [[ -n "${WARO_MQTT_HOST:-}" && -n "${WARO_RECORDER_URL:-}" ]]           || { echo "$CLOUD_ENV must set WARO_MQTT_HOST (IoT ATS endpoint) and WARO_RECORDER_URL"; exit 2; }
        # Command expiry allows 2 s of clock skew: without a synchronised clock every command would be refused.
        if [[ "$(timedatectl show -p NTPSynchronized --value 2>/dev/null)" != "yes" ]]; then
          echo "clock not NTP-synchronised (chrony): run tools/install_chrony.sh --cloud and wait"; exit 2
        fi
        BROKER="$WARO_MQTT_HOST"; RECORDER_URL="$WARO_RECORDER_URL"
        ;;
      lan)
        BROKER=$(BROKER="${BROKER:-auto}" python3 "$HOME/warotrans_tools/broker_discovery.py" resolve --retry-s 30)       || { echo "backend not found (MQTT 1883): start the backend or give BROKER=<ip>"; exit 2; }
        RECORDER_URL="http://$BROKER:5095"
        ;;
      *) echo "MODE must be lan or cloud"; exit 2 ;;
    esac
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
    echo "robot_code=$ROBOT_CODE domain=$ROS_DOMAIN_ID motors_enabled=$MOTORS ekf=$EKF bundle=$BUNDLE mode=$MODE broker=$BROKER"
    setsid nohup ros2 launch warotrans_bringup navigation.launch.py \
      map:="$BUNDLE/map.yaml" lanes_json:="$BUNDLE/lanes.json" \
      extra_params_file:="$SRC/warotrans_navigation/config/nav2_params_lane_aware.yaml;$SRC/warotrans_navigation/config/nav2_params_fleet.yaml;$SRC/warotrans_navigation/config/nav2_params_rbt001.yaml" \
      enable_costmap_filters:=true keepout_mask:="$BUNDLE/keepout.yaml" speed_mask:="$BUNDLE/speed.yaml" \
      enable_authority_gate:=true motors_enabled:="$MOTORS" start_foxglove:=false \
      enable_ekf:="$EKF" amcl_params_file:="$AMCL_PARAMS" \
      > "$LOGS/bringup_${ROBOT_CODE}.log" 2>&1 < /dev/null &
    setsid nohup ros2 launch warotrans_fleet fleet.launch.py \
      params_file:="$SRC/warotrans_fleet/config/robot_gateway_semantic.yaml" \
      semantic_manifest:="$BUNDLE/semantic_manifest.json" mqtt_host:="$BROKER" \
      robot_code:="$ROBOT_CODE" \
      > "$LOGS/gateway_${ROBOT_CODE}.log" 2>&1 < /dev/null &
    if [[ "$CAMERA" == "true" ]]; then
      setsid nohup ros2 run warotrans_camera camera_stream --ros-args         --params-file "$SRC/warotrans_camera/config/camera_rbt001.yaml"         > "$LOGS/camera_${ROBOT_CODE}.log" 2>&1 < /dev/null &
      if [[ -f "$EVIDENCE_ENV" ]]; then
        set -a; source "$EVIDENCE_ENV"; set +a
        setsid nohup ros2 run warotrans_evidence_agent evidence_agent --ros-args           --params-file "$SRC/warotrans_evidence_agent/config/evidence_agent_rbt001.yaml"           -p recorder_url:="$RECORDER_URL" -p robot_code:="$ROBOT_CODE"           -p map_version_code:="$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['mapCode'])" "$BUNDLE/semantic_manifest.json")"           > "$LOGS/evidence_${ROBOT_CODE}.log" 2>&1 < /dev/null &
      else
        echo "no $EVIDENCE_ENV (WARO_EVIDENCE_TOKEN=...): camera live only, no evidence recording"
      fi
    fi
    # Follow the laptop if its address changes mid-session: 3 missed MQTT checks -> resolve again -> stop/start
    # (same environment). The restart runs detached because stop kills this watcher.
    # Cloud: IoT Core has a fixed name, paho reconnects by itself; nothing to follow.
    if [[ "$MODE" == "lan" ]]; then
    mkdir -p "$HOME/.cache/warotrans"; echo "$BROKER" > "$HOME/.cache/warotrans/broker"
    setsid nohup python3 "$HOME/warotrans_tools/broker_discovery.py" watch --state "$HOME/.cache/warotrans/broker"       --on-change "setsid nohup bash -c 'BROKER=auto $0 stop; sleep 3; BROKER=auto $0 start' >> $LOGS/broker_watch_${ROBOT_CODE}.log 2>&1 < /dev/null &"       >> "$LOGS/broker_watch_${ROBOT_CODE}.log" 2>&1 < /dev/null &
    fi
    echo started
    ;;
  stop)
    # Bracketed patterns: an SSH command line that mentions these names must not match (it would kill itself).
    pkill -INT -f 'ros2 launch warotrans[_]' 2>/dev/null
    sleep 6
    pkill -INT -f 'camera_strea[m]|evidence_agen[t]' 2>/dev/null
    pkill -f 'broker_discovery.py watc[h]' 2>/dev/null
    pkill -f 'esp32_bridge_nod[e]|wheel_odom_nod[e]|robot_gateway_nod[e]|cmd_vel_authority_gat[e]|collision_monito[r]|velocity_smoothe[r]|controller_serve[r]|planner_serve[r]|behavior_serve[r]|bt_navigato[r]|waypoint_followe[r]|nav2_amc[l]|/amc[l]|map_serve[r]|lifecycle_manage[r]|costmap_filte[r]|rplida[r]|robot_state_publishe[r]|battery_monitor_nod[e]|recovery_guar[d]|bno055_nod[e]|ekf_nod[e]' 2>/dev/null
    sleep 1
    pgrep -af 'warotrans|nav2|rplidar|amcl' | grep -v pi_run_semantic | grep -v pgrep | cut -c1-100 || true
    echo stopped
    ;;
  status)
    pgrep -af 'ros2 launch|esp32_bridge|robot_gateway|authority_gate|amcl' | grep -v pgrep | cut -c1-110
    ;;
  *) echo "usage: $0 start|stop|status"; exit 2;;
esac
