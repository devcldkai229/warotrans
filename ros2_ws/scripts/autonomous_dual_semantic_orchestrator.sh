#!/usr/bin/env bash
# Stage-3 dual SIM-01/SIM-02: bringup (no open-grant) + standalone NavigateToPose scenarios.
set +e
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u

ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
AUD=/mnt/e/programming/my_project/warotrans-system/docs/audits/autonomous_gazebo_acceptance
ART=$AUD/runtime
EP_JSON=$ROOT/src/warotrans_simulation/config/map_one_v001_endpoints.json
LOG=$ART/dual_orch_bringup.log
OUT=$AUD/DUAL_SCENARIO_RUN.md
CSV=$AUD/DUAL_MOTION_RESULTS.csv
# Keep DUAL_ROBOT_RESULTS.md as the curated audit narrative (not overwritten by the runner).
mkdir -p "$ART"

bash "$ROOT/scripts/kill_sim.sh"
pkill -9 -f ros2-daemon 2>/dev/null || true
sleep 2

bash "$ROOT/scripts/launch_sim_map_one_dual.sh" gui:=false >"$LOG" 2>&1 &
LP=$!
cleanup() { kill "$LP" 2>/dev/null; bash "$ROOT/scripts/kill_sim.sh"; }
trap cleanup EXIT

echo "Waiting dual Nav2..."
READY=0
for i in $(seq 1 180); do
  A1=$(grep -c 'sim01.*Managed nodes are active\|\[sim01\].*Managed nodes are active' "$LOG" 2>/dev/null || true)
  # lifecycle logs often lack ns prefix — count total activations
  ACT=$(grep -c 'Managed nodes are active' "$LOG" 2>/dev/null || true)
  LA=$(grep -c 'LaneAwareNavfn' "$LOG" 2>/dev/null || true)
  GW=$(grep -c 'traffic_mode=semantic_active' "$LOG" 2>/dev/null || true)
  # mqtt_publisher uses stdlib logging — "MQTT connected" may not appear in ros launch logs.
  GW_STARTED=$(grep -c 'Robot gateway started' "$LOG" 2>/dev/null || true)
  PAHO_ERR=$(grep -c "No module named 'paho'" "$LOG" 2>/dev/null || true)
  NAV_ACT=$(grep -c 'lifecycle_manager_navigation.*Managed nodes are active' "$LOG" 2>/dev/null || true)
  DIED_GW=$(grep -c 'robot_gateway_node.*process has died' "$LOG" 2>/dev/null || true)
  CRITICS_ERR=$(grep -c 'No critics defined for FollowPath' "$LOG" 2>/dev/null || true)
  if [ "${NAV_ACT:-0}" -ge 2 ] && [ "${GW:-0}" -ge 2 ] && [ "${GW_STARTED:-0}" -ge 2 ] \
     && [ "${PAHO_ERR:-0}" -eq 0 ] && [ "${DIED_GW:-0}" -eq 0 ] && [ "${CRITICS_ERR:-0}" -eq 0 ]; then
    echo "DUAL_READY nav=$NAV_ACT act=$ACT lane=$LA gw=$GW gw_started=$GW_STARTED t=${i}s"
    READY=1
    break
  fi
  if [ $((i % 15)) -eq 0 ]; then
    echo "wait t=${i}s nav=$NAV_ACT act=$ACT lane=$LA gw=$GW gw_started=$GW_STARTED died_gw=$DIED_GW critics=$CRITICS_ERR paho_err=$PAHO_ERR mqtt_full=$(grep -c 'MQTT outbound queue full' "$LOG" || true)"
  fi
  sleep 1
done

# Probe backend traffic mode / snapshot (Windows host via WSL)
HOST_IP=$(grep nameserver /etc/resolv.conf | awk '{print $2}' | head -1)
for base in "http://127.0.0.1:5090" "http://${HOST_IP}:5090" "http://localhost:5090"; do
  if curl -fsS "$base/api/fleet/ping" >/dev/null 2>&1; then
    API="$base"
    break
  fi
done
echo "API=${API:-NONE}"
if [ -n "${API:-}" ]; then
  curl -fsS "$API/api/fleet/traffic/snapshot" 2>/dev/null | head -c 2000 | tee "$ART/traffic_snapshot_boot.json" || true
  echo
fi

if [ "$READY" != 1 ]; then
  echo "DUAL_NOT_READY — capturing log greps"
  grep -E 'Managed nodes|LaneAware|semantic_active|MQTT|ERROR|Fatal|died|SemanticPolicy' "$LOG" | tail -60 | tee "$ART/dual_not_ready_grep.txt"
  exit 1
fi

sleep 5
python3 "$ROOT/scripts/run_dual_semantic_scenarios.py" \
  --endpoints "$EP_JSON" \
  --out-csv "$CSV" \
  --out-md "$OUT" \
  --api "${API:-}" \
  --timeout 180 | tee "$ART/dual_scenarios_summary.txt"

echo DUAL_ORCH_DONE
