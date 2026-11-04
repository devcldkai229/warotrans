#!/usr/bin/env bash
# Stage-3 dual live motion recovery: Gazebo UI + Tests A–F.
# A/B = single robot (peer not spawned — parking bay clearance).
# C–F = both robots.
set +e
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u

ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
AUD=/mnt/e/programming/my_project/warotrans-system/docs/audits/autonomous_gazebo_acceptance
ART=$AUD/runtime
EP_JSON=$ROOT/src/warotrans_simulation/config/map_one_v001_endpoints.json
CSV=$AUD/DUAL_MOTION_RESULTS.csv
MD=$AUD/DUAL_RECOVERY_RUN.md
mkdir -p "$ART"
: >"$CSV"
echo "test,robot,start_endpoint,goal_endpoint,status,error_code,start_xy,end_xy,displacement_m,goal_error_m,note" >"$CSV"
echo "# Dual motion recovery run" >"$MD"
echo "" >>"$MD"
echo "**Updated:** $(date -Iseconds)" >>"$MD"
echo "" >>"$MD"

launch_wait() {
  local robots="$1"
  local log="$2"
  bash "$ROOT/scripts/kill_sim.sh" 1>&2
  pkill -9 -f ros2-daemon 2>/dev/null || true
  sleep 2
  echo "Launching robots=$robots gui:=true ..." 1>&2
  # GUI + AMCL need more settle time than headless dual (avoid planner TF timeout).
  bash "$ROOT/scripts/launch_sim_map_one_dual.sh" gui:=true "robots:=${robots}" \
    nav_start_delay:=40.0 gateway_extra_delay:=25.0 >"$log" 2>&1 &
  LP=$!
  READY=0
  local need_nav=1
  local need_gw=1
  if [ "$robots" = "both" ]; then
    need_nav=2
    need_gw=2
  fi
  for i in $(seq 1 240); do
    GW=$(grep -c 'traffic_mode=semantic_active' "$log" 2>/dev/null || true)
    GW_STARTED=$(grep -c 'Robot gateway started' "$log" 2>/dev/null || true)
    PAHO_ERR=$(grep -c "No module named 'paho'" "$log" 2>/dev/null || true)
    NAV_ACT=$(grep -c 'lifecycle_manager_navigation.*Managed nodes are active' "$log" 2>/dev/null || true)
    DIED_GW=$(grep -c 'robot_gateway_node.*process has died' "$log" 2>/dev/null || true)
    if [ "${NAV_ACT:-0}" -ge "$need_nav" ] && [ "${GW:-0}" -ge "$need_gw" ] \
       && [ "${GW_STARTED:-0}" -ge "$need_gw" ] && [ "${PAHO_ERR:-0}" -eq 0 ] \
       && [ "${DIED_GW:-0}" -eq 0 ]; then
      echo "READY robots=$robots nav=$NAV_ACT gw=$GW t=${i}s" 1>&2
      READY=1
      break
    fi
    if [ $((i % 15)) -eq 0 ]; then
      echo "wait robots=$robots t=${i}s nav=$NAV_ACT gw=$GW gw_started=$GW_STARTED" 1>&2
    fi
    sleep 1
  done
  if [ "$READY" != 1 ]; then
    echo "NOT_READY robots=$robots" 1>&2
    grep -E 'Managed nodes|semantic_active|ERROR|died|paho|robots=' "$log" | tail -50 1>&2
    kill "$LP" 2>/dev/null
    return 1
  fi
  # Allow heartbeat ONLINE → MarkOnline to clear sticky OFFLINE from bringup gap.
  sleep 8
  echo "$LP"
  return 0
}

append_csv() {
  # append rows from a temp csv skipping header
  local src="$1"
  if [ -f "$src" ]; then
    tail -n +2 "$src" >>"$CSV" || true
  fi
}

# --- Test A: SIM-01 only EP-PARKING-A → EP-INBOUND ---
echo "=== ANNOUNCE: Test A — SIM-01 only START=EP-PARKING-A GOAL=EP-INBOUND ==="
LOGA=$ART/recovery_test_a_bringup.log
TRACEA=$ART/sync_motion_trace_a.csv
LP=$(launch_wait sim01 "$LOGA") || { echo "A BLOCKED bringup"; exit 1; }
python3 "$ROOT/scripts/sync_motion_trace.py" --out "$TRACEA" --ns sim01 --hz 5 >"$ART/sync_trace_a.log" 2>&1 &
TRACE_PID=$!
python3 "$ROOT/scripts/run_dual_motion_recovery.py" \
  --endpoints "$EP_JSON" --out-csv "$ART/tmp_a.csv" --out-md "$ART/tmp_a.md" \
  --timeout 180 --tests A | tee "$ART/recovery_a_summary.txt"
kill "$TRACE_PID" 2>/dev/null
append_csv "$ART/tmp_a.csv"
cat "$ART/tmp_a.md" >>"$MD" || true
A_STATUS=$(tail -n +2 "$ART/tmp_a.csv" | head -1 | cut -d, -f5 || true)
bash "$ROOT/scripts/kill_sim.sh"
if [ "$A_STATUS" != "PASS" ]; then
  echo "Test A status=$A_STATUS — stop before dual contention (safety gate)"
  echo "B,C,D,E,F NOT_RUN (blocked by A)" >>"$MD"
  for t in B C D E F; do
    echo "$t,-,-,-,NOT_RUN,,,,,\"blocked by Test A $A_STATUS\"" >>"$CSV"
  done
  echo DUAL_MOTION_RECOVERY_DONE
  exit 0
fi

# --- Test B: SIM-02 only EP-PARKING-B → EP-OUTBOUND ---
echo "=== ANNOUNCE: Test B — SIM-02 only START=EP-PARKING-B GOAL=EP-OUTBOUND ==="
LOGB=$ART/recovery_test_b_bringup.log
TRACEB=$ART/sync_motion_trace_b.csv
LP=$(launch_wait sim02 "$LOGB") || { echo "B BLOCKED bringup"; exit 1; }
python3 "$ROOT/scripts/sync_motion_trace.py" --out "$TRACEB" --ns sim02 --hz 5 >"$ART/sync_trace_b.log" 2>&1 &
TRACE_PID=$!
python3 "$ROOT/scripts/run_dual_motion_recovery.py" \
  --endpoints "$EP_JSON" --out-csv "$ART/tmp_b.csv" --out-md "$ART/tmp_b.md" \
  --timeout 180 --tests B | tee "$ART/recovery_b_summary.txt"
kill "$TRACE_PID" 2>/dev/null
append_csv "$ART/tmp_b.csv"
cat "$ART/tmp_b.md" >>"$MD" || true
B_STATUS=$(tail -n +2 "$ART/tmp_b.csv" | head -1 | cut -d, -f5 || true)
bash "$ROOT/scripts/kill_sim.sh"
if [ "$B_STATUS" != "PASS" ]; then
  echo "Test B status=$B_STATUS — stop before dual"
  echo "C,D,E,F NOT_RUN (blocked by B)" >>"$MD"
  for t in C D E F; do
    echo "$t,-,-,-,NOT_RUN,,,,,\"blocked by Test B $B_STATUS\"" >>"$CSV"
  done
  echo DUAL_MOTION_RECOVERY_DONE
  exit 0
fi

# --- Tests C–F: both robots ---
echo "=== ANNOUNCE: Tests C–F — both robots Parking A/B ==="
LOGD=$ART/recovery_test_dual_bringup.log
TRACED=$ART/sync_motion_trace_dual.csv
LP=$(launch_wait both "$LOGD") || { echo "DUAL BLOCKED bringup"; exit 1; }
python3 "$ROOT/scripts/sync_motion_trace.py" --out "$TRACED" --ns sim01,sim02 --hz 5 >"$ART/sync_trace_dual.log" 2>&1 &
TRACE_PID=$!
python3 "$ROOT/scripts/run_dual_motion_recovery.py" \
  --endpoints "$EP_JSON" --out-csv "$ART/tmp_dual.csv" --out-md "$ART/tmp_dual.md" \
  --timeout 180 --tests C,D,E,F | tee "$ART/recovery_dual_summary.txt"
kill "$TRACE_PID" 2>/dev/null
append_csv "$ART/tmp_dual.csv"
cat "$ART/tmp_dual.md" >>"$MD" || true
bash "$ROOT/scripts/kill_sim.sh"

echo DUAL_MOTION_RECOVERY_DONE
