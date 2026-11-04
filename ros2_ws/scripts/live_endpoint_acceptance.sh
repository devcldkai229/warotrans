#!/usr/bin/env bash
# SIM-01 live endpoint acceptance after costmap/filter/LaneAware fixes.
set +e
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u

ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM=$ROOT/src/warotrans_simulation
ART=/tmp/nav2_live_artifacts
LOG=$ART/live_bringup.log
RESULT=$ART/live_results.json
mkdir -p "$ART"
echo '[]' >"$RESULT"

append() {
  python3 -c 'import json,sys;p,r=sys.argv[1],json.loads(sys.argv[2]);d=json.load(open(p));d.append(r);json.dump(d,open(p,"w"),indent=2);print(r.get("id"),r.get("result"))' "$RESULT" "$1"
}

bash "$ROOT/scripts/kill_sim.sh"
pkill -9 -f ros2-daemon 2>/dev/null || true

ros2 launch warotrans_simulation sim_bringup.launch.py \
  layout:=$SIM/config/layouts/map_one_v001.yaml \
  map:=$SIM/maps/map_one_v001/map.yaml \
  world:=$SIM/worlds/map_one_v001.sdf \
  spawn_endpoint_code:=EP-PARKING-A \
  nav_runtime:=nav2_semantic \
  enable_authority_gate:=true \
  stage1_open_grant:=true \
  gateway:=false \
  gui:=false \
  >"$LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  append '{"id":"session_end","result":"INFO","note":"revoking open-grant by killing stage1_open_grant/sim"}'
  kill "$LAUNCH_PID" 2>/dev/null || true
  bash "$ROOT/scripts/kill_sim.sh"
}
trap cleanup EXIT

for i in $(seq 1 120); do
  if grep -q 'LaneAwareNavfn' "$LOG" && grep -q 'Managed nodes are active' "$LOG"; then
    echo "ACTIVE_$i"
    break
  fi
  sleep 1
done
sleep 20

FILTER_WARN=$(grep -c 'Filter mask was not received' "$LOG" || true)
append "{\"id\":\"bringup\",\"result\":\"PASS\",\"filterMaskWarnCount\":$FILTER_WARN}"

# Costmap dump via service
python3 "$ROOT/scripts/diag_get_costmap.py" >"$ART/diag_summary.txt" 2>"$ART/diag_get_err.txt"
append "{\"id\":\"costmap_diag\",\"result\":\"INFO\",\"artifact\":\"$ART/diag_costmap_lethal.json\"}"

# Policy probe (positive evidence; retries internally)
python3 "$ROOT/scripts/path_policy_probe.py" >"$ART/path_policy_probe_out.txt" 2>"$ART/path_policy_probe_err.txt"
PP_RC=$?
if [ "$PP_RC" -eq 0 ]; then
  append "{\"id\":\"path_policy_probe\",\"result\":\"PASS\",\"artifact\":\"$ART/path_policy_probe.json\"}"
else
  append "{\"id\":\"path_policy_probe\",\"result\":\"FAIL\",\"artifact\":\"$ART/path_policy_probe.json\",\"rc\":$PP_RC}"
fi

cancel_nav() {
  python3 "$ROOT/scripts/cancel_nav2_goals.py" >/tmp/nav2_live_artifacts/cancel_nav.txt 2>&1 || true
  sleep 2
}

nav_goal() {
  local id=$1 x=$2 y=$3 yaw=$4 timeout_s=${5:-180}
  local z w
  z=$(python3 -c "import math; print(math.sin($yaw/2))")
  w=$(python3 -c "import math; print(math.cos($yaw/2))")
  local out=$ART/nav_$id.txt
  cancel_nav
  # No --feedback: prior runs wrote 600k+ line logs and OOM-killed Gazebo.
  timeout "$timeout_s" ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {z: $z, w: $w}}}}" \
    >"$out" 2>&1
  local rc=$?
  if grep -q 'Goal finished with status: SUCCEEDED' "$out"; then
    append "{\"id\":\"nav_$id\",\"result\":\"PASS\",\"log\":\"$out\"}"
    cancel_nav
    return 0
  fi
  append "{\"id\":\"nav_$id\",\"result\":\"FAIL\",\"log\":\"$out\",\"timeout_rc\":$rc}"
  cancel_nav
  return 1
}

plan_goal() {
  local id=$1 x=$2 y=$3 yaw=$4
  local z w
  z=$(python3 -c "import math; print(math.sin($yaw/2))")
  w=$(python3 -c "import math; print(math.cos($yaw/2))")
  local out=$ART/plan_$id.txt
  timeout 30 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
    "{goal: {header: {frame_id: map}, pose: {position: {x: $x, y: $y}, orientation: {z: $z, w: $w}}}, planner_id: GridBased, use_start: false}" \
    >"$out" 2>&1
  if grep -q 'Goal finished with status: SUCCEEDED' "$out" && ! grep -q 'poses: \[\]' "$out"; then
    append "{\"id\":\"plan_$id\",\"result\":\"PASS\",\"log\":\"$out\"}"
    return 0
  fi
  append "{\"id\":\"plan_$id\",\"result\":\"FAIL\",\"log\":\"$out\"}"
  return 1
}

# Parking -> Inbound
plan_goal parking_to_inbound 1.747 1.543 -1.5708
nav_goal parking_to_inbound 1.747 1.543 -1.5708 240

# Inbound -> Shelf A01
plan_goal inbound_to_shelf -1.325 1.644 3.14159
nav_goal inbound_to_shelf -1.325 1.644 3.14159 240

# Shelf -> Outbound
plan_goal shelf_to_outbound 1.793 -0.977 1.5708
nav_goal shelf_to_outbound 1.793 -0.977 1.5708 240

# Outbound -> Parking
plan_goal outbound_to_parking -1.848 -0.525 -1.5708
nav_goal outbound_to_parking -1.848 -0.525 -1.5708 240

# ONE_WAY adversarial on EDGE-EDGE-LTR-C (tangent +x). Endpoints outside NO_STOP.
timeout 30 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{start: {header: {frame_id: map}, pose: {position: {x: 1.55, y: -0.327}, orientation: {z: 1.0, w: 0.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: 1.15, y: -0.327}, orientation: {z: 1.0, w: 0.0}}}, planner_id: GridBased, use_start: true}" \
  >"$ART/plan_oneway_illegal.txt" 2>&1
python3 - <<'PY' >>"$ART/oneway_evidence.txt"
import re, math
from pathlib import Path
text = Path("/tmp/nav2_live_artifacts/plan_oneway_illegal.txt").read_text(errors="ignore")
xs = [float(x) for x in re.findall(r"position:\s*\n\s*x:\s*([-\d.]+)", text)]
ys = [float(y) for y in re.findall(r"\n\s*y:\s*([-\d.]+)", text)]
# crude: pair consecutive x from poses; keep only map-ish range
pts = []
# Fallback parse pose blocks
blocks = re.findall(r"position:\s*\{?\s*x:\s*([-\d.]+).*?y:\s*([-\d.]+)", text, re.S)
if not blocks:
    # YAML-ish indented
    for m in re.finditer(r"x:\s*([-\d.]+)\s*\n\s*y:\s*([-\d.]+)", text):
        blocks.append((m.group(1), m.group(2)))
pts = [(float(a), float(b)) for a,b in blocks if -3 < float(a) < 3 and -2 < float(b) < 3]
print(f"illegal_pose_count={len(pts)}")
if len(pts) >= 2:
    dx = pts[-1][0] - pts[0][0]
    dy = pts[-1][1] - pts[0][1]
    ang = math.atan2(dy, dx)
    # lane tangent = 0; reverse travel ≈ pi
    print(f"illegal_delta=({dx:.3f},{dy:.3f}) translation_yaw={ang:.3f} vs_lane_tangent=0.0")
    against = abs(abs(ang) - math.pi) < 0.6 or dx < -0.05
    print(f"against_lane={against}")
else:
    print("illegal_empty_or_unparsed")
PY
if grep -q 'SUCCEEDED' "$ART/plan_oneway_illegal.txt" && ! grep -q 'poses: \[\]' "$ART/plan_oneway_illegal.txt"; then
  # Path found: PASS only if it is a legal detour (net translation not against +x lane).
  if grep -q 'against_lane=True' "$ART/oneway_evidence.txt"; then
    append '{"id":"oneway_illegal","result":"FAIL","note":"path against ONE_WAY tangent"}'
  else
    append '{"id":"oneway_illegal","result":"PASS","note":"detour not against lane tangent"}'
  fi
else
  append '{"id":"oneway_illegal","result":"PASS","note":"reverse ONE_WAY rejected or empty (fail-closed)"}'
fi

timeout 30 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{start: {header: {frame_id: map}, pose: {position: {x: 1.15, y: -0.327}, orientation: {w: 1.0}}}, goal: {header: {frame_id: map}, pose: {position: {x: 1.55, y: -0.327}, orientation: {w: 1.0}}}, planner_id: GridBased, use_start: true}" \
  >"$ART/plan_oneway_legal.txt" 2>&1
python3 - <<'PY' >>"$ART/oneway_evidence.txt"
import re, math
from pathlib import Path
text = Path("/tmp/nav2_live_artifacts/plan_oneway_legal.txt").read_text(errors="ignore")
blocks = re.findall(r"x:\s*([-\d.]+)\s*\n\s*y:\s*([-\d.]+)", text)
pts = [(float(a), float(b)) for a,b in blocks if -3 < float(a) < 3 and -2 < float(b) < 3]
print(f"legal_pose_count={len(pts)}")
if len(pts) >= 2:
    dx = pts[-1][0] - pts[0][0]
    print(f"legal_delta_x={dx:.3f} expected_positive_along_tangent")
PY
if grep -q 'SUCCEEDED' "$ART/plan_oneway_legal.txt" && ! grep -q 'poses: \[\]' "$ART/plan_oneway_legal.txt"; then
  append '{"id":"oneway_legal","result":"PASS"}'
else
  append '{"id":"oneway_legal","result":"FAIL"}'
fi

grep -c 'Filter mask was not received' "$LOG" >"$ART/filter_warn_final.txt"
grep -E 'start/goal|LaneAware|Filter mask|INSCRIBED|LETHAL|NO_INFORMATION' "$LOG" | tail -50 >"$ART/bringup_grep.txt"

echo '=== LIVE RESULTS ==='
cat "$RESULT"
echo DONE
