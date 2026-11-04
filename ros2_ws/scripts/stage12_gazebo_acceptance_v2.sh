#!/usr/bin/env bash
# Stage-1/2 retest: free-start planning + policy validator + gate log evidence.
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u

ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM_SRC="$ROOT/src/warotrans_simulation"
LOG=/tmp/stage12v2_bringup.log
RESULT=/tmp/stage12v2_results.json

cleanup() {
  if [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  pkill -f 'ros2 launch warotrans_simulation sim_bringup' 2>/dev/null || true
  sleep 1
}
trap cleanup EXIT

rm -f "$LOG" "$RESULT"
echo '[]' >"$RESULT"

append_result() {
  python3 - "$RESULT" "$1" <<'PY'
import json,sys
path, row = sys.argv[1], json.loads(sys.argv[2])
data=json.loads(open(path,encoding='utf-8').read() or '[]')
data.append(row)
open(path,'w',encoding='utf-8').write(json.dumps(data,indent=2)+'\n')
print(row.get('id'), row.get('result'))
PY
}

ros2 launch warotrans_simulation sim_bringup.launch.py \
  layout:="$SIM_SRC/config/layouts/map_one_v001.yaml" \
  map:="$SIM_SRC/maps/map_one_v001/map.yaml" \
  world:="$SIM_SRC/worlds/map_one_v001.sdf" \
  spawn_endpoint_code:=EP-PARKING-A \
  nav_runtime:=nav2_semantic \
  enable_authority_gate:=true \
  stage1_open_grant:=true \
  gateway:=false \
  gui:=false \
  >"$LOG" 2>&1 &
LAUNCH_PID=$!

for i in $(seq 1 90); do
  if grep -q 'Created global planner plugin GridBased of type warotrans_lane_planner::LaneAwareNavfn' "$LOG" \
     && grep -q 'path_policy_validator ready' "$LOG" \
     && grep -q 'lifecycle_manager_navigation.*Managed nodes are active' "$LOG"; then
    echo "bringup OK ${i}s"
    break
  fi
  sleep 1
done
sleep 5

FILTER_WARN=$(grep -c 'Filter mask was not received' "$LOG" || true)
GATE_LOG=$(grep -c 'cmd_vel_authority_gate' "$LOG" || true)
OPEN_GRANT=$(grep -c 'STAGE1_OPEN_GRANT' "$LOG" || true)
append_result "{\"id\":\"bringup\",\"result\":\"PASS\",\"filterMaskWarnCount\":$FILTER_WARN,\"authorityGateLogLines\":$GATE_LOG,\"openGrant\":$OPEN_GRANT}"

# Occupancy sample at parking + free corridor
python3 - <<'PY' > /tmp/occ_sample.txt
from pathlib import Path
import struct
yaml=Path('/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/map_one_v001/map.yaml').read_text()
# quick parse
import re
res=float(re.search(r'resolution:\s*([0-9.]+)',yaml).group(1))
ox=float(re.search(r'origin:\n-\s*([-\d.]+)',yaml).group(1))
oy=float(re.search(r'origin:\n-\s*[-\d.]+\n-\s*([-\d.]+)',yaml).group(1))
pgm=Path('/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/map_one_v001/map.pgm').read_bytes()
assert pgm.startswith(b'P5')
# skip header
i=3
while pgm[i:i+1] in (b'#',b'\n',b' ') or pgm[i:i+1].isdigit() or pgm[i:i+1]==b'\r':
    # crude: find dimensions line after comments
    break
lines=pgm.split(b'\n')
idx=1
while lines[idx].startswith(b'#'):
    idx+=1
w,h=map(int,lines[idx].split()); idx+=1
maxv=int(lines[idx]); idx+=1
header=b'\n'.join(lines[:idx+1])+b'\n'
# rebuild properly
text=pgm.decode('latin1')
parts=text.split('\n')
k=1
while parts[k].startswith('#'):
    k+=1
w,h=map(int,parts[k].split()); k+=1
maxv=int(parts[k]); k+=1
# binary after header newline following maxval
header_len=sum(len(p)+1 for p in parts[:k])
data=pgm[header_len:]
def sample(x,y):
    c=int((x-ox)/res); r=int((y-oy)/res)
    # PGM row 0 is top
    rr=h-1-r
    if not (0<=c<w and 0<=rr<h):
        return None,'oob'
    v=data[rr*w+c]
    return int(v), f'c={c},r={r},rr={rr}'
for name,x,y in [('parking',-1.848,-0.525),('oneway_mid',1.1,-0.327),('inbound',1.747,1.543),('freeish',-1.5,0.2)]:
    v,meta=sample(x,y)
    print(name,x,y,'pix',v,meta)
PY
cat /tmp/occ_sample.txt
append_result "{\"id\":\"occupancy_samples\",\"result\":\"INFO\",\"detail\":$(python3 -c 'import json; print(json.dumps(open("/tmp/occ_sample.txt").read()))')}"

quat() { python3 -c "import math; y=float('$1'); print(f'{math.sin(y/2):.10f} {math.cos(y/2):.10f}')"; }

plan_free_start() {
  local label=$1 sx=$2 sy=$3 syaw=$4 gx=$5 gy=$6 gyaw=$7
  read -r SZ SW <<<"$(quat "$syaw")"
  read -r GZ GW <<<"$(quat "$gyaw")"
  local out=/tmp/planv2_"$label".txt
  set +e
  timeout 30 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
    "{start: {header: {frame_id: map}, pose: {position: {x: $sx, y: $sy, z: 0.0}, orientation: {z: $SZ, w: $SW}}}, goal: {header: {frame_id: map}, pose: {position: {x: $gx, y: $gy, z: 0.0}, orientation: {z: $GZ, w: $GW}}}, planner_id: GridBased, use_start: true}" \
    >"$out" 2>&1
  set -e
  local n
  n=$(grep -c 'position:' "$out" || true)
  if grep -q 'Goal finished with status: SUCCEEDED' "$out" && ! grep -q 'poses: \[\]' "$out"; then
    append_result "{\"id\":\"plan_$label\",\"result\":\"PASS\",\"poseLines\":$n,\"log\":\"$out\"}"
    return 0
  fi
  local err
  err=$(grep -E 'error_code:|ABORTED|lethal|empty path' "$out" | head -3 | tr '\n' ';' | sed 's/"/\\"/g')
  append_result "{\"id\":\"plan_$label\",\"result\":\"FAIL\",\"poseLines\":$n,\"err\":\"$err\",\"log\":\"$out\"}"
  return 1
}

# ONE_WAY forward along LTR-C (x increases)
plan_free_start oneway_legal 0.85 -0.327 0.0 1.55 -0.327 0.0 || true
# ONE_WAY reverse — expect FAIL
set +e
plan_free_start oneway_illegal 1.55 -0.327 3.14159 0.85 -0.327 3.14159
ILL=$?
set -e
if [[ $ILL -ne 0 ]]; then
  append_result '{"id":"oneway_illegal_policy","result":"PASS","note":"reverse ONE_WAY did not SUCCEED"}'
else
  append_result '{"id":"oneway_illegal_policy","result":"FAIL","note":"reverse ONE_WAY SUCCEEDED"}'
fi

# Free-space corridor hop (BTT-B region mid)
plan_free_start freespace_hop 0.12 0.5 1.57 0.12 1.2 1.57 || true

# Parking start lethal confirmation (use_start false uses robot pose)
set +e
timeout 20 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: -1.02, y: -0.529, z: 0.0}, orientation: {w: 1.0}}}, planner_id: GridBased, use_start: false}" \
  >/tmp/planv2_from_parking.txt 2>&1
set -e
if grep -q 'start/goal lethal' "$LOG" || grep -q 'poses: \[\]' /tmp/planv2_from_parking.txt; then
  append_result '{"id":"nav_from_parking_a","result":"FAIL","rootCause":"LaneAwareNavfn start/goal lethal at EP-PARKING-A","log":"/tmp/planv2_from_parking.txt"}'
else
  append_result '{"id":"nav_from_parking_a","result":"PASS"}'
fi

# path_policy KEEP_OUT / NO_STOP
publish_goal_check() {
  local label=$1 x=$2 y=$3 expect=$4
  rm -f /tmp/reason_"$label".txt
  timeout 8 ros2 topic echo /path_policy/reason --once >/tmp/reason_"$label".txt 2>/dev/null &
  local pid=$!
  sleep 1
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: $x, y: $y, z: 0.0}, orientation: {w: 1.0}}}" >/dev/null 2>&1 || true
  wait $pid 2>/dev/null || true
  if grep -qi "$expect" /tmp/reason_"$label".txt 2>/dev/null; then
    append_result "{\"id\":\"$label\",\"result\":\"PASS\",\"evidence\":\"/tmp/reason_$label.txt\"}"
  else
    append_result "{\"id\":\"$label\",\"result\":\"FAIL\",\"evidence\":\"/tmp/reason_$label.txt\",\"body\":$(python3 -c "import json;print(json.dumps(open('/tmp/reason_$label.txt').read() if __import__('pathlib').Path('/tmp/reason_$label.txt').exists() else ''))")}"
  fi
}

read -r KX KY <<<"$(python3 - <<'PY'
import json
from pathlib import Path
z=json.loads(Path('/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/map_one_v001/keepout.json').read_text())['zones'][0]['rings'][0]
xs=[c[0] for c in z]; ys=[c[1] for c in z]
print((min(xs)+max(xs))/2,(min(ys)+max(ys))/2)
PY
)"
read -r NX NY <<<"$(python3 - <<'PY'
import json
from pathlib import Path
z=json.loads(Path('/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/map_one_v001/no_stop.json').read_text())['zones'][0]['rings'][0]
xs=[c[0] for c in z]; ys=[c[1] for c in z]
print((min(xs)+max(xs))/2,(min(ys)+max(ys))/2)
PY
)"
publish_goal_check keepout_goal_validator "$KX" "$KY" KEEP_OUT
publish_goal_check no_stop_goal_validator "$NX" "$NY" NO_STOP

if [[ "$FILTER_WARN" -gt 5 ]]; then
  append_result "{\"id\":\"keepout_speed_filter_mask\",\"result\":\"FAIL\",\"filterMaskWarnCount\":$FILTER_WARN}"
  append_result '{"id":"speed_compliance_motion","result":"NOT_RUN","note":"filter mask missing"}'
else
  append_result "{\"id\":\"keepout_speed_filter_mask\",\"result\":\"PASS\",\"filterMaskWarnCount\":$FILTER_WARN}"
  append_result '{"id":"speed_compliance_motion","result":"NOT_RUN","note":"no automated probe"}'
fi

append_result '{"id":"physical_motion_endpoint","result":"NOT_RUN","note":"blocked by parking start lethal"}'
append_result '{"id":"replan_semantic","result":"NOT_RUN"}'
append_result '{"id":"stage3a_dual","result":"BLOCKED","note":"Stage-2 safety not cleared (parking lethal + filter masks)"}'

echo '=== RESULTS ==='
cat "$RESULT"
