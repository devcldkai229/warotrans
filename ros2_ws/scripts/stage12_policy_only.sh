#!/usr/bin/env bash
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u
ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM_SRC=$ROOT/src/warotrans_simulation
LOG=/tmp/policy_only.log
RESULT=/tmp/policy_only_results.json
cleanup(){ kill $LAUNCH_PID 2>/dev/null || true; pkill -f 'sim_bringup.launch' 2>/dev/null || true; }
trap cleanup EXIT
rm -f "$LOG"; echo '[]' >"$RESULT"
append(){ python3 -c 'import json,sys;p=sys.argv[1];r=json.loads(sys.argv[2]);d=json.load(open(p));d.append(r);json.dump(d,open(p,"w"),indent=2);print(r["id"],r["result"])' "$RESULT" "$1"; }

ros2 launch warotrans_simulation sim_bringup.launch.py \
  layout:=$SIM_SRC/config/layouts/map_one_v001.yaml \
  map:=$SIM_SRC/maps/map_one_v001/map.yaml \
  world:=$SIM_SRC/worlds/map_one_v001.sdf \
  spawn_endpoint_code:=EP-PARKING-A nav_runtime:=nav2_semantic \
  enable_authority_gate:=true stage1_open_grant:=true gateway:=false gui:=false \
  >"$LOG" 2>&1 &
LAUNCH_PID=$!
for i in $(seq 1 90); do
  grep -q 'path_policy_validator ready' "$LOG" && grep -q 'Managed nodes are active' "$LOG" && break
  sleep 1
done
sleep 3
FILTER_WARN=$(grep -c 'Filter mask was not received' "$LOG" || true)
GATE=$(grep -c 'cmd_vel_authority_gate' "$LOG" || true)
append "{\"id\":\"bringup_policy\",\"result\":\"PASS\",\"filterMaskWarnCount\":$FILTER_WARN,\"gateLog\":$GATE}"

check(){
  local id=$1 x=$2 y=$3 exp=$4
  rm -f /tmp/r_$id.txt
  timeout 10 ros2 topic echo /path_policy/reason --once >/tmp/r_$id.txt 2>/dev/null &
  local p=$!; sleep 1
  ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'map'}, pose: {position: {x: $x, y: $y}, orientation: {w: 1.0}}}" >/dev/null 2>&1 || true
  wait $p 2>/dev/null || true
  if grep -qi "$exp" /tmp/r_$id.txt; then append "{\"id\":\"$id\",\"result\":\"PASS\"}"; else
    body=$(python3 -c "import json;print(json.dumps(open('/tmp/r_$id.txt').read() if __import__('pathlib').Path('/tmp/r_$id.txt').exists() else ''))")
    append "{\"id\":\"$id\",\"result\":\"FAIL\",\"body\":$body}"; fi
}
read -r KX KY <<EOF
$(python3 - <<'PY'
import json
from pathlib import Path
z=json.loads(Path('/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/map_one_v001/keepout.json').read_text())['zones'][0]['rings'][0]
xs=[c[0] for c in z]; ys=[c[1] for c in z]
print((min(xs)+max(xs))/2,(min(ys)+max(ys))/2)
PY
)
EOF
read -r NX NY <<EOF
$(python3 - <<'PY'
import json
from pathlib import Path
z=json.loads(Path('/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/map_one_v001/no_stop.json').read_text())['zones'][0]['rings'][0]
xs=[c[0] for c in z]; ys=[c[1] for c in z]
print((min(xs)+max(xs))/2,(min(ys)+max(ys))/2)
PY
)
EOF
check keepout_goal $KX $KY KEEP_OUT
check no_stop_goal $NX $NY NO_STOP
if [[ $FILTER_WARN -gt 5 ]]; then append "{\"id\":\"filter_mask\",\"result\":\"FAIL\",\"n\":$FILTER_WARN}"; else append "{\"id\":\"filter_mask\",\"result\":\"PASS\",\"n\":$FILTER_WARN}"; fi
echo '==='; cat "$RESULT"
