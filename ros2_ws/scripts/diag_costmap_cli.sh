#!/usr/bin/env bash
# CLI-based costmap dump (avoids TF listener spam / QoS guesswork).
set -eo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u

ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
SIM=$ROOT/src/warotrans_simulation
ART=/tmp/nav2_live_artifacts
LOG=/tmp/diag_bringup.log
mkdir -p "$ART"

bash "$ROOT/scripts/kill_sim.sh"

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
cleanup(){ kill $LAUNCH_PID 2>/dev/null || true; bash "$ROOT/scripts/kill_sim.sh"; }
trap cleanup EXIT

for i in $(seq 1 120); do
  grep -q 'Managed nodes are active' "$LOG" && grep -q 'LaneAwareNavfn' "$LOG" && break
  sleep 1
done
sleep 20

echo "=== topics ===" | tee "$ART/topics.txt"
ros2 topic list | grep -E 'costmap|keepout|map$|clock' | tee -a "$ART/topics.txt"
echo "=== topic types ===" | tee -a "$ART/topics.txt"
for t in /map /global_costmap/costmap /global_costmap/costmap_raw /keepout_filter_mask /clock; do
  echo -n "$t: " | tee -a "$ART/topics.txt"
  ros2 topic info -v "$t" 2>/dev/null | head -20 | tee -a "$ART/topics.txt" || echo missing | tee -a "$ART/topics.txt"
done

# Sample grids via python without TF
python3 - <<'PY' | tee "$ART/diag_costmap_lethal.json"
import json, math, subprocess, tempfile, os, time
from pathlib import Path

def echo_once(topic, msg_type, timeout=8):
    out = Path(tempfile.mkstemp(prefix='grid_', suffix='.yaml')[1])
    try:
        # Use ros2 topic echo --once into file
        p = subprocess.run(
            ['bash','-lc', f'source /opt/ros/jazzy/setup.bash; source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash; timeout {timeout} ros2 topic echo {topic} --once > {out}'],
            capture_output=True, text=True)
        text = out.read_text(encoding='utf-8', errors='ignore')
        return text, p.returncode
    finally:
        out.unlink(missing_ok=True)

# Prefer costmap_raw then costmap
texts = {}
for t in ['/map', '/global_costmap/costmap_raw', '/global_costmap/costmap', '/keepout_filter_mask']:
    txt, rc = echo_once(t, '')
    texts[t] = {'rc': rc, 'chars': len(txt), 'head': txt[:500]}

# Parse YAML-ish OccupancyGrid from echo (fragile but ok for info fields)
import re

def parse_info(text):
    if not text.strip():
        return None
    def f(pat):
        m=re.search(pat, text)
        return float(m.group(1)) if m else None
    def i(pat):
        m=re.search(pat, text)
        return int(m.group(1)) if m else None
    return {
        'resolution': f(r'resolution:\s*([-\d.eE]+)'),
        'width': i(r'width:\s*(\d+)'),
        'height': i(r'height:\s*(\d+)'),
        'ox': f(r'origin:\s*\n\s*position:\s*\n\s*x:\s*([-\d.eE]+)'),
        'oy': f(r'origin:\s*\n\s*position:\s*\n\s*x:.*\n\s*y:\s*([-\d.eE]+)'),
        'data_len': len(re.findall(r'^-?\d+$', text, re.M)),
    }

# Better: use rclpy briefly without TF
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter

TRANSIENT = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
VOLATILE = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)

class C(Node):
    def __init__(self):
        super().__init__('c', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.got={}
        for topic in ['/map','/global_costmap/costmap','/global_costmap/costmap_raw','/keepout_filter_mask']:
            for qos in (TRANSIENT, VOLATILE):
                self.create_subscription(OccupancyGrid, topic, lambda m, t=topic: self.got.__setitem__(t,m), qos)

rclpy.init(); n=C()
t0=time.time()
while time.time()-t0 < 25:
    rclpy.spin_once(n, timeout_sec=0.2)
    if '/map' in n.got and ('/global_costmap/costmap' in n.got or '/global_costmap/costmap_raw' in n.got):
        break

def classify(c):
    return {255:'NO_INFORMATION',254:'LETHAL',253:'INSCRIBED',0:'FREE'}.get(c, f'INFLATED({c})' if c<253 else f'OTHER({c})')

def sample(grid, x, y):
    info=grid.info
    mx=int((x-info.origin.position.x)/info.resolution)
    my=int((y-info.origin.position.y)/info.resolution)
    if mx<0 or my<0 or mx>=info.width or my>=info.height: return None,'OOB',mx,my
    v=int(grid.data[my*info.width+mx]); return v, classify(v), mx, my

cost = n.got.get('/global_costmap/costmap') or n.got.get('/global_costmap/costmap_raw')
mp = n.got.get('/map')
ko = n.got.get('/keepout_filter_mask')
poses=[('EP-PARKING-A',-1.848,-0.525),('EP-INBOUND',1.747,1.543),('EP-OUTBOUND',1.793,-0.977),
       ('EP-SHELF-A-01',-1.325,1.644),('ONEWAY_MID',1.1,-0.327),('FREEISH',-1.5,0.2)]
samples=[]
for name,x,y in poses:
    row={'name':name,'x':x,'y':y}
    if mp:
        v,cl,mx,my=sample(mp,x,y); row['occ']=v; row['occ_class_note']='OccupancyGrid -1/0/100'
    if cost:
        v,cl,mx,my=sample(cost,x,y); row['cost']=v; row['cost_class']=cl; row['cmx']=mx; row['cmy']=my
    if ko:
        v,cl,mx,my=sample(ko,x,y); row['keepout']=v
    samples.append(row)

out={
  'echo_meta': texts,
  'received': list(n.got.keys()),
  'cost_topic': '/global_costmap/costmap' if '/global_costmap/costmap' in n.got else ('/global_costmap/costmap_raw' if cost else None),
  'cost_info': None if not cost else {
    'res': cost.info.resolution, 'w': cost.info.width, 'h': cost.info.height,
    'ox': cost.info.origin.position.x, 'oy': cost.info.origin.position.y,
  },
  'samples': samples,
}
print(json.dumps(out, indent=2))
n.destroy_node(); rclpy.shutdown()
PY

# Plan probes
timeout 20 ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
  "{goal: {header: {frame_id: map}, pose: {position: {x: 1.747, y: 1.543}, orientation: {z: -0.7071, w: 0.7071}}}, planner_id: GridBased, use_start: false}" \
  >"$ART/plan_laneaware.txt" 2>&1 || true

FILTER_WARN=$(grep -c 'Filter mask was not received' "$LOG" || true)
echo "filter_warn=$FILTER_WARN" | tee "$ART/filter_warn.txt"
grep -E 'start/goal lethal|empty path|Filter mask|Created global planner' "$LOG" | tail -30 | tee "$ART/bringup_grep.txt"
echo DONE
