#!/usr/bin/env bash
# P0: one Gazebo GUI + SIM-01; prove physical motion via GT odom/model pose.
set +e
set +u
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
set -u

ROOT=/mnt/e/programming/my_project/warotrans/ros2_ws
AUD=/mnt/e/programming/my_project/warotrans-system/docs/audits
ART=$AUD/gz_physical_motion
EP_JSON=$ROOT/src/warotrans_simulation/config/map_one_v001_endpoints.json
LOG=$ART/bringup.log
TRACE=$ART/gt_trace.csv
PIPE=$ART/pipeline_probe.txt
SUMMARY=$ART/nav_summary.txt
mkdir -p "$ART"

echo "=== ANNOUNCE: P0 physical motion — ONE world map_one_v001, ONE robot warotrans_sim_01 @ EP-PARKING-A ==="

bash "$ROOT/scripts/kill_sim.sh"
# Soft-stop daemon only — SIGKILL left subsequent `ros2 topic hz` with invalid rcl context.
ros2 daemon stop 2>/dev/null || true
sleep 2

# Inventory after kill
{
  echo "## post-kill processes"
  pgrep -af 'gz sim|multi_sim|robot_gateway' || echo none
} | tee "$ART/instance_inventory.txt"

bash "$ROOT/scripts/launch_sim_map_one_dual.sh" gui:=true robots:=sim01 \
  nav_start_delay:=40.0 gateway_extra_delay:=25.0 >"$LOG" 2>&1 &
LP=$!
cleanup() { kill "$TRACE_PID" 2>/dev/null; kill "$LP" 2>/dev/null; bash "$ROOT/scripts/kill_sim.sh"; }
trap cleanup EXIT

READY=0
for i in $(seq 1 240); do
  NAV=$(grep -c 'lifecycle_manager_navigation.*Managed nodes are active' "$LOG" 2>/dev/null || true)
  GW=$(grep -c 'traffic_mode=semantic_active' "$LOG" 2>/dev/null || true)
  GW_S=$(grep -c 'Robot gateway started' "$LOG" 2>/dev/null || true)
  if [ "${NAV:-0}" -ge 1 ] && [ "${GW:-0}" -ge 1 ] && [ "${GW_S:-0}" -ge 1 ]; then
    echo "READY t=${i}s nav=$NAV gw=$GW"
    READY=1
    break
  fi
  [ $((i % 15)) -eq 0 ] && echo "wait t=${i}s nav=$NAV gw=$GW gw_s=$GW_S"
  sleep 1
done
if [ "$READY" != 1 ]; then
  echo "NOT_READY"
  grep -E 'ERROR|Failed|died|semantic_active|Managed nodes' "$LOG" | tail -40 | tee "$ART/not_ready.txt"
  exit 1
fi

# Instance verification (all ros2 calls hard-timeout — verbose info can hang forever)
{
  echo "## live instance"
  echo "launch_pid=$LP"
  grep -E 'world=|robot=warotrans_sim_01|robots=sim01|spawn=' "$LOG" | head -20
  echo "## gz topics (sample)"
  gz topic -l 2>/dev/null | grep -E 'sim01|map_one|warotrans' | head -40 || true
  echo "## clock hz (2s)"
  timeout 2 ros2 topic hz /clock 2>&1 | head -5 || echo "clock_hz_timeout"
  echo "## odom hz (2s)"
  timeout 2 ros2 topic hz /sim01/odom 2>&1 | head -5 || echo "odom_hz_timeout"
  echo "## gz stats once"
  timeout 2 gz topic -e -t /world/map_one_v001/stats -n 1 2>&1 | head -20 || true
} | tee "$ART/instance_verify.txt"

sleep 8  # HB ONLINE / MarkOnline

python3 "$ROOT/scripts/gz_physical_motion_trace.py" \
  --out "$TRACE" --ns sim01 --world map_one_v001 --model warotrans_sim_01 --hz 10 \
  >"$ART/trace_runner.log" 2>&1 &
TRACE_PID=$!

# Pipeline probe — list + short hz only (no ros2 topic info -v)
{
  echo "## ros2 topic list (sim01)"
  timeout 5 ros2 topic list 2>/dev/null | grep sim01 | head -40 || echo topic_list_timeout
  echo "## hz samples (2s each)"
  for t in /sim01/cmd_vel_nav /sim01/cmd_vel_smoothed /sim01/cmd_vel_safe /sim01/cmd_vel /sim01/traffic/authority_ok /sim01/odom; do
    echo "--- $t ---"
    timeout 2 ros2 topic hz "$t" 2>&1 | head -4 || echo "no_hz"
  done
  echo "## gz cmd_vel"
  gz topic -l 2>/dev/null | grep cmd_vel || true
  echo "## gz cmd_vel echo 1s"
  timeout 1 gz topic -e -t /sim01/cmd_vel -n 3 2>&1 | head -30 || true
} | tee "$PIPE"

GOAL_CODE="${P0_GOAL_CODE:-EP-CHARGING-A}"
echo "=== ANNOUNCE: NavigateToPose SIM-01 EP-PARKING-A → ${GOAL_CODE} (watch Gazebo GUI) ==="
python3 - <<PY | tee "$SUMMARY"
import json, math, time, os
from pathlib import Path
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from tf2_ros import Buffer, TransformListener

GOAL_CODE = os.environ.get("P0_GOAL_CODE", "EP-CHARGING-A")
EP = json.loads(Path("/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/config/map_one_v001_endpoints.json").read_text())["endpoints"]
goal_ep = next(e for e in EP if e["code"] == GOAL_CODE)

class N(Node):
    def __init__(self):
        super().__init__("p0_nav", namespace="sim01",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            cli_args=["--ros-args", "-r", "/tf:=tf", "-r", "/tf_static:=tf_static"])
        self.ac = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tfl = TransformListener(self.tf, self, spin_thread=True)
        self.odom = None
        self.create_subscription(Odometry, "odom", self._od, 10)
    def _od(self, msg):
        p = msg.pose.pose
        self.odom = (p.position.x, p.position.y)
    def map_pose(self):
        try:
            t = self.tf.lookup_transform("map", "sim01/base_footprint", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None

rclpy.init()
n = N()
t0 = time.time()
while time.time()-t0 < 20:
    rclpy.spin_once(n, timeout_sec=0.05)
    if n.odom is not None:
        break
print(f"START odom={n.odom} map={n.map_pose()} goal={GOAL_CODE} ({goal_ep['x']},{goal_ep['y']})", flush=True)
if not n.ac.wait_for_server(timeout_sec=20.0):
    print("NO_SERVER"); raise SystemExit(1)
g = NavigateToPose.Goal()
g.pose = PoseStamped()
g.pose.header.frame_id = "map"
g.pose.header.stamp = n.get_clock().now().to_msg()
g.pose.pose.position.x = float(goal_ep["x"])
g.pose.pose.position.y = float(goal_ep["y"])
yaw = float(goal_ep.get("yaw") or 0.0)
g.pose.pose.orientation.z = math.sin(yaw/2)
g.pose.pose.orientation.w = math.cos(yaw/2)
send = n.ac.send_goal_async(g)
t_deadline = time.time() + 180.0
while not send.done() and time.time() < t_deadline:
    rclpy.spin_once(n, timeout_sec=0.05)
if not send.done() or not send.result().accepted:
    print("REJECTED"); raise SystemExit(2)
fut = send.result().get_result_async()
odom0 = n.odom
while not fut.done() and time.time() < t_deadline:
    rclpy.spin_once(n, timeout_sec=0.05)
odom1 = n.odom
map1 = n.map_pose()
if not fut.done():
    try: send.result().cancel_goal_async()
    except Exception: pass
    status = "TIMEOUT"
else:
    wrap = fut.result()
    status = "PASS" if int(wrap.status)==4 else f"FAIL_status_{wrap.status}"
def dist(a,b):
    if not a or not b: return None
    return math.hypot(a[0]-b[0], a[1]-b[1])
tol = 0.20
gerr = dist(odom1,(goal_ep['x'],goal_ep['y']))
gt_pass = (gerr is not None and gerr <= tol)
print(f"END status={status} odom0={odom0} odom1={odom1} odom_disp={dist(odom0,odom1)} map1={map1} goal_err_odom={gerr} GT_ARRIVAL={'PASS' if gt_pass else 'FAIL'} tol={tol}", flush=True)
n.destroy_node(); rclpy.shutdown()
PY

sleep 2
kill "$TRACE_PID" 2>/dev/null
wait "$TRACE_PID" 2>/dev/null

# Analyze GT
python3 - <<'PY' | tee "$ART/gt_analysis.txt"
import csv, math
from pathlib import Path
p = Path("/mnt/e/programming/my_project/warotrans-system/docs/audits/gz_physical_motion/gt_trace.csv")
rows = list(csv.DictReader(p.open(encoding="utf-8")))
def series(key):
    out=[]
    for r in rows:
        if r.get(key):
            try: out.append((float(r["wall_t"]), float(r[key.replace("_x","_x") if False else key]), float(r[key[:-1]+"y" if key.endswith("x") else "0"])))
            except Exception: pass
    return out
# simpler
odom=[]; mmap=[]; gz=[]; gated_nz=0; safe_nz=0; nav_nz=0
for r in rows:
    if r["odom_x"]:
        odom.append((float(r["wall_t"]), float(r["odom_x"]), float(r["odom_y"])))
    if r["map_x"]:
        mmap.append((float(r["wall_t"]), float(r["map_x"]), float(r["map_y"])))
    if r["gz_x"]:
        gz.append((float(r["wall_t"]), float(r["gz_x"]), float(r["gz_y"])))
    if abs(float(r["cmd_gated_vx"]))>1e-4 or abs(float(r["cmd_gated_wz"]))>1e-4: gated_nz+=1
    if abs(float(r["cmd_safe_vx"]))>1e-4 or abs(float(r["cmd_safe_wz"]))>1e-4: safe_nz+=1
    if abs(float(r["cmd_nav_vx"]))>1e-4 or abs(float(r["cmd_nav_wz"]))>1e-4: nav_nz+=1

def net(seq):
    if len(seq)<2: return 0.0
    return math.hypot(seq[-1][1]-seq[0][1], seq[-1][2]-seq[0][2])
def pathlen(seq):
    s=0.0
    for i in range(1,len(seq)):
        s+=math.hypot(seq[i][1]-seq[i-1][1], seq[i][2]-seq[i-1][2])
    return s
print(f"rows={len(rows)}")
print(f"odom_net_m={net(odom):.4f} odom_path_m={pathlen(odom):.4f} n={len(odom)}")
print(f"map_net_m={net(mmap):.4f} map_path_m={pathlen(mmap):.4f} n={len(mmap)}")
print(f"gz_net_m={net(gz):.4f} gz_path_m={pathlen(gz):.4f} n={len(gz)} (gz model -p samples)")
print(f"cmd_nz nav={nav_nz} safe={safe_nz} gated={gated_nz}")
if odom:
    print(f"odom_start={odom[0][1:]} odom_end={odom[-1][1:]}")
if mmap:
    print(f"map_start={mmap[0][1:]} map_end={mmap[-1][1:]}")
# physical classification uses odom (GZ OdometryPublisher) as primary if gz model -p empty
primary = odom if odom else gz
phys = net(primary)
print("PHYSICAL_CLASS=" + ("MOVED" if phys >= 0.15 else "PHYSICALLY_STATIONARY"))
print(f"PRIMARY_NET_M={phys:.4f}")
PY

echo P0_PHYSICAL_VERIFY_DONE
