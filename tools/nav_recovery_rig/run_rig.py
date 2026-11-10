#!/usr/bin/env python3
"""Real Nav2 (same launch and config as tools/pi_run_semantic.sh) against a simulated base, one stack per scenario.

    wsl -e bash -lc 'python3 tools/nav_recovery_rig/run_rig.py --label baseline'
    ... --only nose_05,side_05 --deadband-w 0.2 --bt /path/to/tree.xml

Never touches the robot: ROS_LOCALHOST_ONLY=1 plus a private ROS_DOMAIN_ID per scenario; no MQTT, no backend, no DB.
Prints a table and writes <ws>/results/<label>/summary.json. Any chassis collision fails the run (exit code 1).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from nav_stuck_probe import summarize  # noqa: E402

WS = Path(os.environ.get("WT_RIG_WS", Path.home() / ".cache/warotrans_rig_ws"))
BUNDLE = WS / "src/warotrans_navigation/maps/map_one_v002"
CONFIG = WS / "src/warotrans_navigation/config"
WALL_X = -1.473  # free space starts here: west wall of MAP-ONE-V002 at y = -0.18
FLOOR_Y = -1.356  # free space starts here: south wall near the west corner
PARKING_A = (1.244, -0.997, math.pi / 2)  # EP-PARKING-A, MAP-ONE-V002
OPEN_Y = -0.18


@dataclass
class Scenario:
    name: str
    x: float
    y: float
    yaw: float
    goal: tuple = PARKING_A
    obstacles: list = field(default_factory=list)
    authority: str = "ok"
    recovery: str = "ok"
    recovery_backup: str = ""
    timeout: float = 240.0
    note: str = ""


def scenarios() -> list[Scenario]:
    out = []
    for g in (2, 5, 10, 20):
        out.append(Scenario(f"nose_{g:02d}", WALL_X + 0.16 + g / 100, OPEN_Y, math.pi, note=f"nose {g} cm from wall, goal behind"))
    for g in (1, 5, 10):
        out.append(Scenario(f"side_{g:02d}", WALL_X + 0.13 + g / 100, OPEN_Y, math.pi / 2, note=f"left side {g} cm from wall, goal to the right"))
    out.append(Scenario("corner_05", WALL_X + 0.25, FLOOR_Y + 0.25, -3 * math.pi / 4, note="nose into a corner, front corners ~4.5 cm"))
    # Virtual walls appear in the scan only: not on the static map the global planner sees.
    x0 = 0.0
    out.append(Scenario(
        "dead_end", x0, OPEN_Y, math.pi,
        obstacles=[[x0 - 0.32, OPEN_Y - 0.25, x0 - 0.21, OPEN_Y + 0.25],
                   [x0 - 0.32, OPEN_Y + 0.18, x0 + 0.40, OPEN_Y + 0.25],
                   [x0 - 0.32, OPEN_Y - 0.25, x0 + 0.40, OPEN_Y - 0.18]],
        note="nose 5 cm, both sides 5 cm, open only straight behind",
    ))
    out.append(Scenario(
        "rear_blocked", WALL_X + 0.21, OPEN_Y, math.pi,
        obstacles=[[WALL_X + 0.42, OPEN_Y - 0.20, WALL_X + 0.50, OPEN_Y + 0.20]],
        note="nose 5 cm, obstacle 5 cm behind: must not hit anything",
    ))
    out.append(Scenario("gate_closed", WALL_X + 0.21, OPEN_Y, math.pi, authority="closed", timeout=120.0, note="no authority: zero motion"))
    out.append(Scenario("straight", 0.0, OPEN_Y, 0.0, goal=(1.244, OPEN_Y, 0.0), note="open floor, control case"))
    return out


# Real endpoints of MAP-ONE-V004 (read from the database): the legs a workflow would drive.
V004 = {
    "PARKING_A": (0.495, 3.722, math.pi), "PARKING_B": (0.465, 3.123, math.pi), "CHARGE": (0.444, 2.569, math.pi),
    "INBOUND": (2.607, 0.078, math.pi), "OUTBOUND": (0.081, 0.046, 0.0), "QUEUE": (2.068, 0.503, -math.pi / 2),
    "SHELF_A": (2.409, 3.158, 0.0), "SHELF_B1": (2.195, 1.113, 0.0), "SHELF_B2": (2.611, 1.113, math.pi / 2),
    "SHELF_C": (0.087, 1.088, -math.pi / 2),
}


def v004_scenarios() -> list[Scenario]:
    legs = [("PARKING_A", "INBOUND"), ("INBOUND", "SHELF_A"), ("INBOUND", "SHELF_B1"), ("INBOUND", "SHELF_B2"),
            ("INBOUND", "SHELF_C"), ("SHELF_A", "OUTBOUND"), ("SHELF_B2", "QUEUE"), ("OUTBOUND", "PARKING_A"),
            ("CHARGE", "PARKING_B")]
    out = []
    for a, b in legs:
        x, y, yaw = V004[a]
        out.append(Scenario(f"{a}__{b}".lower(), x, y, yaw, goal=V004[b], timeout=300.0, note=f"{a} -> {b}, MAP-ONE-V004"))
    return out


class Stack:
    """All processes of one scenario, in one process group each, stopped together."""

    def __init__(self, domain: int, out: Path) -> None:
        self.env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY="1", PYTHONUNBUFFERED="1",
                        RCUTILS_COLORIZED_OUTPUT="0")
        self.out = out
        self.procs: list[subprocess.Popen] = []

    def start(self, name: str, argv: list[str]) -> None:
        log = open(self.out / f"{name}.log", "wb")
        script = f"source /opt/ros/jazzy/setup.bash; source {shlex.quote(str(WS))}/install/setup.bash; exec " + shlex.join(argv)
        self.procs.append(subprocess.Popen(["bash", "-c", script], env=self.env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))

    def stop(self) -> None:
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            for p in self.procs:
                if p.poll() is None:
                    try:
                        os.killpg(p.pid, sig)
                    except ProcessLookupError:
                        pass
            deadline = time.time() + 6
            while time.time() < deadline and any(p.poll() is None for p in self.procs):
                time.sleep(0.2)


GOAL_SCRIPT = r"""
import json, math, sys, time
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
x, y, yaw, timeout = [float(v) for v in sys.argv[1:5]]
rclpy.init(); n = rclpy.create_node("rig_goal"); c = ActionClient(n, NavigateToPose, "navigate_to_pose")
deadline = time.time() + 90
while not c.wait_for_server(timeout_sec=1.0):
    if time.time() > deadline: print(json.dumps({"status": "NAV2_NOT_READY"})); sys.exit(3)
g = NavigateToPose.Goal(); g.pose.header.frame_id = "map"
g.pose.pose.position.x, g.pose.pose.position.y = x, y
g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
accepted = None
for _ in range(45):
    g.pose.header.stamp = n.get_clock().now().to_msg()
    f = c.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
    accepted = f.result()
    if accepted is not None and accepted.accepted: break
    time.sleep(2.0)
if accepted is None or not accepted.accepted: print(json.dumps({"status": "GOAL_REJECTED"})); sys.exit(3)
t0 = time.time(); r = accepted.get_result_async()
while not r.done() and time.time() - t0 < timeout:
    rclpy.spin_once(n, timeout_sec=0.5)
if r.done():
    res = r.result()
    names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
    print(json.dumps({"status": names.get(res.status, str(res.status)), "t0": t0, "t1": time.time(), "error_code": getattr(res.result, "error_code", None)}))
else:
    accepted.cancel_goal_async(); print(json.dumps({"status": "STILL_RUNNING_AT_TIMEOUT", "t0": t0, "t1": time.time()}))
"""


def wait_for_truth(path: Path, timeout: float) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if path.exists() and path.stat().st_size > 400:
            return True
        time.sleep(0.5)
    return False


def invalid(s: Scenario, why: str, args: argparse.Namespace) -> dict:
    return {"scenario": s.name, "note": s.note, "result": "INVALID", "invalid_reasons": [why], "error_code": None, "duration_s": None,
            "first_motion_s": None, "travel_m": None, "turned_rad": None, "start_gap_m": None, "min_gap_m": None,
            "end_gap_m": None, "collisions": 0, "goal_dist_m": None, "goal_yaw_err": None,
            "probe": {"blocked_s": {}, "actions": {}}, "deadband": [args.deadband_v, args.deadband_w]}


def validity_problems(s: Scenario, result: dict, run: list, samples: list, nav_log: str) -> list[str]:
    """A rig that stalled proves nothing about Nav2: such a run is thrown away, not reported."""
    out = []
    if result.get("status") in (None, "NAV2_NOT_READY", "GOAL_REJECTED", "NO_RESULT"):
        out.append(f"stack did not come up: {result.get('status')}")
    if not run or not samples:
        return out + ["no data"]
    steps = sorted(b["wall"] - a["wall"] for a, b in zip(run, run[1:]))
    if steps and steps[int(len(steps) * 0.99)] > 0.4:
        out.append(f"base stalled: p99 tick {steps[int(len(steps) * 0.99)]:.2f}s")
    if s.authority == "ok":
        seen = [x for x in samples if x.get("state", {}).get("authority_ok") is True]
        if len(seen) < 0.95 * len(samples):
            out.append(f"authority_ok seen in only {len(seen)}/{len(samples)} samples")
    if nav_log.count("Filter mask was not received") > 3:
        out.append("costmap filter masks not loaded")
    # The global costmap has not received /map yet: an abort here says nothing about recovery.
    if "Costmap timed out" in nav_log or "out of map bounds (0.00, 0.00)" in nav_log:
        out.append("global costmap had no map")
    return out


def run_one(s: Scenario, domain: int, root: Path, args: argparse.Namespace) -> dict:
    for attempt in range(3):
        row = run_attempt(s, domain, root, args)
        if not row.get("invalid_reasons"):
            row["attempts"] = attempt + 1
            return row
        print(f"[{s.name}] attempt {attempt + 1} invalid: {row['invalid_reasons']}", flush=True)
    row["result"] = "INVALID"
    return row


def run_attempt(s: Scenario, domain: int, root: Path, args: argparse.Namespace) -> dict:
    out = root / s.name
    out.mkdir(parents=True, exist_ok=True)
    for old in out.iterdir():
        old.unlink()
    truth = out / "truth.jsonl"
    stack = Stack(domain, out)
    map_yaml = BUNDLE / "map.yaml"
    try:
        stack.start("base", [sys.executable, str(HERE / "rig_base.py"), "--map", str(map_yaml), "--truth", str(truth),
                             "--x", str(s.x), "--y", str(s.y), "--yaw", str(s.yaw), "--obstacles", json.dumps(s.obstacles),
                             "--deadband-v", str(args.deadband_v), "--deadband-w", str(args.deadband_w)])
        stack.start("gateway", [sys.executable, str(HERE / "rig_gateway.py"), "--authority", s.authority,
                                "--recovery", s.recovery, "--recovery-backup", s.recovery_backup])
        # Nav2 only after the simulated hardware runs: its costmaps fail activation without TF, and a late gateway
        # looks like a closed gate.
        if not wait_for_truth(truth, 90.0):
            return invalid(s, "base never published", args)
        time.sleep(2.0)
        stack.start("map_server", ["ros2", "run", "nav2_map_server", "map_server", "--ros-args", "-p", f"yaml_filename:={map_yaml}"])
        stack.start("map_lifecycle", ["ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager", "--ros-args",
                                      "-r", "__node:=lifecycle_manager_map", "-p", "autostart:=true", "-p", "node_names:=[map_server]"])
        extra = str(CONFIG / "nav2_params_lane_aware.yaml") + (f";{args.nav_overlay}" if args.nav_overlay else "")
        launch = ["ros2", "launch", "warotrans_navigation", "navigation.launch.py",
                  f"extra_params_file:={extra}", f"lanes_json:={BUNDLE / 'lanes.json'}",
                  "enable_costmap_filters:=true", f"keepout_mask:={BUNDLE / 'keepout.yaml'}", f"speed_mask:={BUNDLE / 'speed.yaml'}",
                  "enable_authority_gate:=true", "gate_output_topic:=cmd_vel_motor", "required_authority_publisher:=robot_gateway"]
        if args.bt:
            launch.append(f"default_nav_to_pose_bt_xml:={args.bt}")
        stack.start("nav2", launch)
        stack.start("probe", [sys.executable, str(HERE.parent / "nav_stuck_probe.py"), "record", str(out / "probe.jsonl")])
        goal_py = out / "goal.py"
        goal_py.write_text(GOAL_SCRIPT, encoding="utf-8")
        script = (f"source /opt/ros/jazzy/setup.bash; source {shlex.quote(str(WS))}/install/setup.bash; exec "
                  + shlex.join([sys.executable, str(goal_py), *map(str, s.goal), str(s.timeout)]))
        done = subprocess.run(["bash", "-c", script], env=stack.env, capture_output=True, text=True, timeout=s.timeout + 200)
        lines = done.stdout.strip().splitlines()
        result = json.loads(lines[-1]) if lines else {"status": "NO_RESULT", "stderr": done.stderr[-400:]}
        time.sleep(1.0)
    finally:
        stack.stop()
    return analyse(s, result, out, args)


def analyse(s: Scenario, result: dict, out: Path, args: argparse.Namespace) -> dict:
    truth = [json.loads(x) for x in (out / "truth.jsonl").read_text().splitlines() if x.strip()] if (out / "truth.jsonl").exists() else []
    t0 = result.get("t0") or (truth[0]["wall"] if truth else 0)
    run = [t for t in truth if t["wall"] >= t0]
    start = run[0] if run else None
    first_move = next((round(t["wall"] - t0, 1) for t in run if start and (t["travel"] - start["travel"] > 0.005 or t["turned"] - start["turned"] > 0.02)), None)
    probe_lines = []
    if (out / "probe.jsonl").exists():
        probe_lines = [json.loads(x) for x in (out / "probe.jsonl").read_text().splitlines() if x.strip()]
    in_run = [ln for ln in probe_lines if ln.get("wall", 0) >= t0]
    probe = summarize(in_run)
    nav_log = (out / "nav2.log").read_text(errors="replace") if (out / "nav2.log").exists() else ""
    problems = validity_problems(s, result, run, [ln for ln in in_run if ln.get("t") == "s"], nav_log)
    last = run[-1] if run else None
    return {
        "scenario": s.name, "note": s.note, "result": result.get("status"), "error_code": result.get("error_code"),
        "invalid_reasons": problems,
        "duration_s": round(result["t1"] - result["t0"], 1) if "t1" in result else None,
        "first_motion_s": first_move,
        "travel_m": round(last["travel"] - start["travel"], 3) if last else None,
        "turned_rad": round(last["turned"] - start["turned"], 3) if last else None,
        "start_gap_m": start["gap"] if start else None,
        "min_gap_m": min(t["gap"] for t in run) if run else None,
        "end_gap_m": last["gap"] if last else None,
        "collisions": last["collisions"] if last else None,
        "goal_dist_m": round(math.hypot(last["x"] - s.goal[0], last["y"] - s.goal[1]), 3) if last else None,
        "goal_yaw_err": round(abs(math.atan2(math.sin(last["yaw"] - s.goal[2]), math.cos(last["yaw"] - s.goal[2]))), 3) if last else None,
        "probe": probe,
        "deadband": [args.deadband_v, args.deadband_w],
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--only", default="")
    ap.add_argument("--parallel", type=int, default=3)
    ap.add_argument("--deadband-v", type=float, default=0.0)
    ap.add_argument("--deadband-w", type=float, default=0.0)
    ap.add_argument("--bt", default="")
    ap.add_argument("--map", default="map_one_v002", help="bundle directory under warotrans_navigation/maps")
    ap.add_argument("--nav-overlay", default="", help="extra Nav2 params merged last (candidate values)")
    args = ap.parse_args()
    root = WS / "results" / args.label
    root.mkdir(parents=True, exist_ok=True)
    global BUNDLE
    BUNDLE = WS / "src/warotrans_navigation/maps" / args.map
    pool_of = v004_scenarios() if args.map == "map_one_v004" else scenarios()
    chosen = [s for s in pool_of if not args.only or s.name in args.only.split(",")]
    base_domain = os.getpid() % 80
    with ThreadPoolExecutor(args.parallel) as pool:
        rows = list(pool.map(lambda p: run_one(p[1], (base_domain + p[0]) % 80 + 20, root, args), enumerate(chosen)))
    (root / "summary.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    print(f"{'scenario':<24}{'result':<26}{'dur':>6}{'1st mv':>7}{'trav':>7}{'turn':>7}{'gap0':>7}{'min':>7}{'end':>7}{'coll':>5}{'dist':>7}  blocked_s / actions")
    bad = 0
    for r in rows:
        bad += 1 if (r["collisions"] or 0) > 0 or r["result"] == "INVALID" else 0
        print(f"{r['scenario']:<24}{str(r['result']):<26}{str(r['duration_s']):>6}{str(r['first_motion_s']):>7}{str(r['travel_m']):>7}{str(r['turned_rad']):>7}"
              f"{str(r['start_gap_m']):>7}{str(r['min_gap_m']):>7}{str(r['end_gap_m']):>7}{str(r['collisions']):>5}{str(r['goal_dist_m']):>7}  {r['probe']['blocked_s']}")
    print(f"results: {root}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
