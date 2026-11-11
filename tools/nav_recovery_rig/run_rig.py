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

sys.path.insert(0, str(HERE))
import rig_metrics  # noqa: E402

WS = Path(os.environ.get("WT_RIG_WS", Path.home() / ".cache/warotrans_rig_ws"))
BUNDLE = WS / "src/warotrans_navigation/maps/map_one_v002"
CONFIG = WS / "src/warotrans_navigation/config"
WALL_X = -1.473  # free space starts here: west wall of MAP-ONE-V002 at y = -0.18
FLOOR_Y = -1.356  # free space starts here: south wall near the west corner
PARKING_A = (1.244, -0.997, math.pi / 2)  # EP-PARKING-A, MAP-ONE-V002
OPEN_Y = -0.18
SPEED_CEILING_MPS = 0.105  # desired_linear_vel 0.10, smoother max 0.10, with room for the rounding of the log


@dataclass
class Scenario:
    name: str
    x: float
    y: float
    yaw: float
    goal: tuple = PARKING_A
    obstacles: list = field(default_factory=list)
    timed_obstacles: list = field(default_factory=list)
    authority: str = "ok"
    recovery: str = "ok"
    recovery_backup: str = ""
    deny_from_s: float = 0.0
    deny_for_s: float = 0.0
    expect: str = "SUCCEEDED"
    retry_abort_s: float = 0.0
    timeout: float = 240.0
    note: str = ""
    inertia_tau: float = 0.0


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
    """Workflow-style sample legs (subset)."""
    legs = [("PARKING_A", "INBOUND"), ("INBOUND", "SHELF_A"), ("INBOUND", "SHELF_B1"), ("INBOUND", "SHELF_B2"),
            ("INBOUND", "SHELF_C"), ("SHELF_A", "OUTBOUND"), ("SHELF_B2", "QUEUE"), ("OUTBOUND", "PARKING_A"),
            ("CHARGE", "PARKING_B")]
    out = []
    for a, b in legs:
        x, y, yaw = V004[a]
        out.append(Scenario(f"{a}__{b}".lower(), x, y, yaw, goal=V004[b], timeout=300.0, authority="semantic", note=f"{a} -> {b}, MAP-ONE-V004"))
    return out


# MAP-ONE-V006 = V005 geometry with ZONE-SPEED-ZONE-BOUND at 0.06 m/s (snapshot samples/maps/MAP-ONE-V006).
V006 = {
    "PARKING_A": (-0.541, 2.468, math.pi), "PARKING_B": (-0.52, 1.775, math.pi), "CHARGE": (-0.911, 3.098, 0.0),
    "INBOUND": (1.459, -1.043, math.pi), "OUTBOUND": (-0.953, -1.048, 0.0), "QUEUE": (0.938, -0.751, -math.pi / 2),
    "INSPECTION": (-0.953, 0.197, 0.0), "SHELF_A": (1.36, 1.994, math.pi / 2), "SHELF_B1": (1.573, -0.085, math.pi / 2),
    "SHELF_B2": (1.219, -0.085, 0.0), "SHELF_C": (-0.953, -0.111, 0.0),
}


def v006_scenarios() -> list[Scenario]:
    """The INBOUND run of the 2026-11-10 lab, leg by leg, then the ways back to rest (parking, else the queue point)."""
    legs = [("PARKING_A", "INBOUND"), ("INBOUND", "SHELF_B1"), ("SHELF_B1", "SHELF_A"), ("SHELF_A", "SHELF_C"),
            ("SHELF_C", "PARKING_A"), ("SHELF_C", "PARKING_B"), ("SHELF_C", "QUEUE")]
    out = []
    for a, b in legs:
        x, y, yaw = V006[a]
        out.append(Scenario(f"{a}__{b}".lower(), x, y, yaw, goal=V006[b], timeout=300.0, authority="semantic", note=f"{a} -> {b}, MAP-ONE-V006"))
    return out


def v004_all_pairs() -> list[Scenario]:
    """All directed endpoint pairs on MAP-ONE-V004 (10×9 = 90)."""
    names = list(V004.keys())
    out = []
    for a in names:
        for b in names:
            if a == b:
                continue
            x, y, yaw = V004[a]
            out.append(Scenario(
                f"{a}__{b}".lower(), x, y, yaw, goal=V004[b], timeout=480.0, authority="semantic",
                note=f"{a} -> {b}, MAP-ONE-V004 all-pairs",
            ))
    return out


def obstacle_scenarios() -> list[Scenario]:
    """Step 5 of the lab plan, through the real SemanticTrafficClient. Like MoveCommandHandler, the goal is re-sent when
    Nav2 aborts, for 60 s without progress; only then does the run report ABORTED (backend: NAVIGATION_BLOCKED, job PAUSED).
    Times count from the goal. Leg INBOUND -> SHELF_A goes up BTT-C (x 1.19) through the aisle y 0.9..1.2, x -0.19..1.51."""
    start, goal = V004["INBOUND"], V004["SHELF_A"]
    # The row y = 1.2, wall to wall. The aisle at x < 1.52 alone would do: nothing else connects the south to the north.
    barrier = [-0.20, 1.15, 2.92, 1.25]
    common = dict(goal=goal, timeout=420.0, authority="semantic", retry_abort_s=60.0)
    return [
        Scenario("obs_road_closed", *start, obstacles=[[0.80, 2.95, 1.52, 3.05]], expect="ABORTED", **common,
                 note="the road of BTT-A is shut far ahead. V004 has no legal way round a closed lane: ONE_WAY lanes and keep-out leave none"),
        Scenario("obs_box_on_lane", *start, obstacles=[[1.08, 0.90, 1.28, 1.10]], expect="ABORTED", **common,
                 note="0.22 m left in the lane, the body is 0.30 m and the opposite lane is ONE_WAY: no legal way round"),
        Scenario("obs_partial", *start, obstacles=[[1.33, 0.95, 1.51, 1.05]], **common,
                 note="box on the edge of the lane: pass without leaving the aisle"),
        Scenario("obs_clears_after_20s", *start, timed_obstacles=[[*barrier, 0.0, 20.0]], **common,
                 note="no path for 20 s, then free: wait and continue, job never paused"),
        Scenario("obs_stays", *start, timed_obstacles=[[*barrier, 0.0, -1.0]], expect="ABORTED", **common,
                 note="no path, ever: reported after the 60 s window, no motion through the barrier"),
        Scenario("obs_grant_denied_30s", *start, deny_from_s=10.0, deny_for_s=30.0, **common,
                 note="backend revokes and grants nothing for 30 s: gate closed, no motion, then continue"),
    ]


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
        self._kill_leftovers()

    def _kill_leftovers(self) -> None:
        """ros2 launch children outlive their group now and then. They stay in this private domain, burning CPU, and a
        stalled base is what makes later runs INVALID. Identified by environment: nothing but this stack has it."""
        wanted = f"ROS_DOMAIN_ID={self.env['ROS_DOMAIN_ID']}".encode()
        for pid in (x for x in os.listdir("/proc") if x.isdigit() and int(x) != os.getpid()):
            try:
                env = Path(f"/proc/{pid}/environ").read_bytes().split(b"\0")
                if wanted in env and b"ROS_LOCALHOST_ONLY=1" in env and any(b"warotrans_rig_ws" in e for e in env):
                    os.kill(int(pid), signal.SIGKILL)
            except (OSError, ProcessLookupError):
                continue


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
open(sys.argv[5], "w").close()
t0 = time.time(); r = accepted.get_result_async(); aborts = []
retry = float(sys.argv[6])
while time.time() - t0 < timeout:
    while not r.done() and time.time() - t0 < timeout:
        rclpy.spin_once(n, timeout_sec=0.5)
    if r.done() and r.result().status == 6 and time.time() - t0 < retry:
        aborts.append(getattr(r.result().result, "error_code", None))
        g.pose.header.stamp = n.get_clock().now().to_msg()
        f = c.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=5.0)
        accepted = f.result(); r = accepted.get_result_async()
        continue
    break
if r.done():
    res = r.result()
    names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
    print(json.dumps({"status": names.get(res.status, str(res.status)), "t0": t0, "t1": time.time(), "error_code": getattr(res.result, "error_code", None), "aborts": aborts}))
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
    # A goal planned before the mask arrives runs into the backend's keep-out reject: true, but not what this run measures.
    if rig_metrics.log_lines_after(nav_log, "Filter mask was not received", result.get("t0") or 0.0) > 0:
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


def nav2_goal(goal: tuple) -> tuple:
    """The gateway's goal offset: Nav2 drives base_footprint, the endpoint is for the chassis centre."""
    off = rig_metrics.geometry_body_offset()
    x, y, yaw = goal
    return (x - off * math.cos(yaw), y - off * math.sin(yaw), yaw)


def run_attempt(s: Scenario, domain: int, root: Path, args: argparse.Namespace) -> dict:
    out = root / s.name
    out.mkdir(parents=True, exist_ok=True)
    for old in out.iterdir():
        old.unlink()
    truth = out / "truth.jsonl"
    stack = Stack(domain, out)
    map_yaml = BUNDLE / "map.yaml"
    try:
        base_argv = [sys.executable, str(HERE / "rig_base.py"), "--map", str(map_yaml), "--truth", str(truth),
                     "--x", str(s.x), "--y", str(s.y), "--yaw", str(s.yaw), "--obstacles", json.dumps(s.obstacles),
                     "--timed-obstacles", json.dumps(s.timed_obstacles), "--clock-file", str(out / "goal_accepted"),
                     "--inertia-tau", str(s.inertia_tau if s.inertia_tau else args.inertia_tau),
                     "--deadband-v", str(args.deadband_v), "--deadband-w", str(args.deadband_w)]
        stack.start("base", base_argv)
        auth = args.authority if args.authority else s.authority
        gw = [sys.executable, str(HERE / "rig_gateway.py"), "--authority", auth,
              "--recovery", s.recovery, "--recovery-backup", s.recovery_backup,
              "--bundle", str(BUNDLE), "--events", str(out / "gateway.jsonl"), "--lane-half-width", str(args.lane_half_width),
              "--deny-from-s", str(s.deny_from_s), "--deny-for-s", str(s.deny_for_s)]
        stack.start("gateway", gw)
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
        # Record /plan, /cmd_vel_motor, /odom for post-mortem (abort/keepout analysis).
        stack.start("recorder", [sys.executable, str(HERE / "rig_recorder.py"), str(out / "trace.jsonl")])
        goal_py = out / "goal.py"
        goal_py.write_text(GOAL_SCRIPT, encoding="utf-8")
        script = (f"source /opt/ros/jazzy/setup.bash; source {shlex.quote(str(WS))}/install/setup.bash; exec "
                  + shlex.join([sys.executable, str(goal_py), *map(str, nav2_goal(s.goal)), str(s.timeout), str(out / "goal_accepted"), str(s.retry_abort_s)]))
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
    # Endpoints, lanes and the body metrics are about the chassis centre (cx, cy); the base turns about base_footprint.
    run = [dict(t, x=t.get("cx", t["x"]), y=t.get("cy", t["y"])) for t in truth if t["wall"] >= t0]
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
    events = []
    if (out / "gateway.jsonl").exists():
        events = [json.loads(x) for x in (out / "gateway.jsonl").read_text().splitlines() if x.strip()]
    keepout = json.loads((BUNDLE / "keepout.json").read_text()).get("zones", []) if (BUNDLE / "keepout.json").exists() else []
    lanes = json.loads((BUNDLE / "lanes.json").read_text()).get("lanes", []) if (BUNDLE / "lanes.json").exists() else []
    row = {
        "scenario": s.name, "note": s.note, "result": result.get("status"), "error_code": result.get("error_code"),
        "aborts": result.get("aborts"),
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
        "keepout_clearance_m": rig_metrics.keepout_clearance(run, keepout),
        "lanes": rig_metrics.lane_offsets(run, lanes),
        "travel_gate_closed_m": rig_metrics.travel_while_gate_closed(run, events),
        "max_speed_mps": round(max((abs(t["v"]) for t in run), default=0.0), 3),
        "opposite_lane_m": rig_metrics.opposite_lane_intrusion(run, lanes),
        "speed_zone": rig_metrics.speed_zone_excess(run, rig_metrics.speed_mask_reader(str(BUNDLE / "speed.yaml")))
        if (BUNDLE / "speed.yaml").exists() else {},
        "speed_ceiling_mps": SPEED_CEILING_MPS,
        "keepout_rejects": sum(1 for e in events if e.get("e") == "keepout_reject"),
        "gate_closed_reasons": sorted({e["reason"] for e in events if e.get("e") == "gate" and not e["ok"]}),
    }
    row["expected"] = s.expect
    if s.obstacles or s.timed_obstacles:
        row["lanes"] = {}  # going round something is the point of these runs
    row["failures"] = rig_metrics.failures(row)
    if row["keepout_rejects"]:
        row["failures"].append(f"backend rejected {row['keepout_rejects']} plans through keep-out")
    return row


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="run")
    ap.add_argument("--only", default="")
    ap.add_argument("--parallel", type=int, default=3)
    ap.add_argument("--deadband-v", type=float, default=0.0)
    ap.add_argument("--deadband-w", type=float, default=0.0)
    ap.add_argument("--bt", default="")
    ap.add_argument("--map", default="map_one_v004", help="bundle directory under warotrans_navigation/maps")
    ap.add_argument("--nav-overlay", default="", help="extra Nav2 params merged last (candidate values)")
    ap.add_argument("--lane-half-width", type=float, default=0.0, help="reserve lane rectangles, as in a two-robot run")
    ap.add_argument("--all-pairs", action="store_true", help="MAP-ONE-V004: all 90 directed endpoint pairs")
    ap.add_argument("--obstacle-suite", action="store_true", help="4 Step-5 obstacle / grant scenarios on V004")
    ap.add_argument("--authority", default="", help="override scenario authority: ok|closed|semantic")
    ap.add_argument("--inertia-tau", type=float, default=0.0, help="default base inertia tau for all scenarios")
    args = ap.parse_args()
    root = WS / "results" / args.label
    root.mkdir(parents=True, exist_ok=True)
    global BUNDLE
    BUNDLE = WS / "src/warotrans_navigation/maps" / args.map
    if args.obstacle_suite:
        pool_of = obstacle_scenarios()
        BUNDLE = WS / "src/warotrans_navigation/maps/map_one_v004"
    elif args.map == "map_one_v006":
        pool_of = v006_scenarios()
    elif args.map == "map_one_v004":
        pool_of = v004_all_pairs() if args.all_pairs else v004_scenarios()
    else:
        pool_of = scenarios()
    chosen = [s for s in pool_of if not args.only or s.name in args.only.split(",")]
    base_domain = os.getpid() % 80
    with ThreadPoolExecutor(args.parallel) as pool:
        rows = list(pool.map(lambda p: run_one(p[1], (base_domain + p[0]) % 80 + 20, root, args), enumerate(chosen)))
    (root / "summary.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    for r in rows:
        r.setdefault("failures", [])
        if r["result"] != r.get("expected", "SUCCEEDED"):
            r["failures"].append(f"expected {r.get('expected', 'SUCCEEDED')}")
    (root / "summary.json").write_text(json.dumps(rows, indent=1), encoding="utf-8")
    print(f"{'scenario':<24}{'result':<26}{'dur':>6}{'1st mv':>7}{'trav':>7}{'turn':>7}{'gap0':>7}{'min':>7}{'end':>7}{'coll':>5}{'dist':>7}  blocked_s / actions")
    bad = 0
    for r in rows:
        bad += 1 if r["failures"] or r["result"] == "INVALID" else 0
        print(f"{r['scenario']:<24}{str(r['result']):<26}{str(r['duration_s']):>6}{str(r['first_motion_s']):>7}{str(r['travel_m']):>7}{str(r['turned_rad']):>7}"
              f"{str(r['start_gap_m']):>7}{str(r['min_gap_m']):>7}{str(r['end_gap_m']):>7}{str(r['collisions']):>5}{str(r['goal_dist_m']):>7}  {r['probe']['blocked_s']}")
        if r["failures"]:
            print(f"    FAIL: {'; '.join(r['failures'])}")
    print(f"{len(rows) - bad}/{len(rows)} met every criterion")
    print(f"results: {root}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
