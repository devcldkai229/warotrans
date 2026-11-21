"""Two simulated robots share MAP-ONE-V013 (the map RBT-001 and RBT-002 run): same request and separate requests,
head-on in the INBOUND/OUTBOUND corridor, an idle robot in the way. Wait point = queue point = recovery point.

Real Host (develop build, RealRobot traffic settings: Semantic, prefix grants, 0.10 m cells, no auto return to park) +
real Mosquitto + the robot gateway's own traffic logic (SimRobot). Isolated: database warotrans_test, topic tree
warotrans-e2e/v1, robot codes E2E-*. Robots follow A* paths on the V013 grid (walls, inflated keepout, one-way lanes)
at RBT-001's and RBT-002's cruise speeds. Staff confirm every hand-over the way the app does (claim, begin, scan).

Every scenario is watched the whole time (Watch, 20 Hz):
  * traffic never gives a resource to more robots than it holds;
  * no robot moves while its cmd_vel gate is closed (no motion without authority);
  * the two robot bodies (0.32 x 0.26 m, at their real headings) never touch: the smallest gap is reported;
  * every stop of 2 s or more is classified: at an endpoint, at a wait point, or on the way (lane / no-stop zone).

Run on its own (it owns port 5199 and the coordinator lease of warotrans_test):
    WAROTRANS_SYSTEM_REPO=<system checkout> python -m pytest tools/traffic_e2e/test_multi_robot_waitpoints_v013.py -s
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

import pytest

from harness import PREFIX, ROBOT_REPO, Api, Host, wait_for
from map_paths import MapPaths
from sim_robot import SimRobot

HERE = Path(__file__).resolve().parent
ARTIFACTS = HERE / "artifacts" / "waitpoints_v013"
BUNDLE = ROBOT_REPO / "ros2_ws" / "src" / "warotrans_navigation" / "maps" / "map_one_v013"
MANIFEST = json.loads((BUNDLE / "semantic_manifest.json").read_text(encoding="utf-8"))
ZONES = json.loads((BUNDLE / "single_occupancy.json").read_text(encoding="utf-8"))["zones"]
NO_STOP = json.loads((BUNDLE / "no_stop.json").read_text(encoding="utf-8"))["zones"]
MAP_ID, MAP_CODE, FINGERPRINT = MANIFEST["mapVersionId"], MANIFEST["mapCode"], MANIFEST["semanticMapFingerprint"]
WORKFLOW = "E2E_INBOUND_TRIPS"  # the real INBOUND steps, without the voucher rule tied to the INBOUND code
RUN = uuid.uuid4().hex[:4].upper()
SPEED = {"R1": 0.35, "R2": 0.30}  # RBT-001 / RBT-002 cruise
BODY = (0.32, 0.26)  # length, width (manifest footprint)
PATHS = MapPaths(BUNDLE)
ROBOTS: list[SimRobot] = []
WAIT_POINT_CODES = {"EP-WAIT"}  # V013: "Wait" (EP-WAIT*, EP-QUEUE*, EP-RECOVERY*, names with wait/queue/recovery)


@pytest.fixture(scope="module")
def host():
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    seed = HERE / "seed_test_db.py"
    for args in (["--cleanup-e2e-robots"], ["--map-code", MAP_CODE, "--workflow-code", "INBOUND", "--workflow-alias", WORKFLOW]):
        done = subprocess.run([sys.executable, str(seed), *args], capture_output=True, text=True)
        assert done.returncode == 0, done.stderr
    h = Host(ARTIFACTS, name="host-waitpoints", extra_env={
        # = launchSettings "RealRobot" (what the lab Host runs), minus hardware.
        "Traffic__SemanticMapDirectory": str(BUNDLE),
        "Traffic__SemanticHorizonMeters": "4.0",
        "Traffic__SemanticPrefixGrant": "true",
        "Traffic__SemanticCellSizeMeters": os.environ.get("WARO_E2E_CELL", "0.10"),
        f"Traffic__RobotHomeEndpoints__E2E-{RUN}-R1": "EP-PARKING-A",
        f"Traffic__RobotHomeEndpoints__E2E-{RUN}-R2": "EP-PARKING-B",
        "FleetIdle__AutoReturnToPark": os.environ.get("WARO_E2E_AUTO_PARK", "false"),
        "FleetIdle__IdleSeconds": "3",
        "FleetIdle__FallbackEndpointCodes": "EP-QUEUE-POINT",
        "JobPlanning__MaxContainersPerJob": "3",
    })
    h.start()
    yield h
    for r in ROBOTS:
        r.stop()
    h.stop()


@pytest.fixture(scope="module")
def api(host):
    a = Api()
    a.login()
    return a


def psql(sql: str) -> list[list[str]]:
    out = subprocess.run(["docker", "exec", "warotrans-postgres", "psql", "-U", "warotrans", "-d", "warotrans_test",
                          "-t", "-A", "-F", "|", "-c", sql], capture_output=True, text=True, check=True)
    return [l.split("|") for l in out.stdout.strip().splitlines() if l]


def endpoints() -> dict[str, tuple[str, float, float, float]]:
    return {c: (i, float(x), float(y), float(w)) for c, i, x, y, w in
            psql(f"select code, id, x, y, yaw from navigation.endpoints where map_version_id = '{MAP_ID}'")}


def warehouse() -> str:
    return psql(f"select warehouse_id from navigation.map_versions where id = '{MAP_ID}'")[0][0]


def robot(api, warehouse_id: str, name: str, at: tuple[str, float, float, float]) -> SimRobot:
    code = f"E2E-{RUN}-{name}"
    rid = api.call("POST", "/api/fleet/robots", {
        "warehouseId": warehouse_id, "currentMapVersionId": MAP_ID, "code": code, "name": code})["id"]
    api.call("PUT", f"/api/fleet/robots/{rid}/capacity", {"maxContainers": 3, "lengthM": BODY[0], "widthM": BODY[1]})
    sim = SimRobot(code=code, x=at[1], y=at[2], fingerprint=FINGERPRINT, zones=ZONES, prefix=PREFIX, map_code=MAP_CODE,
                   speed=SPEED[name], progress_timeout_sec=60.0, path_provider=PATHS.provider())
    sim.yaw = at[3]
    sim.start()
    ROBOTS.append(sim)
    wait_for(lambda: api.call("GET", f"/api/fleet/robots/{rid}").get("isOnline"), 20, what=f"{code} online")
    wait_for(lambda: api.call("GET", f"/api/fleet/robots/{rid}").get("localizationStatus") == "LOCALIZED", 20,
             what=f"{code} localized")
    api.call("POST", f"/api/fleet/robots/{rid}/put-in-service", {})
    sim.robot_id = rid  # type: ignore[attr-defined]
    sim.name = name  # type: ignore[attr-defined]
    return sim


@pytest.fixture(scope="module")
def pair(api):
    """R1 at RBT-001's home (EP-PARKING-A), R2 at RBT-002's (EP-PARKING-B)."""
    wh = warehouse()
    eps = endpoints()
    return wh, robot(api, wh, "R1", eps["EP-PARKING-A"]), robot(api, wh, "R2", eps["EP-PARKING-B"])


def request(api, warehouse_id: str, moves: list[tuple[str, str]], enqueue: bool = True) -> tuple[str, list[str]]:
    """One transport request, one container per (source, destination); queued, its first trip enqueued."""
    eps = endpoints()
    cat = api.call("POST", "/api/warehouse/product-categories", {"code": f"E2E-{RUN}-{uuid.uuid4().hex[:4]}", "name": "E2E"})["id"]
    prod = api.call("POST", "/api/warehouse/products", {"categoryId": cat, "sku": f"E2E-{uuid.uuid4().hex[:8]}", "name": "E2E box"})["id"]
    containers = [api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"] for _ in moves]
    wf = next(w for w in api.call("GET", "/api/workflow-execution/workflows?pageSize=100")["items"] if w["code"] == WORKFLOW)["id"]
    tr = api.call("POST", "/api/transportation/transport-requests", {
        "warehouseId": warehouse_id, "mapVersionId": MAP_ID, "workflowId": wf, "workflowInputs": {}, "note": f"E2E waitpoints {RUN}",
        "details": [{"sequenceNo": i + 1, "containerId": c, "sourceEndpointId": eps[s][0], "destinationEndpointId": eps[d][0]}
                    for i, (c, (s, d)) in enumerate(zip(containers, moves))]})["id"]
    jobs = [j["jobId"] for j in api.call("POST", f"/api/transportation/transport-requests/{tr}/queue", {})["jobs"]]
    if enqueue:
        for job in jobs:
            api.call("POST", f"/api/workflow-execution/jobs/{job}/enqueue", {}, ok=(200, 201, 202, 204, 409))
    return tr, jobs


def assignments(job_id: str) -> list[tuple[str, bool]]:
    return [(r, o == "t") for r, o in psql(
        f"select robot_id, ended_at is null from execution.job_robot_assignments where job_id = '{job_id}' order by started_at")]


def robot_free(rid: str) -> bool:
    return psql(f"select count(*) from execution.job_robot_assignments where robot_id = '{rid}' and ended_at is null")[0][0] == "0"


def terminal(states: dict[str, dict]) -> bool:
    return bool(states) and all(s["status"] in ("COMPLETED", "FAILED", "CANCELLED") for s in states.values())


# ---------------------------------------------------------------------- geometry


def _in_ring(x: float, y: float, ring: list[list[float]]) -> bool:
    inside = False
    for i in range(len(ring)):
        (x1, y1), (x2, y2) = ring[i - 1], ring[i]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def _corners(x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    hl, hw = BODY[0] / 2, BODY[1] / 2
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * a - s * b, y + s * a + c * b) for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def body_gap(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    """Separating-axis distance between two robot bodies: <= 0 means they overlap."""
    pa, pb = _corners(*a), _corners(*b)
    best = -math.inf
    for poly in (pa, pb):
        for i in range(4):
            ex, ey = poly[(i + 1) % 4][0] - poly[i][0], poly[(i + 1) % 4][1] - poly[i][1]
            n = math.hypot(ex, ey)
            ax, ay = -ey / n, ex / n
            ra = [px * ax + py * ay for px, py in pa]
            rb = [px * ax + py * ay for px, py in pb]
            best = max(best, max(min(rb) - max(ra), min(ra) - max(rb)))
    return best


class Watch:
    """Safety and stopping places, sampled at 20 Hz in the background; the main loop plays the floor staff."""

    def __init__(self, api, sims: list[SimRobot], name: str) -> None:
        self.api, self.sims, self.name = api, sims, name
        self.eps = endpoints()
        self.min_gap = math.inf
        self.min_gap_at: tuple = ()
        self.moved = {s.code: 0.0 for s in sims}
        self.both_moving_s = 0.0
        self.max_holders = 0
        self.capacity_breaches: list[str] = []
        self.stops: list[dict] = []
        self._still_since = {s.code: None for s in sims}
        self._last = {s.code: (s.x, s.y, time.time()) for s in sims}
        self.confirmations = 0
        self.notes: list[str] = []
        self.timeline: list[dict] = []
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._busy = {s.code: False for s in sims}
        self.both_busy_s = 0.0  # both robots had work at the same time (two trips in parallel)

    # -- background
    def start(self) -> "Watch":
        self._thread.start()
        return self

    def stop(self) -> None:
        if self._stop.is_set():
            return
        self._stop.set()
        self._thread.join(2)
        now = time.time()
        for s in self.sims:
            self._close_stop(s, now)

    def _place(self, x: float, y: float) -> str:
        code, d = min(((c, math.hypot(x - e[1], y - e[2])) for c, e in self.eps.items()), key=lambda t: t[1])
        if d <= 0.20:
            return f"{code}{' (wait point)' if code in WAIT_POINT_CODES else ''}"
        tags = [z["code"] for z in NO_STOP if _in_ring(x, y, z["rings"][0])]
        zones = [z["code"] for z in ZONES if _in_ring(x, y, z["rings"][0])]
        return f"on the way ({x:.2f}, {y:.2f}){' NO_STOP ' + ','.join(tags) if tags else ''}{' in ' + ','.join(zones) if zones else ''}"

    def _close_stop(self, s: SimRobot, now: float) -> None:
        since = self._still_since[s.code]
        if since is not None and now - since[0] >= 2.0:
            self.stops.append({"robot": s.code[-2:], "place": since[1], "seconds": round(now - since[0], 1),
                               "busy": since[2], "at": round(since[0], 1)})
        self._still_since[s.code] = None

    def _loop(self) -> None:
        tick = 0
        while not self._stop.wait(0.05):
            now = time.time()
            moving = []
            for s in self.sims:
                lx, ly, _ = self._last[s.code]
                d = math.hypot(s.x - lx, s.y - ly)
                self.moved[s.code] += d
                self._last[s.code] = (s.x, s.y, now)
                is_moving = d > 0.0005
                moving.append(is_moving)
                if is_moving:
                    self._close_stop(s, now)
                elif self._still_since[s.code] is None:
                    self._still_since[s.code] = (now, self._place(s.x, s.y), self._busy[s.code])
            if all(moving):
                self.both_moving_s += 0.05
            a, b = self.sims
            gap = body_gap((a.x, a.y, a.yaw), (b.x, b.y, b.yaw))
            if gap < self.min_gap:
                self.min_gap, self.min_gap_at = gap, ((round(a.x, 2), round(a.y, 2)), (round(b.x, 2), round(b.y, 2)))
            tick += 1
            if tick % 5 == 0:  # 4 Hz: traffic snapshot
                try:
                    snap = self.api.snapshot()
                except Exception:
                    continue
                for r in snap["resources"]:
                    self.max_holders = max(self.max_holders, len(r["holders"]))
                    if len(r["holders"]) > r["capacity"]:
                        self.capacity_breaches.append(f"{r['resourceId']}: {r['holders']}")

    # -- floor staff
    def _handover(self, stop: dict) -> bool:
        base = f"/api/workflow-execution/jobs/{stop['jobId']}/handover-stops/{stop['id']}"
        version = stop["version"]
        for action in ("claim", "begin"):
            allowed = {a["action"]: a["allowed"] for a in stop.get("availableActions", [])}
            if not allowed.get(action.upper(), False):
                continue
            stop = self.api.call("POST", f"{base}/{action}", {"moveId": stop["moveId"], "version": version})
            version = stop["version"]
        scans = [{"containerId": c["containerId"], "barcode": c["barcode"]} for c in stop["containers"]]
        self.api.call("POST", f"{base}/confirm", {"moveId": stop["moveId"], "version": version, "scans": scans})
        self.confirmations += 1
        return True

    def staff(self, jobs: list[str]) -> None:
        listing = self.api.call("GET", "/api/workflow-execution/handover-stops")
        for stop in listing.get("items", []):
            if stop["jobId"] in jobs and stop.get("robotReadyAt"):
                try:
                    self._handover(stop)
                except AssertionError as refused:  # a stop that moved on meanwhile: look again next round
                    self.notes.append(f"handover retry: {str(refused)[:160]}")

    def run_until(self, done, timeout: float, jobs: list[str], on_tick=None) -> dict[str, dict]:
        end = time.time() + timeout
        states: dict[str, dict] = {}
        still_since: float | None = None
        last_pos = None
        dumped = False
        try:
            while time.time() < end:
                pos = tuple((round(s.x, 3), round(s.y, 3)) for s in self.sims)
                if pos != last_pos:
                    still_since, last_pos, dumped = time.time(), pos, False
                elif still_since and time.time() - still_since > 20 and not dumped and states                         and any(v["status"] == "RUNNING" for v in states.values()):
                    (ARTIFACTS / f"{self.name}_stall_snapshot.json").write_text(json.dumps(self.api.snapshot(), indent=1), encoding="utf-8")
                    dumped = True
                if still_since and time.time() - still_since > 120 and states                         and any(v["status"] == "RUNNING" for v in states.values()):
                    raise AssertionError(f"STALLED: both robots still 120 s at {pos}; jobs "
                                         f"{ {k[:8]: (v['status'], v.get('blockingReason')) for k, v in states.items()} }")
                states = {j: self.api.call("GET", f"/api/workflow-execution/jobs/{j}") for j in jobs}
                for s in self.sims:
                    self._busy[s.code] = not robot_free(s.robot_id)  # type: ignore[attr-defined]
                if all(self._busy.values()) and self.timeline:
                    self.both_busy_s += time.time() - self.timeline[-1]["t"]
                self.staff(jobs)
                waits = {j[:8]: [st.get("outputValues", {}).get("waitingFor") for t in v["tasks"] for st in t["steps"]
                                 if st.get("outputValues", {}).get("waitingFor")] for j, v in states.items()}
                self.timeline.append({"t": round(time.time(), 2),
                                      **{s.code[-2:]: [round(s.x, 2), round(s.y, 2)] for s in self.sims},
                                      "jobs": {k[:8]: v["status"] for k, v in states.items()},
                                      "waits": {k: w for k, w in waits.items() if w}})
                if on_tick is not None:
                    on_tick(states)
                if done(states):
                    return states
                time.sleep(0.5)
            raise AssertionError(f"timed out; jobs: { {k[:8]: (v['status'], v.get('failureCode'), v.get('blockingReason')) for k, v in states.items()} }")
        finally:
            self.stop()
            self.save()

    def save(self) -> None:
        (ARTIFACTS / f"{self.name}.json").write_text(json.dumps({
            "minBodyGapM": round(self.min_gap, 3), "minGapAt": self.min_gap_at, "moved": self.moved,
            "bothMovingSeconds": round(self.both_moving_s, 1), "bothBusySeconds": round(self.both_busy_s, 1), "stops": self.stops, "notes": self.notes,
            "capacityBreaches": self.capacity_breaches, "timeline": self.timeline}, indent=1), encoding="utf-8")

    def assert_safe(self) -> None:
        assert not self.capacity_breaches, self.capacity_breaches[:5]
        for s in self.sims:
            assert not s.moved_while_gate_closed(), f"{s.code} moved without authority"
        assert self.min_gap > 0.0, f"bodies touched: gap {self.min_gap:.3f} m at {self.min_gap_at}"

    def report(self) -> str:
        lines = [f"[{self.name}] min body gap {self.min_gap:.3f} m at {self.min_gap_at}; moved "
                 f"{ {k[-2:]: round(v, 2) for k, v in self.moved.items()} } m; both moving {self.both_moving_s:.1f} s; "
                 f"both with work {self.both_busy_s:.1f} s; "
                 f"handovers {self.confirmations}"]
        for st in self.stops:
            lines.append(f"   stop {st['robot']} {st['seconds']:>5}s {'(job)' if st['busy'] else '(idle)'} {st['place']}")
        return "\n".join(lines)


def free_both(api, r1: SimRobot, r2: SimRobot) -> None:
    wait_for(lambda: robot_free(r1.robot_id) and robot_free(r2.robot_id)  # type: ignore[attr-defined]
             and all(api.call("GET", f"/api/fleet/robots/{r.robot_id}")["status"] == "AVAILABLE" for r in (r1, r2)),  # type: ignore[attr-defined]
             300, what="both robots free")


def finish(watch: Watch, states: dict[str, dict]) -> None:
    watch.stop()
    print("\n" + watch.report())
    watch.save()
    watch.assert_safe()
    assert all(s["status"] == "COMPLETED" for s in states.values()), \
        {k[:8]: (s["status"], s.get("failureCode"), s.get("blockingReason")) for k, s in states.items()}


def bad_stops(watch: Watch) -> list[dict]:
    """Stops of a robot with work: longer than 5 s inside a no-stop zone, or longer than 150 s anywhere (a retreat
    holds at most 90 s; anything longer is two robots waiting on each other)."""
    return [s for s in watch.stops if s["busy"] and ((s["seconds"] > 5 and "NO_STOP" in s["place"]) or s["seconds"] > 150)]


# ---------------------------------------------------------------------- 1 same request


def test_1_one_request_split_over_both_robots(api, pair):
    wh, r1, r2 = pair
    tr, jobs = request(api, wh, [("EP-INBOUND", d) for d in ("EP-SHELF-A-1", "EP-SHELF-A-2", "EP-SHELF-B-2", "EP-SHELF-C-1")])
    assert len(jobs) == 2, f"4 containers, at most 3 per trip: 2 trips, got {len(jobs)}"
    w = Watch(api, [r1, r2], "1_same_request").start()
    states = w.run_until(terminal, 600, jobs)
    finish(w, states)
    assert {assignments(j)[-1][0] for j in jobs} == {r1.robot_id, r2.robot_id}, "each robot carried a trip"  # type: ignore[attr-defined]
    assert api.call("GET", f"/api/transportation/transport-requests/{tr}")["status"] == "COMPLETED"
    assert w.confirmations >= 4, w.confirmations
    assert w.both_moving_s >= 5.0, f"both robots moved together only {w.both_moving_s:.1f} s"
    assert not bad_stops(w), bad_stops(w)


# ---------------------------------------------------------------------- 2 separate requests, separate areas


def test_2_two_requests_in_separate_areas_at_the_same_time(api, pair):
    wh, r1, r2 = pair
    free_both(api, r1, r2)
    _, jobs_a = request(api, wh, [("EP-SHELF-A-1", "EP-SHELF-A-2")])
    _, jobs_c = request(api, wh, [("EP-SHELF-C-1", "EP-OUTBOUND")])
    jobs = jobs_a + jobs_c
    w = Watch(api, [r1, r2], "2_separate_areas").start()
    states = w.run_until(terminal, 400, jobs)
    finish(w, states)
    assert {assignments(j)[-1][0] for j in jobs} == {r1.robot_id, r2.robot_id}  # type: ignore[attr-defined]
    # In parallel, not one after the other (one trip may be short: a robot already standing at its pickup).
    assert w.both_busy_s >= 10.0, f"both robots had work at the same time only {w.both_busy_s:.1f} s"
    assert w.both_moving_s > 0.5, f"never both moving ({w.both_moving_s:.1f} s)"
    assert not bad_stops(w), bad_stops(w)


# ---------------------------------------------------------------------- 3 head-on in the corridor


def test_3_two_requests_head_on_in_the_inbound_outbound_corridor(api, pair):
    """One request INBOUND -> OUTBOUND, the other OUTBOUND -> INBOUND: the two robots meet in the right-hand corridor.
    Traffic must keep them apart; if they lock, one backs off to a wait point (EP-WAIT / parking) and both finish."""
    wh, r1, r2 = pair
    free_both(api, r1, r2)
    _, jobs_x = request(api, wh, [("EP-INBOUND", "EP-OUTBOUND")])
    _, jobs_y = request(api, wh, [("EP-OUTBOUND", "EP-INBOUND")])
    jobs = jobs_x + jobs_y
    w = Watch(api, [r1, r2], "3_head_on_corridor").start()
    states = w.run_until(terminal, 600, jobs)
    finish(w, states)
    assert {assignments(j)[-1][0] for j in jobs} == {r1.robot_id, r2.robot_id}  # type: ignore[attr-defined]
    assert not bad_stops(w), bad_stops(w)


# ---------------------------------------------------------------------- 4 idle robot in the way


def test_4_an_idle_robot_standing_on_the_goal_makes_way(api, pair):
    """One robot finishes at EP-OUTBOUND and stays (no auto return to park); the next request delivers to EP-OUTBOUND
    with the other robot: the idle one must move off to a free wait point / parking place, the delivery completes."""
    wh, r1, r2 = pair
    free_both(api, r1, r2)
    _, first = request(api, wh, [("EP-SHELF-C-1", "EP-OUTBOUND")])
    w0 = Watch(api, [r1, r2], "4a_first_to_outbound").start()
    finish(w0, w0.run_until(terminal, 300, first))
    blocker = r1 if assignments(first[0])[-1][0] == r1.robot_id else r2  # type: ignore[attr-defined]
    other = r2 if blocker is r1 else r1
    eps = endpoints()
    assert math.hypot(blocker.x - eps["EP-OUTBOUND"][1], blocker.y - eps["EP-OUTBOUND"][2]) < 0.25, "blocker at OUTBOUND"
    free_both(api, r1, r2)
    # The pickup is next to the other robot, so the queue gives the trip to it.
    pickup = "EP-SHELF-A-1" if other.y > 0 else "EP-INSPECTION"
    _, second = request(api, wh, [(pickup, "EP-OUTBOUND")])
    w = Watch(api, [r1, r2], "4b_idle_blocker").start()
    states = w.run_until(terminal, 500, second)
    finish(w, states)
    assert assignments(second[0])[-1][0] == other.robot_id, "the robot that was not standing there took it"  # type: ignore[attr-defined]
    moved_off = math.hypot(blocker.x - eps["EP-OUTBOUND"][1], blocker.y - eps["EP-OUTBOUND"][2])
    assert moved_off > 0.3, f"the idle robot never made way ({moved_off:.2f} m from OUTBOUND)"
    print(f"idle robot {blocker.code[-2:]} made way to {w._place(blocker.x, blocker.y)}")


# ---------------------------------------------------------------------- 5 cancelled with cargo -> recovery point


def test_5_cancelled_with_cargo_the_robot_takes_it_to_a_recovery_point_while_the_other_robot_works_on(api, pair):
    """Request X (INBOUND -> SHELF-C-1) is cancelled while its robot carries the container; staff send the robot to a
    recovery point (wait point = queue point = recovery point). Request Y (SHELF-A-1 -> SHELF-A-2) runs on the other
    robot meanwhile and completes. Both robots stay apart the whole time."""
    wh, r1, r2 = pair
    free_both(api, r1, r2)
    tr_x, jobs_x = request(api, wh, [("EP-INBOUND", "EP-SHELF-C-1")])
    _, jobs_y = request(api, wh, [("EP-SHELF-A-1", "EP-SHELF-A-2")])
    jobs = jobs_x + jobs_y
    eps = endpoints()
    w = Watch(api, [r1, r2], "5_cancel_to_recovery").start()
    sent: dict[str, object] = {}

    def cancel_once_onboard(states: dict[str, dict]) -> None:
        x = states[jobs_x[0]]
        if sent or x["status"] != "RUNNING" or not any(c["status"] == "ONBOARD" for c in x["containers"]):
            return
        api.call("POST", f"/api/transportation/transport-requests/{tr_x}/cancel", {"reason": "e2e: customer cancelled"})
        points = api.call("GET", f"/api/transportation/transport-requests/{tr_x}/cancellation/jobs/{jobs_x[0]}/recovery-points")
        sent["points"] = points
        done = wait_for(lambda: _try_recovery(api, tr_x, jobs_x[0]), 60, interval=1.0, what="recovery accepted")
        sent["response"] = done
        sent["robot"] = assignments(jobs_x[0])[-1][0]

    def at_recovery(states: dict[str, dict]) -> bool:
        if not sent or states[jobs_y[0]]["status"] not in ("COMPLETED", "FAILED", "CANCELLED"):
            return False
        robot_x = r1 if sent["robot"] == r1.robot_id else r2  # type: ignore[attr-defined]
        place = w._place(robot_x.x, robot_x.y)
        return "wait point" in place or "PARKING" in place

    states = w.run_until(at_recovery, 600, jobs, on_tick=cancel_once_onboard)
    w.stop()
    print(chr(10) + w.report())
    w.save()
    w.assert_safe()
    assert states[jobs_y[0]]["status"] == "COMPLETED", states[jobs_y[0]]["status"]
    robot_x = r1 if sent["robot"] == r1.robot_id else r2  # type: ignore[attr-defined]
    print(f"recovery points offered: {[(p.get('code'), p.get('free')) for p in sent['points']] if isinstance(sent['points'], list) else sent['points']}")
    print(f"robot {robot_x.code[-2:]} holds the cancelled cargo at {w._place(robot_x.x, robot_x.y)}")
    x = api.call("GET", f"/api/workflow-execution/jobs/{jobs_x[0]}")
    assert any(c["status"] == "ONBOARD" for c in x["containers"]), "the cargo is still on the robot, waiting for the unload by scan"

    # Staff decide to deliver it after all: the robot leaves the recovery point and drops it at SHELF-C-1.
    api.call("POST", f"/api/transportation/transport-requests/{tr_x}/cancellation/jobs/{jobs_x[0]}/deliver", {})
    w2 = Watch(api, [r1, r2], "5b_deliver_from_recovery").start()
    done = w2.run_until(terminal, 300, jobs)
    w2.stop()
    print(chr(10) + w2.report())
    w2.assert_safe()
    # A cancelled request's job ends COMPLETED (cargo delivered) or CANCELLED, never FAILED.
    assert all(v["status"] in ("COMPLETED", "CANCELLED") for v in done.values()), {k[:8]: v["status"] for k, v in done.items()}
    assert robot_free(robot_x.robot_id), "the robot is free again"  # type: ignore[attr-defined]
    assert math.hypot(robot_x.x - eps["EP-SHELF-C-1"][1], robot_x.y - eps["EP-SHELF-C-1"][2]) < 0.25, "delivered at C-1"


def _try_recovery(api, tr: str, job: str):
    try:
        return api.call("POST", f"/api/transportation/transport-requests/{tr}/cancellation/jobs/{job}/recovery", {})
    except AssertionError as refused:
        print(f"recovery not yet: {str(refused)[:160]}")
        return None


# ---------------------------------------------------------------------- 6-9 more ways for two robots to meet


def run_requests(api, pair, name: str, requests: list[list[tuple[str, str]]], timeout: float = 700) -> Watch:
    """Creates the requests at once, lets both robots run them, checks the safety rules, returns the watch."""
    wh, r1, r2 = pair
    free_both(api, r1, r2)
    jobs: list[str] = []
    trs: list[str] = []
    for moves in requests:
        tr, js = request(api, wh, moves)
        trs.append(tr)
        jobs += js
    w = Watch(api, [r1, r2], name).start()
    states = w.run_until(terminal, timeout, jobs)
    finish(w, states)
    for tr in trs:
        assert api.call("GET", f"/api/transportation/transport-requests/{tr}")["status"] == "COMPLETED", tr
    assert not bad_stops(w), bad_stops(w)
    w.jobs = jobs  # type: ignore[attr-defined]
    return w


def test_6_paths_cross_in_the_centre(api, pair):
    """Shelf A-1 -> C-1 (north-west to south) and B-2 -> INSPECTION (north-east to south-west) at the same time."""
    w = run_requests(api, pair, "6_cross_centre", [[("EP-SHELF-A-1", "EP-SHELF-C-1")], [("EP-SHELF-B-2", "EP-INSPECTION")]])
    assert {assignments(j)[-1][0] for j in w.jobs} == {pair[1].robot_id, pair[2].robot_id}  # type: ignore[attr-defined]
    assert w.both_busy_s >= 10.0, w.both_busy_s


def test_7_two_robots_deliver_to_the_same_place(api, pair):
    """Different pickups, one drop-off (OUTBOUND): the later robot must not drive up behind the first one."""
    w = run_requests(api, pair, "7_same_dropoff", [[("EP-SHELF-A-2", "EP-OUTBOUND")], [("EP-SHELF-B-2", "EP-OUTBOUND")]])
    assert {assignments(j)[-1][0] for j in w.jobs} == {pair[1].robot_id, pair[2].robot_id}  # type: ignore[attr-defined]


def test_8_three_requests_at_once_in_opposite_directions(api, pair):
    """More work than robots, opposite directions on the same corridor: INBOUND -> A-1, A-1 -> INBOUND, OUTBOUND -> B-2."""
    run_requests(api, pair, "8_three_requests", [
        [("EP-INBOUND", "EP-SHELF-A-1")], [("EP-SHELF-A-1", "EP-INBOUND")], [("EP-OUTBOUND", "EP-SHELF-B-2")]], timeout=900)


def test_9_each_robot_goes_where_the_other_stands(api, pair):
    """The robots swap places: each request starts at the other robot's spot."""
    wh, r1, r2 = pair
    free_both(api, r1, r2)
    eps = endpoints()

    def at(sim: SimRobot) -> str:
        return min(eps, key=lambda c: math.hypot(sim.x - eps[c][1], sim.y - eps[c][2]))

    a, b = at(r1), at(r2)
    assert a != b, (a, b)
    # Pickup where the other robot stands, drop-off where this one stands (so neither can simply stay).
    w = run_requests(api, pair, "9_swap", [[(b, a)], [(a, b)]], timeout=900)
    print(f"swap between {a} and {b}")
