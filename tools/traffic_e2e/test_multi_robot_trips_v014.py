"""Two simulated robots run whole transport requests on MAP-ONE-V014 (one robot per area, queue point), motion-free.

Real Host (Semantic traffic, prefix grants, current build) + real Mosquitto + the robot gateway's own logic (SimRobot),
isolated: database warotrans_test, topic tree warotrans-e2e/v1, robot codes E2E-*. The robots follow paths planned on
the V013 grid (map_paths.py: walls, inflated keepout, one-way lanes), at RBT-001's and RBT-002's cruise speeds.

What is shown:
  * a request of 5 containers becomes 2 trips (3 + 2); each robot takes one, both drive at the same time, traffic
    never gives one cell to two robots, nobody moves without authority, and the request completes;
  * a robot that drops off the network on its way to the pickup (nothing on board) gives its trip back to the queue;
    once it is back the trip is taken again and the request still completes.

Run on its own (it owns port 5199 and the coordinator lease of warotrans_test):
    python -m pytest tools/traffic_e2e/test_multi_robot_trips_v013.py -q
"""

from __future__ import annotations

import json
import os
import math
import subprocess
import sys
import time
import uuid
from pathlib import Path

import pytest

from harness import PREFIX, ROBOT_REPO, Api, EvidenceRecorder, Host, wait_for
from map_paths import MapPaths
from sim_robot import SimRobot

HERE = Path(__file__).resolve().parent
ARTIFACTS = HERE / "artifacts"
BUNDLE = ROBOT_REPO / "ros2_ws" / "src" / "warotrans_navigation" / "maps" / "map_one_v014"
MANIFEST = json.loads((BUNDLE / "semantic_manifest.json").read_text(encoding="utf-8"))
ZONES = json.loads((BUNDLE / "single_occupancy.json").read_text(encoding="utf-8"))["zones"]
MAP_ID, MAP_CODE, FINGERPRINT = MANIFEST["mapVersionId"], MANIFEST["mapCode"], MANIFEST["semanticMapFingerprint"]
WORKFLOW = "E2E_INBOUND_TRIPS"  # the real INBOUND steps, without the voucher rule tied to the INBOUND code
RUN = uuid.uuid4().hex[:4].upper()
SPEED = {"R1": 0.35, "R2": 0.30}  # RBT-001 / RBT-002 cruise
PATHS = MapPaths(BUNDLE)
ROBOTS: list[SimRobot] = []
# The Evidence Recorder runs beside the Host when it is built (WARO_E2E_EVIDENCE=0 turns it off).
EVIDENCE = os.environ.get("WARO_E2E_EVIDENCE", "1") == "1" and EvidenceRecorder.available()
RECORDERS: list[EvidenceRecorder] = []


@pytest.fixture(scope="module")
def host():
    ARTIFACTS.mkdir(exist_ok=True)
    seed = HERE / "seed_test_db.py"
    for args in (["--cleanup-e2e-robots"], ["--map-code", MAP_CODE, "--workflow-code", "INBOUND", "--workflow-alias", WORKFLOW]):
        done = subprocess.run([sys.executable, str(seed), *args], capture_output=True, text=True)
        assert done.returncode == 0, done.stderr
    evidence_env: dict[str, str] = {}
    if EVIDENCE:
        recorder = EvidenceRecorder(ARTIFACTS)
        recorder.start()
        RECORDERS.append(recorder)
        evidence_env = recorder.host_env()
    h = Host(ARTIFACTS, name="host-v013", extra_env={
        **evidence_env,
        "Traffic__SemanticMapDirectory": str(BUNDLE),
        "Traffic__SemanticPrefixGrant": os.environ.get("WARO_E2E_PREFIX_GRANT", "true"),  # RealRobot: true
        # RealRobot uses 0.25 m cells. WARO_E2E_CELL lets the same run be tried with another size.
        "Traffic__SemanticCellSizeMeters": os.environ.get("WARO_E2E_CELL", "0.10"),
        "Traffic__SemanticLaneHalfWidthMeters": os.environ.get("WARO_E2E_LANE_HALF", "0"),
        # As the RealRobot profile: idle robots go home (or to the queue point), and make way when in the way.
        "FleetIdle__AutoReturnToPark": "true",
        "FleetIdle__IdleSeconds": "3",
        "FleetIdle__FallbackEndpointCodes": "EP-QUEUE-POINT",
    })
    try:
        h.start()
    except BaseException:
        for recorder in RECORDERS:
            recorder.stop()
        raise
    yield h
    for r in ROBOTS:
        r.stop()
    h.stop()
    for recorder in RECORDERS:
        recorder.stop()


@pytest.fixture(scope="module")
def api(host):
    a = Api()
    a.login()
    return a


def endpoints() -> dict[str, tuple[str, float, float, float]]:
    out = subprocess.run(
        ["docker", "exec", "warotrans-postgres", "psql", "-U", "warotrans", "-d", "warotrans_test", "-t", "-A", "-F", ",",
         "-c", f"select code, id, x, y, yaw from navigation.endpoints where map_version_id = '{MAP_ID}'"],
        capture_output=True, text=True, check=True)
    return {c: (i, float(x), float(y), float(w)) for c, i, x, y, w in (l.split(",") for l in out.stdout.strip().splitlines())}


def robot(api, warehouse_id: str, name: str, at: tuple[str, float, float, float]) -> tuple[SimRobot, str]:
    code = f"E2E-{RUN}-{name}"
    rid = api.call("POST", "/api/fleet/robots", {
        "warehouseId": warehouse_id, "currentMapVersionId": MAP_ID, "code": code, "name": code})["id"]
    api.call("PUT", f"/api/fleet/robots/{rid}/capacity", {"maxContainers": 3, "lengthM": 0.30, "widthM": 0.26})
    sim = SimRobot(code=code, x=at[1], y=at[2], fingerprint=FINGERPRINT, zones=ZONES, prefix=PREFIX, map_code=MAP_CODE,
                   speed=SPEED[name], progress_timeout_sec=60.0, path_provider=PATHS.provider())
    sim.start()
    ROBOTS.append(sim)
    wait_for(lambda: api.call("GET", f"/api/fleet/robots/{rid}").get("isOnline"), 20, what=f"{code} online")
    wait_for(lambda: api.call("GET", f"/api/fleet/robots/{rid}").get("localizationStatus") == "LOCALIZED", 20,
             what=f"{code} localized")
    api.call("POST", f"/api/fleet/robots/{rid}/put-in-service", {})
    sim.robot_id = rid  # type: ignore[attr-defined]
    return sim, rid


@pytest.fixture(scope="module")
def pair(api):
    """R1 at RBT-001's home (EP-PARKING-A), R2 at RBT-002's (EP-PARKING-B), once per module."""
    wh = warehouse(api)
    eps = endpoints()
    r1, id1 = robot(api, wh, "R1", eps["EP-PARKING-A"])
    r2, id2 = robot(api, wh, "R2", eps["EP-PARKING-B"])
    return wh, (r1, id1), (r2, id2)


def request(api, warehouse_id: str, source: str, destinations: list[str]) -> tuple[str, list[str], list[str]]:
    return request_pairs(api, warehouse_id, [(source, d) for d in destinations])


def request_pairs(api, warehouse_id: str, moves: list[tuple[str, str]]) -> tuple[str, list[str], list[str]]:
    eps = endpoints()
    destinations = [d for _, d in moves]
    cat = api.call("POST", "/api/warehouse/product-categories", {"code": f"E2E-{RUN}-{uuid.uuid4().hex[:4]}", "name": "E2E"})["id"]
    prod = api.call("POST", "/api/warehouse/products", {"categoryId": cat, "sku": f"E2E-{uuid.uuid4().hex[:8]}", "name": "E2E box"})["id"]
    containers = [api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"] for _ in destinations]
    wf = next(w for w in api.call("GET", "/api/workflow-execution/workflows?pageSize=100")["items"] if w["code"] == WORKFLOW)["id"]
    tr = api.call("POST", "/api/transportation/transport-requests", {
        "warehouseId": warehouse_id, "mapVersionId": MAP_ID, "workflowId": wf, "workflowInputs": {}, "note": "E2E V014 trips",
        "details": [{"sequenceNo": i + 1, "containerId": c, "sourceEndpointId": eps[src][0], "destinationEndpointId": eps[d][0]}
                    for i, (c, (src, d)) in enumerate(zip(containers, moves))]})["id"]
    jobs = [j["jobId"] for j in api.call("POST", f"/api/transportation/transport-requests/{tr}/queue", {})["jobs"]]
    # "Run on robot" in the web: the first trip joins the queue, the queue brings in the others.
    api.call("POST", f"/api/workflow-execution/jobs/{jobs[0]}/enqueue", {})
    return tr, jobs, containers


class Recorder:
    """Samples the two robots and the traffic snapshot; confirms every waiting person step (the floor staff)."""

    def __init__(self, api, sims: list[SimRobot]) -> None:
        self.api, self.sims = api, sims
        self.last = {s.code: (s.x, s.y) for s in sims}
        self.both_moving_s = 0.0
        self.moved_m = {s.code: 0.0 for s in sims}
        self.confirmations = 0
        self.max_holders_seen = 0
        self.queue_waits = set()  # robots seen standing at EP-QUEUE-POINT while their job waited for an area
        self.timeline: list[dict] = []

    def sample(self, dt: float) -> None:
        moving = []
        for s in self.sims:
            lx, ly = self.last[s.code]
            d = math.hypot(s.x - lx, s.y - ly)
            self.moved_m[s.code] += d
            moving.append(d > 0.005)
            self.last[s.code] = (s.x, s.y)
        if all(moving):
            self.both_moving_s += dt
        snap = self.api.snapshot()
        for r in snap["resources"]:
            assert len(r["holders"]) <= r["capacity"], f"{r['resourceId']} over capacity: {r['holders']}"
            self.max_holders_seen = max(self.max_holders_seen, len(r["holders"]))

    def confirm_waiting(self, jobs: list[str]) -> dict[str, dict]:
        states = {}
        for job in jobs:
            j = self.api.call("GET", f"/api/workflow-execution/jobs/{job}")
            states[job] = j
            if any(st.get("outputValues", {}).get("areaQueueZone") for t in j["tasks"] for st in t["steps"]):
                self.queue_waits.add(job)
            for t in sorted(j["tasks"], key=lambda t: t["sequenceNo"]):
                for s in t["steps"]:
                    if s["status"] != "WAITING" or s.get("stepType") != "HUMAN_INTERACTION":
                        continue
                    # The hand-over steps carry their container in resolvedInputs (the task may not).
                    container = (s.get("resolvedInputs") or {}).get("containerId") or t.get("containerId")
                    if container:
                        self.api.call("POST", f"/api/workflow-execution/job-steps/{s['id']}/confirm",
                                      {"containerId": container, "note": "e2e staff"})
                        self.confirmations += 1
        return states

    def save(self, name: str = "v014_timeline.json") -> None:
        (ARTIFACTS / name).write_text(json.dumps({"bothMovingSeconds": round(self.both_moving_s, 1), "movedMeters": self.moved_m,
                                                  "timeline": self.timeline}), encoding="utf-8")

    def run_until(self, done, timeout: float, jobs: list[str], on_tick=None) -> dict[str, dict]:
        end = time.time() + timeout
        states: dict[str, dict] = {}
        while time.time() < end:
            t0 = time.time()
            states = self.confirm_waiting(jobs)
            self.timeline.append({"t": round(t0, 2), **{s.code: [round(s.x, 2), round(s.y, 2)] for s in self.sims},
                                  "jobs": {k[:8]: v["status"] for k, v in states.items()}})
            if on_tick is not None:
                on_tick(states)
            if done(states):
                self.save()
                return states
            time.sleep(0.25)
            self.sample(time.time() - t0)
        self.save()
        raise AssertionError(f"timed out; jobs: { {k[:8]: (v['status'], v.get('failureCode'), v.get('blockingReason')) for k, v in states.items()} }")


def warehouse(api) -> str:
    out = subprocess.run(
        ["docker", "exec", "warotrans-postgres", "psql", "-U", "warotrans", "-d", "warotrans_test", "-t", "-A", "-c",
         f"select warehouse_id from navigation.map_versions where id = '{MAP_ID}'"], capture_output=True, text=True, check=True)
    return out.stdout.strip()


def assignments(job_id: str) -> list[tuple[str, bool]]:
    out = subprocess.run(
        ["docker", "exec", "warotrans-postgres", "psql", "-U", "warotrans", "-d", "warotrans_test", "-t", "-A", "-F", ",",
         "-c", f"select robot_id, ended_at is null from execution.job_robot_assignments where job_id = '{job_id}' order by started_at"],
        capture_output=True, text=True, check=True)
    return [(r, ok == "t") for r, ok in (l.split(",") for l in out.stdout.strip().splitlines() if l)]


def assignments_open(robot_id: str) -> bool:
    out = subprocess.run(
        ["docker", "exec", "warotrans-postgres", "psql", "-U", "warotrans", "-d", "warotrans_test", "-t", "-A",
         "-c", f"select count(*) from execution.job_robot_assignments where robot_id = '{robot_id}' and ended_at is null"],
        capture_output=True, text=True, check=True)
    return out.stdout.strip() not in ("", "0")


def terminal(states: dict[str, dict]) -> bool:
    return bool(states) and all(s["status"] in ("COMPLETED", "FAILED", "CANCELLED") for s in states.values())


# ---------------------------------------------------------------------- 1


def test_two_robots_split_five_containers_and_drive_at_the_same_time(api, pair):
    wh, (r1, id1), (r2, id2) = pair

    shelves = os.environ.get("WARO_E2E_SHELVES", "EP-SHELF-A-1,EP-SHELF-A-2,EP-SHELF-B-01-02,EP-SHELF-C-1,EP-SHELF-A-1").split(",")
    tr, jobs, containers = request(api, wh, "EP-INBOUND", shelves)
    assert len(jobs) == 2, "5 containers, at most 3 per robot: 2 trips"

    rec = Recorder(api, [r1, r2])
    states = rec.run_until(terminal, 420, jobs)

    assert all(s["status"] == "COMPLETED" for s in states.values()), {k[:8]: s["status"] for k, s in states.items()}
    taken_by = {j: assignments(j)[-1][0] for j in jobs}
    assert set(taken_by.values()) == {id1, id2}, "each robot carried one trip"
    sizes = sorted(len(s["containers"]) for s in states.values())
    assert sizes == [2, 3]
    assert all(c["status"] == "DELIVERED" for s in states.values() for c in s["containers"])
    assert api.call("GET", f"/api/transportation/transport-requests/{tr}")["status"] == "COMPLETED"
    assert rec.confirmations == 10, "5 pickups + 5 drops"
    # Both robots really drove, and at the same time (not one after the other).
    assert min(rec.moved_m.values()) > 3.0, rec.moved_m
    assert rec.both_moving_s >= 5.0, f"robots moved together only {rec.both_moving_s:.1f} s"
    assert not r1.moved_while_gate_closed() and not r2.moved_while_gate_closed()
    print(f"\nmoved {rec.moved_m}, both moving {rec.both_moving_s:.1f} s, confirmations {rec.confirmations}")

    if EVIDENCE:
        # Evidence: one journal for the request, one run per robot assignment (3 + 2 containers), filed from the
        # Host's outbox; the request's status is copied as Transportation states it.
        recorder = RECORDERS[0]

        def complete_journal():
            j = recorder.journal(api, tr)
            return j if j and j["sourceTransportStatus"] == "COMPLETED" and len(j["runs"]) == 2 else None

        journal = wait_for(complete_journal, 60, interval=1.0, what="evidence journal with 2 runs")
        assert journal["requestedContainerCount"] == 5
        assert sorted(r["assignedContainerCount"] for r in journal["runs"]) == [2, 3]
        assert {r["robotCode"] for r in journal["runs"]} == {r1.code, r2.code}
        assert {r["sourceJobId"] for r in journal["runs"]} == set(jobs)
        assert all(r["runCorrelationKey"].startswith("jra:") and r["endedAtUtc"] for r in journal["runs"])
        assert all(r["mapVersion"] == MAP_CODE for r in journal["runs"])
        assert journal["eventsBySeverity"].get("INFO", 0) > 10, journal["eventsBySeverity"]
        print(f"evidence journal {journal['requestCode']}: {journal['evidenceStatus']}, events {journal['eventsBySeverity']}")


# ---------------------------------------------------------------------- 2


def test_a_robot_lost_on_the_way_gives_its_trip_back_and_the_request_still_completes(api, pair):
    wh, (r1, _), (r2, _) = pair
    tr, jobs, _ = request(api, wh, "EP-INBOUND", ["EP-SHELF-B-01-02", "EP-SHELF-C-1", "EP-SHELF-A-1", "EP-SHELF-A-2"])
    assert len(jobs) == 2

    lost: dict[str, object] = {}

    def drop_r2_on_its_way(states: dict[str, dict]) -> None:
        if lost:
            return
        for job, s in states.items():
            mine = assignments(job)
            moving = any(st["status"] == "EXECUTING" and st.get("stepType") == "MOVE" for t in s["tasks"] for st in t["steps"])
            if s["status"] == "RUNNING" and mine and mine[-1] == (r2.robot_id, True) and moving \
                    and not any(c["status"] == "ONBOARD" for c in s["containers"]):
                start = (r2.x, r2.y)
                wait_for(lambda: math.hypot(r2.x - start[0], r2.y - start[1]) > 0.3, 30, what="R2 on its way")
                r2.disconnect()
                lost.update(job=job, at=time.time())
                return

    rec = Recorder(api, [r1, r2])
    requeued: dict[str, str] = {}

    def watch(states: dict[str, dict]) -> None:
        drop_r2_on_its_way(states)
        if lost and not requeued:
            s = states[lost["job"]]
            if s["status"] == "QUEUED":
                requeued["reason"] = s.get("blockingReason") or ""
                r2.reconnect()  # it comes back; the backend reconciles it from its reported pose

    # Waiting behind the other robot is normal here (one robot per area, a lost robot keeps its space): allow 15 min.
    states = rec.run_until(lambda st: terminal(st) and bool(requeued), 900, jobs, on_tick=watch)

    assert lost, "R2 never took a trip"
    assert requeued, "the lost robot's trip did not go back to the queue"
    assert all(s["status"] == "COMPLETED" for s in states.values()), {k[:8]: s["status"] for k, s in states.items()}
    history = assignments(lost["job"])
    assert history[0] == (r2.robot_id, False), "R2's assignment of the lost trip was closed"
    # Taken again after it went back to the queue; every assignment is closed now that the trip is done.
    assert len(history) >= 2 and not any(open_ for _, open_ in history), history
    assert all(c["status"] == "DELIVERED" for s in states.values() for c in s["containers"])
    assert api.call("GET", f"/api/transportation/transport-requests/{tr}")["status"] == "COMPLETED"
    assert not r1.moved_while_gate_closed() and not r2.moved_while_gate_closed()
    print(f"\nrequeued: {requeued['reason'][:120]}; lost trip taken again by "
          f"{'R1' if history[-1][0] == r1.robot_id else 'R2'}; both moving {rec.both_moving_s:.1f} s")


# ---------------------------------------------------------------------- 0


def test_two_requests_in_separate_areas_run_on_both_robots_at_the_same_time(api, pair):
    """Shelf row A for one robot, row C to OUTBOUND for the other: the queue gives one to each, both drive at once."""
    wh, (r1, id1), (r2, id2) = pair
    # Both robots free first (an earlier scenario may still be driving one home), else one robot rightly takes both.
    wait_for(lambda: all(api.call("GET", f"/api/fleet/robots/{rid}")["status"] == "AVAILABLE" for rid in (id1, id2))
             and not assignments_open(id1) and not assignments_open(id2), 300, what="both robots free")
    tr_a, jobs_a, _ = request_pairs(api, wh, [("EP-SHELF-A-1", "EP-SHELF-A-2")])
    tr_c, jobs_c, _ = request_pairs(api, wh, [("EP-SHELF-C-1", "EP-OUTBOUND")])
    api.call("POST", f"/api/workflow-execution/jobs/{jobs_c[0]}/enqueue", {})  # request() queued only its own first trip
    jobs = jobs_a + jobs_c

    rec = Recorder(api, [r1, r2])
    states = rec.run_until(terminal, 300, jobs)
    rec.save("v014_separate_areas_timeline.json")

    assert all(s["status"] == "COMPLETED" for s in states.values()),         {k[:8]: (s["status"], s.get("failureCode")) for k, s in states.items()}
    assert {assignments(j)[-1][0] for j in jobs} == {id1, id2}, "each robot took one request"
    for tr in (tr_a, tr_c):
        assert api.call("GET", f"/api/transportation/transport-requests/{tr}")["status"] == "COMPLETED"
    assert min(rec.moved_m.values()) > 1.0, rec.moved_m  # C-1 -> OUTBOUND alone is ~1.1 m
    assert rec.both_moving_s >= 5.0, f"robots moved together only {rec.both_moving_s:.1f} s"
    assert not r1.moved_while_gate_closed() and not r2.moved_while_gate_closed()
    print(f"separate areas: moved {rec.moved_m}, both moving {rec.both_moving_s:.1f} s")
