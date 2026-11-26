"""Drive the two REAL robots (RBT-001, RBT-002) through whole transport requests on the dev Host, with a safety watch.

Not a pytest: it moves real robots. The dev Host (:5090, profile RealRobot) and both robot stacks must be running and
localized. Every person step (pickup / storage confirmations) is confirmed by this script as the floor staff would.

Safety watch (5 Hz): the body gap of the two robots (footprint rectangles at their reported poses). Below
STOP_GAP_M both robots get STOP and the scenario fails. Nothing here bypasses traffic: every motion is a Job MOVE
that the Host grants.

    python tools/traffic_e2e/real_two_robot_run.py A1 A2 A3 B C6 C5
"""

from __future__ import annotations

import json
import os
import math
import subprocess
import sys
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from harness import Api  # noqa: E402

BASE = "http://127.0.0.1:5090"
HERE = Path(__file__).resolve().parent
OUT = HERE / "artifacts" / f"real_{datetime.now():%Y%m%d_%H%M%S}"
DB = "warotrans"
STOP_GAP_M = 0.06   # body gap that stops both robots (real touch = 0; poses are ~5 cm uncertain)
WARN_GAP_M = 0.15
TERMINAL = {"COMPLETED", "FAILED", "CANCELLED"}
# Footprints in base_footprint (x forward): RBT-001 nav2_params, RBT-002 fleet.robots 0.30 x 0.263.
FOOTPRINT = {"RBT-001": (-0.11, 0.21, 0.13), "RBT-002": (-0.15, 0.15, 0.132)}


def sql(q: str) -> list[list[str]]:
    out = subprocess.run(["docker", "exec", "warotrans-postgres", "psql", "-U", "warotrans", "-d", DB, "-At", "-F", "|", "-c", q],
                         capture_output=True, text=True, check=True).stdout
    return [l.split("|") for l in out.strip().splitlines() if l]


# ---------------------------------------------------------------- geometry
def rect(code: str, x: float, y: float, yaw: float) -> list[tuple[float, float]]:
    x0, x1, h = FOOTPRINT[code]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + px * c - py * s, y + px * s + py * c) for px, py in ((x0, -h), (x1, -h), (x1, h), (x0, h))]


def _seg_dist(p, q, a, b) -> float:
    def pt_seg(pt, s0, s1):
        dx, dy = s1[0] - s0[0], s1[1] - s0[1]
        t = max(0.0, min(1.0, ((pt[0] - s0[0]) * dx + (pt[1] - s0[1]) * dy) / (dx * dx + dy * dy or 1e-12)))
        return math.hypot(pt[0] - s0[0] - t * dx, pt[1] - s0[1] - t * dy)
    return min(pt_seg(p, a, b), pt_seg(q, a, b), pt_seg(a, p, q), pt_seg(b, p, q))


def _inside(pt, poly) -> bool:
    sign = None
    for i in range(4):
        a, b = poly[i], poly[(i + 1) % 4]
        cr = (b[0] - a[0]) * (pt[1] - a[1]) - (b[1] - a[1]) * (pt[0] - a[0])
        if sign is None:
            sign = cr > 0
        elif (cr > 0) != sign:
            return False
    return True


def body_gap(r1, r2) -> float:
    if any(_inside(p, r2) for p in r1) or any(_inside(p, r1) for p in r2):
        return 0.0
    return min(_seg_dist(r1[i], r1[(i + 1) % 4], r2[j], r2[(j + 1) % 4]) for i in range(4) for j in range(4))


# ---------------------------------------------------------------- world
class World:
    def __init__(self) -> None:
        self.api = Api(BASE)
        self.api.login()
        self.robots = {c: i for i, c in sql("select id, code from fleet.robots where code in ('RBT-001','RBT-002')")}
        # The published map (V017 since 2026-11-23); WARO_REAL_MAP pins another code.
        code = os.environ.get("WARO_REAL_MAP", "")
        self.map_id = sql("select id from navigation.map_versions where status='PUBLISHED'"
                          + (f" and code='{code}'" if code else "") + " order by published_at desc nulls last limit 1")[0][0]
        self.warehouse = sql(f"select warehouse_id from navigation.map_versions where id='{self.map_id}'")[0][0]
        self.eps = {c: (i, float(x), float(y)) for c, i, x, y in
                    sql(f"select code, id, x, y from navigation.endpoints where map_version_id='{self.map_id}'")}
        self.locs = {c: i for c, i in sql("select code, id from warehouse.storage_locations where is_active")}
        # Newest PUBLISHED version of each workflow code (older versions still point at old maps' endpoints).
        self.wf = {c: i for c, i in sql("select distinct on (code) code, id from execution.workflows "
                                        "where status='PUBLISHED' order by code, version_no desc")}
        # Storage location -> its endpoint code on this map (SHELF-B-2 sits at EP-SHELF-B-01-02 on V017).
        self.loc_ep = {c: e for c, e in sql("select sl.code, e.code from warehouse.storage_locations sl join "
                                            f"navigation.endpoints e on e.id=sl.endpoint_id where e.map_version_id='{self.map_id}'")}
        self.product = sql("select id from warehouse.products order by created_at limit 1")[0][0]

    def robot(self, code: str) -> dict:
        r = self.api.call("GET", f"/api/fleet/robots/{self.robots[code]}")
        return r.get("data", r)


class Watch(threading.Thread):
    """Poses at 5 Hz, body gap, STOP both robots below STOP_GAP_M; traffic snapshot every second."""

    def __init__(self, w: World, name: str) -> None:
        super().__init__(daemon=True)
        self.w, self.name = w, name
        self.api = Api(BASE)
        self.api.login()
        self.running = True
        self.min_gap = math.inf
        self.min_gap_at: dict = {}
        self.warns = 0
        self.tripped = False
        self.samples: list[dict] = []
        self.snapshots: list[dict] = []
        self.both_moving_s = 0.0
        self.any_moving_s = 0.0
        self.moving_s = {c: 0.0 for c in w.robots}
        self._last: dict[str, tuple[float, float]] = {}

    def run(self) -> None:
        last_snap = 0.0
        while self.running:
            t0 = time.time()
            try:
                poses = {}
                for code, rid in self.w.robots.items():
                    r = self.api.call("GET", f"/api/fleet/robots/{rid}")
                    r = r.get("data", r)
                    poses[code] = (r["poseX"], r["poseY"], r["poseYaw"], r.get("status"), r.get("navigationStatus"))
                gap = body_gap(rect("RBT-001", *poses["RBT-001"][:3]), rect("RBT-002", *poses["RBT-002"][:3]))
                moving = [math.hypot(p[0] - self._last.get(c, p[:2])[0], p[1] - self._last.get(c, p[:2])[1]) > 0.004
                          for c, p in poses.items()]
                if all(moving):
                    self.both_moving_s += 0.2
                if any(moving):
                    self.any_moving_s += 0.2
                for c, m in zip(poses, moving):
                    if m:
                        self.moving_s[c] += 0.2
                self._last = {c: p[:2] for c, p in poses.items()}
                self.samples.append({"t": round(t0, 2), "gap": round(gap, 3),
                                     **{c: [round(v, 3) if isinstance(v, float) else v for v in p] for c, p in poses.items()}})
                if gap < self.min_gap:
                    self.min_gap, self.min_gap_at = gap, self.samples[-1]
                if gap < WARN_GAP_M:
                    self.warns += 1
                if gap < STOP_GAP_M and not self.tripped:
                    self.tripped = True
                    for rid in self.w.robots.values():
                        try:
                            self.api.call("POST", f"/api/fleet/robots/{rid}/commands/stop",
                                          {"reason": f"real_two_robot_run safety gap {gap:.3f} m"})
                        except Exception as exc:  # keep stopping the other one
                            print("STOP failed:", exc)
                    print(f"!!! SAFETY STOP: body gap {gap:.3f} m at {poses}")
                if t0 - last_snap >= 1.0:
                    last_snap = t0
                    snap = self.api.call("GET", "/api/fleet/traffic/snapshot")
                    self.snapshots.append({"t": round(t0, 2), "snap": snap})
            except Exception as exc:
                print("watch error:", exc)
            time.sleep(max(0.0, 0.2 - (time.time() - t0)))

    def stop(self) -> None:
        self.running = False
        self.join(timeout=5)
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / f"{self.name}_poses.json").write_text(json.dumps(self.samples), encoding="utf-8")
        (OUT / f"{self.name}_traffic.json").write_text(json.dumps(self.snapshots), encoding="utf-8")


# ---------------------------------------------------------------- flow
def new_containers(w: World, n: int, tag: str) -> list[str]:
    body = {"warehouseId": w.warehouse, "requestId": str(uuid.uuid4()), "containerCount": n,
            "groups": [{"groupKey": tag, "productId": w.product, "containerCount": n, "quantityPerContainer": 1}]}
    r = w.api.call("POST", "/api/warehouse/containers", body)
    ids = [c["id"] for c in (r.get("containers") or r.get("items") or [])]
    if not ids:
        raise AssertionError(f"no container ids in {r}")
    return ids


def voucher(w: World, kind: str, containers: list[str], note: str) -> str:
    r = w.api.call("POST", "/api/warehouse/stock-vouchers", {
        "warehouseId": w.warehouse, "type": kind, "note": note, "containerIds": containers, "confirm": True})
    return r.get("id") or r.get("data", {}).get("id")


def create_tr(w: World, workflow: str, moves: list[dict], note: str, check_quality: bool = False) -> tuple[str, list[str]]:
    """moves: {container, src, dst, srcLoc?, dstLoc?}; returns (tr id, job ids)."""
    details = []
    for i, m in enumerate(moves):
        d = {"sequenceNo": i + 1, "containerId": m["container"], "sourceEndpointId": w.eps[m["src"]][0],
             "destinationEndpointId": w.eps[m["dst"]][0]}
        if m.get("srcLoc"):
            d["sourceStorageLocationId"] = w.locs[m["srcLoc"]]
        if m.get("dstLoc"):
            d["destinationStorageLocationId"] = w.locs[m["dstLoc"]]
        details.append(d)
    tr = w.api.call("POST", "/api/transportation/transport-requests", {
        "warehouseId": w.warehouse, "mapVersionId": w.map_id, "workflowId": w.wf[workflow],
        # INBOUND asks "Check Quality?" (STAFF_INPUT checkQuality): straight to the shelf, no EP-INSPECTION stop.
        # Every transport workflow asks "Check Quality?" (STAFF_INPUT checkQuality); True adds the EP-INSPECTION stop.
        "workflowInputs": {"checkQuality": check_quality},
        "note": note, "details": details})["id"]
    jobs = [j["jobId"] for j in w.api.call("POST", f"/api/transportation/transport-requests/{tr}/queue", {})["jobs"]]
    info = w.api.call("GET", f"/api/transportation/transport-requests/{tr}")
    route = "; ".join(f"{m.get('srcLoc') or m['src']} -> {m.get('dstLoc') or m['dst']}" for m in moves)
    print(f"  >> TR {info.get('requestCode') or tr[:8]} {workflow}{' + quality inspection' if check_quality else ''}: "
          f"{len(moves)} box(es) [{route}], {len(jobs)} trip(s)", flush=True)
    return tr, jobs


def confirm_waiting(w: World, jobs: list[str], log: list) -> dict[str, dict]:
    states = {}
    for job in jobs:
        j = w.api.call("GET", f"/api/workflow-execution/jobs/{job}")
        states[job] = j
        for t in sorted(j["tasks"], key=lambda t: t["sequenceNo"]):
            for s in t["steps"]:
                if s["status"] != "WAITING" or s.get("stepType") != "HUMAN_INTERACTION":
                    continue
                container = (s.get("resolvedInputs") or {}).get("containerId") or t.get("containerId")
                if not container:
                    continue
                body = {"containerId": container, "note": "real 2-robot test staff"}
                if "INSPECTION" in str(s.get("stepKey")):
                    body["inspectionResult"] = "PASSED"
                w.api.call("POST", f"/api/workflow-execution/job-steps/{s['id']}/confirm", body)
                log.append({"t": round(time.time(), 1), "job": j["jobNo"], "step": s.get("name"), "container": container[:8]})
                print(f"  confirmed {j['jobNo']} {s.get('name')} {container[:8]}")
    return states


def robot_code(w: World, rid: str | None) -> str | None:
    return next((c for c, i in w.robots.items() if i == rid), None)


def run_jobs(w: World, name: str, jobs: list[str], timeout_s: float = 1200) -> dict:
    watch = Watch(w, name)
    watch.start()
    log: list = []
    t_start = time.time()
    states: dict[str, dict] = {}
    seen: dict[str, str] = {}
    blocking: dict[str, set] = {j: set() for j in jobs}
    try:
        while time.time() - t_start < timeout_s and not watch.tripped:
            states = confirm_waiting(w, jobs, log)
            for k, v in states.items():
                line = f"{v['status']} robot={robot_code(w, v.get('assignedRobotId'))} {v['completedSteps']}/{v['totalSteps']}"
                if v.get("blockingReason"):
                    blocking[k].add(v["blockingReason"])
                if seen.get(k) != line:
                    seen[k] = line
                    print(f"  [{time.time() - t_start:6.1f}s] {v['jobNo']} {line} {v.get('blockingReason') or ''}")
            if all(v["status"] in TERMINAL for v in states.values()):
                break
            time.sleep(0.5)
    finally:
        watch.stop()
    result = {
        "scenario": name,
        "seconds": round(time.time() - t_start, 1),
        "minBodyGapM": round(watch.min_gap, 3),
        "minGapSample": watch.min_gap_at,
        "warnSamples(<0.15m)": watch.warns,
        "safetyStop": watch.tripped,
        "bothMovingSeconds": round(watch.both_moving_s, 1),
        "anyMovingSeconds": round(watch.any_moving_s, 1),
        "movingSecondsPerRobot": {c: round(v, 1) for c, v in watch.moving_s.items()},
        # Share of the time anything moved during which BOTH moved: 100 % = fully parallel, 0 % = one after the other.
        "parallelPct": round(100 * watch.both_moving_s / watch.any_moving_s, 1) if watch.any_moving_s else 0.0,
        "jobs": {v["jobNo"]: {"status": v["status"], "robot": robot_code(w, v.get("assignedRobotId")),
                               "trip": f"{v.get('tripNo')}/{v.get('tripCount')}", "containers": len(v.get("containers") or []),
                               "failure": v.get("failureCode"), "failureMessage": v.get("failureMessage"),
                               "blockingReasonsSeen": sorted(blocking[k])}
                 for k, v in states.items()},
        "confirmations": log,
    }
    result["pass"] = (not watch.tripped) and all(v["status"] == "COMPLETED" for v in states.values())
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / f"{name}_result.json").write_text(json.dumps(result, indent=1, default=str), encoding="utf-8")
    print(json.dumps({k: v for k, v in result.items() if k != "confirmations"}, indent=1, default=str))
    return result


def dispatch(w: World, job: str, code: str) -> None:
    j = w.api.call("POST", f"/api/workflow-execution/jobs/{job}/dispatch", {"robotId": w.robots[code]})
    print(f"  >> job {j.get('jobNo', job[:8])} -> {code} (from {nearest(w, code)})", flush=True)


def nearest(w: World, code: str) -> str:
    r = w.robot(code)
    return min(w.eps, key=lambda e: math.hypot(w.eps[e][1] - r["poseX"], w.eps[e][2] - r["poseY"]))


# ---------------------------------------------------------------- scenarios
def stored_at(w: World, loc: str, skip: tuple[str, ...] = ()) -> str:
    rows = sql("select c.id from warehouse.containers c join warehouse.storage_locations sl on sl.id=c.current_storage_location_id "
               f"where c.status='STORED' and sl.code='{loc}' order by c.created_at")
    ids = [r[0] for r in rows if r[0] not in skip]
    if not ids:
        raise AssertionError(f"no STORED container on {loc}")
    return ids[0]


def relocation_pair(w: World, name: str, r1: tuple[str, str], r2: tuple[str, str]) -> dict:
    """Two INTERNAL_RELOCATION requests (a STORED box from shelf to shelf), one forced on each robot, started together."""
    jobs = []
    used: tuple[str, ...] = ()
    for code, (src, dst) in (("RBT-001", r1), ("RBT-002", r2)):
        c = stored_at(w, src, used)
        used += (c,)
        tr, js = create_tr(w, "INTERNAL_RELOCATION", [{"container": c, "src": f"EP-{src}", "srcLoc": src,
                                                        "dst": f"EP-{dst}", "dstLoc": dst}], f"real 2-robot {name} {code}")
        assert len(js) == 1, js
        jobs.append((code, js[0]))
    for code, job in jobs:
        dispatch(w, job, code)
    return run_jobs(w, name, [j for _, j in jobs])


def scenario_A1(w: World) -> dict:
    # Different areas at once: RBT-001 PARKING-A -> SHELF-A-1 -> SHELF-A-2 (north-west row),
    # RBT-002 PARKING-B -> SHELF-C-1 -> SHELF-B-2 (south, then up the east side).
    return relocation_pair(w, "A1", ("SHELF-A-1", "SHELF-A-2"), ("SHELF-C-1", "SHELF-B-2"))


def scenario_A2(w: World) -> dict:
    # Crossing: RBT-001 north-west -> south (A-2 -> C-1), RBT-002 east -> north-west (B-2 -> A-1), through the middle.
    return relocation_pair(w, "A2", ("SHELF-A-2", "SHELF-C-1"), ("SHELF-B-2", "SHELF-A-1"))


def scenario_A3(w: World) -> dict:
    """Same goal: two INBOUND requests (phiếu nhập), one forced on each robot, both pick up at the INBOUND dead-end
    bay; the second must wait clear of it until the first has left."""
    jobs = []
    for code, shelf in (("RBT-001", "SHELF-A-2"), ("RBT-002", "SHELF-B-2")):
        c = new_containers(w, 1, f"A3-{code}")
        voucher(w, "INBOUND", c, f"real 2-robot A3 {code}")
        tr, js = create_tr(w, "INBOUND", [{"container": c[0], "src": "EP-INBOUND", "dst": w.loc_ep[shelf], "dstLoc": shelf}],
                           f"real 2-robot A3 {code}")
        jobs.append((code, js[0]))
    for code, job in jobs:
        dispatch(w, job, code)
    return run_jobs(w, "A3", [j for _, j in jobs])


def scenario_B(w: World) -> dict:
    """Two different requests at once: INBOUND (2 new boxes -> shelf A-1 / A-2, phiếu nhập) and OUTBOUND of the
    two STORED boxes (phiếu xuất). The dispatcher picks the robots."""
    new = new_containers(w, 2, "B-IN")
    voucher(w, "INBOUND", new, "real 2-robot B inbound")
    tr_in, jobs_in = create_tr(w, "INBOUND", [
        {"container": new[0], "src": "EP-INBOUND", "dst": "EP-SHELF-A-1", "dstLoc": "SHELF-A-1"},
        {"container": new[1], "src": "EP-INBOUND", "dst": "EP-SHELF-A-2", "dstLoc": "SHELF-A-2"}], "real 2-robot B inbound")
    stored = sql("select c.id, sl.code from warehouse.containers c join warehouse.storage_locations sl "
                 "on sl.id=c.current_storage_location_id where c.status='STORED' and sl.code in ('SHELF-C-1','SHELF-B-2') "
                 "order by c.created_at limit 2")
    jobs = list(jobs_in)
    if stored:
        voucher(w, "OUTBOUND", [c for c, _ in stored], "real 2-robot B outbound")
        tr_out, jobs_out = create_tr(w, "OUTBOUND_RETRIEVAL", [
            {"container": c, "src": f"EP-{loc}", "srcLoc": loc, "dst": "EP-OUTBOUND"} for c, loc in stored],
            "real 2-robot B outbound")
        jobs += jobs_out
    for j in jobs:
        w.api.call("POST", f"/api/workflow-execution/jobs/{j}/enqueue", {})
    return run_jobs(w, "B", jobs)


def scenario_C(w: World, n: int) -> dict:
    """One INBOUND request of n boxes: split into trips of <= 3, both robots take one, pickup at INBOUND in turn."""
    new = new_containers(w, n, f"C{n}")
    voucher(w, "INBOUND", new, f"real 2-robot C{n}")
    shelves = ["SHELF-A-1", "SHELF-A-2", "SHELF-A-1", "SHELF-C-1", "SHELF-B-2", "SHELF-C-1"]
    tr, jobs = create_tr(w, "INBOUND", [{"container": c, "src": "EP-INBOUND", "dst": f"EP-{s}", "dstLoc": s}
                                         for c, s in zip(new, shelves)], f"real 2-robot C{n}")
    print(f"  TR {tr[:8]}: {len(jobs)} trips")
    w.api.call("POST", f"/api/workflow-execution/jobs/{jobs[0]}/enqueue", {})
    return run_jobs(w, f"C{n}", jobs, timeout_s=1800)


def scenario_HOME(w: World) -> dict:
    """Both robots drive home at the same time (RobotCare return-to-park), from wherever they are."""
    jobs = []
    for code, home in (("RBT-001", "EP-PARKING-A"), ("RBT-002", "EP-PARKING-B")):
        j = w.api.call("POST", f"/api/workflow-execution/robots/{w.robots[code]}/return-to-park", {"endpointId": w.eps[home][0]})
        jobs.append(j["id"])
    return run_jobs(w, "HOME", jobs, timeout_s=600)


def scenario_WATCH(w: World) -> dict:
    """Pick up the Jobs already running (after a Host restart) and see them through."""
    jobs = [j for j, in sql("select id from execution.jobs where status in ('RUNNING','QUEUED','ASSIGNED','PAUSED','CREATED') order by created_at")]
    return run_jobs(w, "WATCH", jobs, timeout_s=1800)


def scenario_V1(w: World) -> dict:
    """V017 smoke: one INBOUND per robot, one after the other (RBT-001 -> SHELF-A-1, then RBT-002 -> SHELF-C-1)."""
    results = []
    for code, shelf in (("RBT-001", "SHELF-A-1"), ("RBT-002", "SHELF-C-1")):
        c = new_containers(w, 1, f"V1-{code}")
        voucher(w, "INBOUND", c, f"real V017 V1 {code}")
        _, js = create_tr(w, "INBOUND", [{"container": c[0], "src": "EP-INBOUND", "dst": w.loc_ep[shelf], "dstLoc": shelf}],
                          f"real V017 V1 {code}")
        dispatch(w, js[0], code)
        results.append(run_jobs(w, f"V1-{code}", js))
        if not results[-1]["pass"]:
            break
    res = dict(results[-1])
    res["pass"] = all(r["pass"] for r in results) and len(results) == 2
    res["minBodyGapM"] = min(r["minBodyGapM"] for r in results)
    res["seconds"] = round(sum(r["seconds"] for r in results), 1)
    res["safetyStop"] = any(r["safetyStop"] for r in results)
    return res


def scenario_V2(w: World) -> dict:
    """V017 parallel: two INBOUND at once (sim scenario 1): RBT-001 INBOUND -> SHELF-C-1, RBT-002 INBOUND -> SHELF-B-2
    (EP-SHELF-B-01-02). Both pick up at the INBOUND dead-end bay; the second waits at a queue point / yield spot."""
    jobs = []
    for code, shelf in (("RBT-001", "SHELF-C-1"), ("RBT-002", "SHELF-B-2")):
        c = new_containers(w, 1, f"V2-{code}")
        voucher(w, "INBOUND", c, f"real V017 V2 {code}")
        tr, js = create_tr(w, "INBOUND", [{"container": c[0], "src": "EP-INBOUND", "dst": w.loc_ep[shelf], "dstLoc": shelf}],
                           f"real V017 V2 {code}")
        jobs.append((code, js[0]))
    for code, job in jobs:
        dispatch(w, job, code)
    return run_jobs(w, "V2", [j for _, j in jobs])


# ---------------------------------------------------------------- MAP-ONE-V024 (2026-11-25, scanned from the web)
# Shelves: SHELF-A-1-V2 / SHELF-A-2-V2 (east, top), SHELF-B (east, middle), SHELF-C (west, bottom). Homes: RBT-001
# PARKING-A, RBT-002 PARKING-B. Locked zones: SHELF-A, SHELF-B, CENTER; one-way lanes A-1/A-2, B-1/B-2, BOUND-1/2.
def _inbound(w: World, code: str | None, shelf: str, tag: str, check_quality: bool = False) -> str:
    c = new_containers(w, 1, tag)
    voucher(w, "INBOUND", c, f"real V024 {tag}")
    _, js = create_tr(w, "INBOUND", [{"container": c[0], "src": "EP-INBOUND", "dst": w.loc_ep[shelf], "dstLoc": shelf}],
                      f"real V024 {tag}", check_quality)
    if code:
        dispatch(w, js[0], code)
    return js[0]


def scenario_S1(w: World) -> dict:
    """RBT-001 alone: INBOUND -> SHELF-A-1-V2 (through lane A-1 and the SHELF-A zone)."""
    return run_jobs(w, "S1", [_inbound(w, "RBT-001", "SHELF-A-1-V2", "S1-RBT-001")])


def scenario_S2(w: World) -> dict:
    """RBT-002 alone: INBOUND with quality inspection -> EP-QUALITY-INSPECTION -> SHELF-B."""
    return run_jobs(w, "S2", [_inbound(w, "RBT-002", "SHELF-B", "S2-RBT-002", check_quality=True)])


def scenario_S3(w: World) -> dict:
    """Both at once to different places: RBT-001 INBOUND -> SHELF-C, RBT-002 INBOUND -> SHELF-A-2-V2. Both pick up at
    EP-INBOUND: the second waits until the first has left."""
    return run_jobs(w, "S3", [_inbound(w, "RBT-001", "SHELF-C", "S3-RBT-001"),
                              _inbound(w, "RBT-002", "SHELF-A-2-V2", "S3-RBT-002")])


def scenario_S4(w: World) -> dict:
    """Crossing: RBT-001 OUTBOUND from SHELF-A-1-V2 -> EP-OUTBOUND, RBT-002 INTERNAL_RELOCATION SHELF-B -> SHELF-C."""
    c1 = stored_at(w, "SHELF-A-1-V2")
    voucher(w, "OUTBOUND", [c1], "real V024 S4 outbound")
    _, j1 = create_tr(w, "OUTBOUND_RETRIEVAL", [{"container": c1, "src": w.loc_ep["SHELF-A-1-V2"], "srcLoc": "SHELF-A-1-V2",
                                                 "dst": "EP-OUTBOUND"}], "real V024 S4 RBT-001")
    c2 = stored_at(w, "SHELF-B")
    _, j2 = create_tr(w, "INTERNAL_RELOCATION", [{"container": c2, "src": w.loc_ep["SHELF-B"], "srcLoc": "SHELF-B",
                                                  "dst": w.loc_ep["SHELF-C"], "dstLoc": "SHELF-C"}], "real V024 S4 RBT-002")
    dispatch(w, j1[0], "RBT-001")
    dispatch(w, j2[0], "RBT-002")
    return run_jobs(w, "S4", [j1[0], j2[0]])


def scenario_S5(w: World) -> dict:
    """One INBOUND request of 3 boxes (3 trips, 1 box each): the dispatcher gives them out R1 - R2 - R1."""
    new = new_containers(w, 3, "S5")
    voucher(w, "INBOUND", new, "real V024 S5")
    shelves = ["SHELF-A-1-V2", "SHELF-B", "SHELF-A-2-V2"]
    tr, jobs = create_tr(w, "INBOUND", [{"container": c, "src": "EP-INBOUND", "dst": w.loc_ep[s], "dstLoc": s}
                                         for c, s in zip(new, shelves)], "real V024 S5")
    print(f"  TR {tr[:8]}: {len(jobs)} trips")
    w.api.call("POST", f"/api/workflow-execution/jobs/{jobs[0]}/enqueue", {})
    return run_jobs(w, "S5", jobs, timeout_s=1800)


def scenario_S6(w: World) -> dict:
    """Three INBOUND requests of one box, queued together: the dispatcher hands them out R1 - R2 - R1 (the third goes
    to whichever robot is free first)."""
    jobs = []
    for i, shelf in enumerate(["SHELF-C", "SHELF-B", "SHELF-A-1-V2"], 1):
        jobs.append(_inbound(w, None, shelf, f"S6-{i}"))
        w.api.call("POST", f"/api/workflow-execution/jobs/{jobs[-1]}/enqueue", {})
    return run_jobs(w, "S6", jobs, timeout_s=1800)


SCENARIOS = {"S6": scenario_S6, "S1": scenario_S1, "S2": scenario_S2, "S3": scenario_S3, "S4": scenario_S4, "S5": scenario_S5,
             "V1": scenario_V1, "V2": scenario_V2, "WATCH": scenario_WATCH, "HOME": scenario_HOME, "A1": scenario_A1, "A2": scenario_A2, "A3": scenario_A3, "B": scenario_B,
             "C6": lambda w: scenario_C(w, 6), "C5": lambda w: scenario_C(w, 5)}


def main(names: list[str]) -> int:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    w = World()
    summary = {}
    for name in names:
        print(f"\n===== {name}  (RBT-001 near {nearest(w, 'RBT-001')}, RBT-002 near {nearest(w, 'RBT-002')})")
        res = SCENARIOS[name](w)
        summary[name] = {"pass": res["pass"], "minBodyGapM": res["minBodyGapM"], "seconds": res["seconds"],
                         "safetyStop": res["safetyStop"], "parallelPct": res["parallelPct"],
                         "bothMovingSeconds": res["bothMovingSeconds"]}
        if not res["pass"]:
            print(f"===== {name} FAILED: stop here")
            break
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    print("\nSUMMARY", json.dumps(summary, indent=1), "\nartifacts:", OUT)
    return 0 if all(v["pass"] for v in summary.values()) and len(summary) == len(names) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:] or list(SCENARIOS)))
