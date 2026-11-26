"""Payload recovery on the two REAL robots (V024), after WAROTRANS_PAYLOAD_RECOVERY_DESIGN.md.

Every case: one transport request of 7 boxes -> trips of 3 + 3 + 1. The first trip is forced on RBT-002 (the robot that
fails), the second on RBT-001; the 1-box trip goes to the next free robot (RBT-001, since RBT-002 has failed). RBT-001
finishes its own work first, then rescues the cargo still in RBT-002's custody at the recovery point (EP-QUEUE) on the
SAME Job, and goes on with the remaining steps (quality inspection when still due, then the destination).

    P1  INBOUND + QI    RBT-002 fails at QUALITY-INSPECTION, can still drive: technician approves self-evacuation to a
                        free parking place, secures it; RBT-001 does the inspection for the rescued boxes.
    P2  RELOCATION      total disconnection (whole robot stack stopped) after its pickups; technician confirms where it
                        stands (an endpoint), cargo taken off by staff, loaded on RBT-001.
    P3  OUTBOUND + QI   offline; technician accepts the LAST KNOWN POSE as the place; RBT-001 takes it through QI.
    P4  RELOCATION      partial delivery: RBT-002 delivers 1 box, then fails driving to the next shelf (can drive):
                        only the 2 boxes still on it are rescued; the delivered one is never delivered again.
    P5  OUTBOUND        RBT-002 is lost BEFORE any pickup: no recovery, the trip goes back to the queue for RBT-001.

The failing robot only goes offline while it stands still (STOP hold first), so nothing is cut while it drives.
Afterwards RBT-002 is in maintenance (cannot take work) until its return to service at the end of the case.

    WARO_PI_PW=... python tools/traffic_e2e/real_recovery_run.py P1 P2 P3 P4 P5
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from real_two_robot_run import (OUT, TERMINAL, Watch, World, _pick, confirm_waiting, create_tr, dispatch,  # noqa: E402
                                new_containers, robot_code, sql, voucher)

JOBS = "/api/workflow-execution/jobs"
TR = "/api/transportation/transport-requests"
FLEET = "/api/fleet/robots"
CHECKLIST = [{"item": i, "ok": True} for i in (
    "Motor and driver stop safely", "Encoders / odometry trusted", "Lidar and safety sensors work", "Pose is right on the map",
    "Connection is stable", "Cargo sits firmly", "STOP, collision monitor and watchdog active", "Battery enough for the way")]
FAILED, RESCUER = "RBT-002", "RBT-001"
PI_TOOLS = r"E:\programming\my_project\warotrans-system\tools\technical_e2e"


# ---------------------------------------------------------------- robot side (RBT-002 service), password only from env
def rbt2_service(action: str) -> str:
    os.environ["WARO_PI_HOST"] = "warotrans-rbt002.local"
    sys.path.insert(0, PI_TOOLS)
    import pi  # noqa: PLC0415
    c = pi.connect()
    try:
        code, out = pi.run(c, f"sudo -n /usr/bin/systemctl {action} warotrans-rbt002.service; sleep 2; systemctl is-active warotrans-rbt002",
                           timeout=120)
        return out.strip().splitlines()[-1] if out.strip() else str(code)
    finally:
        c.close()


# ---------------------------------------------------------------- helpers
def say(msg: str) -> None:
    print(f"  ## {msg}", flush=True)


def rec_get(w: World, job: str) -> dict:
    return w.api.call("GET", f"{JOBS}/{job}/recovery")["recovery"]


def ref(r: dict) -> dict:
    return {"caseId": r["id"], "version": r["version"]}


def job_get(w: World, job: str) -> dict:
    return w.api.call("GET", f"{JOBS}/{job}")


def onboard(j: dict) -> list[dict]:
    return [c for c in j.get("containers", []) if c["status"] == "ONBOARD"]


def delivered(j: dict) -> list[dict]:
    return [c for c in j.get("containers", []) if c["status"] == "DELIVERED"]


def waiting_step(j: dict) -> dict | None:
    for t in sorted(j["tasks"], key=lambda t: t["sequenceNo"]):
        for s in t["steps"]:
            if s["status"] == "WAITING" and s.get("stepType") == "HUMAN_INTERACTION":
                return {**s, "task": t}
    return None


def moving_to(j: dict) -> dict | None:
    for t in j["tasks"]:
        for s in t["steps"]:
            if s.get("stepType") == "MOVE" and s["status"] == "EXECUTING":
                return {**s, "task": t}
    return None


def active_jobs_of(w: World, code: str) -> list[str]:
    return [r[0] for r in sql("select j.job_no from execution.jobs j join execution.job_robot_assignments a on a.job_id=j.id "
                              f"and a.ended_at is null where a.robot_id='{w.robots[code]}' "
                              "and j.status in ('RUNNING','QUEUED','ASSIGNED','PAUSED','REASSIGNING','RECOVERY_REQUIRED')")]


class Run:
    """Drives the jobs of one case: confirms the person steps of every job except the held ones, prints changes."""

    def __init__(self, w: World, name: str, jobs: list[str]):
        self.w, self.name, self.jobs = w, name, jobs
        self.hold: set[str] = set()
        self.log: list = []
        self.seen: dict[str, str] = {}
        self.states: dict[str, dict] = {}
        self.t0 = time.time()
        self.watch = Watch(w, name)
        self.watch.start()

    def tick(self) -> dict[str, dict]:
        free = [j for j in self.jobs if j not in self.hold]
        self.states = confirm_waiting(self.w, free, self.log)
        for j in self.hold:
            self.states[j] = job_get(self.w, j)
        for k, v in self.states.items():
            line = f"{v['status']} robot={robot_code(self.w, v.get('assignedRobotId'))} {v['completedSteps']}/{v['totalSteps']}"
            if self.seen.get(k) != line:
                self.seen[k] = line
                print(f"  [{time.time() - self.t0:6.1f}s] {v['jobNo']} {line} {v.get('blockingReason') or v.get('failureCode') or ''}",
                      flush=True)
        if self.watch.tripped:
            raise AssertionError("safety watch tripped (robots too close)")
        return self.states

    def until(self, cond, timeout: float, what: str):
        end = time.time() + timeout
        while time.time() < end:
            st = self.tick()
            got = cond(st)
            if got:
                return got
            time.sleep(0.5)
        raise AssertionError(f"timed out after {timeout:.0f}s waiting for {what}")

    def stop(self) -> None:
        self.watch.stop()


def make_tr(w: World, workflow: str, tag: str, rows: list[tuple], qi: bool) -> tuple[str, list[str]]:
    """rows: (container, src endpoint code, src loc|None, dst endpoint code, dst loc|None) in trip order."""
    moves = [{"container": c, "src": s, "srcLoc": sl, "dst": d, "dstLoc": dl} for c, s, sl, d, dl in rows]
    return create_tr(w, workflow, moves, f"real V024 recovery {tag}", qi)


def split_trips(w: World, jobs: list[str], first: str, second: str) -> tuple[str, str, list[str]]:
    """The trip with box `first` (for RBT-002), the trip with box `second` (for RBT-001), and the rest."""
    info = {j: job_get(w, j) for j in jobs}
    has = lambda c: next(j for j, v in info.items() if any(x["containerId"] == c for x in v["containers"]))  # noqa: E731
    j2, j1 = has(first), has(second)
    assert j2 != j1, "both boxes on one trip"
    say("trips: " + ", ".join(f"{v['jobNo']} {len(v['containers'])} box(es)"
                              f"{' -> ' + FAILED if j == j2 else ' -> ' + RESCUER if j == j1 else ' -> next free robot'}"
                              for j, v in info.items()))
    return j2, j1, [j for j in jobs if j not in (j2, j1)]


def stop_robot(w: World, code: str, why: str) -> None:
    w.api.call("POST", f"{FLEET}/{w.robots[code]}/commands/stop", {"reason": why}, ok=(200, 202, 409))


def secure_flow(w: World, job: str, mode: str, verify: str | None) -> dict:
    """Technician side. mode: SELF_EVAC (assess OK, drive to a free parking place, secure) / MANUAL (offline)."""
    r = rec_get(w, job)
    say(f"recovery case open: phase={r['phase']} fault=({r['fault'].get('x')}, {r['fault'].get('y')}) "
        f"online={r['fault'].get('robotOnline')} containers={[c['barcode'] for c in r['containers']]}")
    if mode == "SELF_EVAC":
        r = w.api.call("POST", f"{JOBS}/{job}/recovery/assessment",
                       {**ref(r), "result": "SELF_EVACUATION_APPROVED", "checklist": CHECKLIST, "note": "lab: robot can drive"})["recovery"]
        cands = w.api.call("GET", f"{JOBS}/{job}/recovery/endpoint-candidates?for=SOURCE")["items"]
        say("failed-robot points: " + ", ".join(f"{p['code']}({p['pathMeters']}m{', ' + p['unsuitable'] if p['unsuitable'] else ''})" for p in cands))
        free = next((p for p in cands if p["unsuitable"] is None), None)
        if free:
            r = w.api.call("POST", f"{JOBS}/{job}/recovery/source-evacuate", {**ref(r), "endpointId": free["endpointId"]})["recovery"]
            say(f"{FAILED} self-evacuates with its cargo to {free['code']}")
            end = time.time() + 240
            while time.time() < end:
                time.sleep(2)
                r = rec_get(w, job)
                try:
                    r = w.api.call("POST", f"{JOBS}/{job}/recovery/source-secure", {**ref(r), "note": f"lab: arrived at {free['code']}"})["recovery"]
                    break
                except AssertionError as exc:
                    if "recovery_source_not_arrived" not in str(exc):
                        raise
            else:
                raise AssertionError("failed robot did not reach its recovery point")
        else:
            r = w.api.call("POST", f"{JOBS}/{job}/recovery/source-secure", {**ref(r), "note": "lab: secured where it stands"})["recovery"]
    else:
        r = w.api.call("POST", f"{JOBS}/{job}/recovery/assessment",
                       {**ref(r), "result": "MANUAL_RECOVERY_REQUIRED", "checklist": [{"item": "Connection is stable", "ok": False}],
                        "note": "lab: robot unreachable"})["recovery"]
        if verify == "ENDPOINT":
            ep = r.get("faultEndpointId") or None
            x, y = r["fault"]["x"], r["fault"]["y"]
            near = min(w.eps.items(), key=lambda e: math.hypot(e[1][1] - x, e[1][2] - y))
            r = w.api.call("POST", f"{JOBS}/{job}/recovery/verify-location", {**ref(r), "endpointId": ep or near[1][0]})["recovery"]
            say(f"technician confirms {FAILED} stands at {near[0]}")
        else:
            r = w.api.call("POST", f"{JOBS}/{job}/recovery/verify-location", {**ref(r), "x": r["fault"]["x"], "y": r["fault"]["y"]})["recovery"]
            say(f"technician accepts the last known pose ({r['fault']['x']:.2f}, {r['fault']['y']:.2f})")
        r = w.api.call("POST", f"{JOBS}/{job}/recovery/source-secure", {**ref(r), "note": "lab: robot off, secured"})["recovery"]
    assert r["phase"] == "SOURCE_SECURED", r["phase"]
    say(f"{FAILED} secured (phase SOURCE_SECURED)")
    return r


def rescue(run: Run, job: str, unload_first: bool) -> dict:
    """Staff side: wait until RBT-001 has no own work left, send it to the recovery point, transfer, resume."""
    w = run.w
    run.until(lambda s: not [j for j in run.jobs if j != job and s[j]["status"] not in TERMINAL] and not active_jobs_of(w, RESCUER),
              2400, f"{RESCUER} to finish its own trips")
    say(f"{RESCUER} finished its own trips -> starts the rescue")
    r = rec_get(w, job)
    r = w.api.call("POST", f"{JOBS}/{job}/recovery/claim", ref(r))["recovery"]
    scans = [{"containerId": c["containerId"], "barcode": c["barcode"]} for c in r["containers"]]
    if unload_first:
        r = w.api.call("POST", f"{JOBS}/{job}/recovery/unload-confirm",
                       {**ref(r), "scans": scans, "holder": "STAFF", "note": "lab: staff took the boxes off"})["recovery"]
        say(f"staff took {len(scans)} box(es) off {FAILED}: holders {[c['holderType'] for c in r['containers']]}")
    targets = w.api.call("GET", f"{JOBS}/{job}/recovery/endpoint-candidates?for=TARGET")["items"]
    say("rescue points: " + ", ".join(f"{p['code']}({p['pathMeters']}m{', ' + p['unsuitable'] if p['unsuitable'] else ''})" for p in targets))
    target = next(p for p in targets if p["unsuitable"] is None)
    r = w.api.call("POST", f"{JOBS}/{job}/recovery/assign", {**ref(r), "robotId": w.robots[RESCUER], "endpointId": target["endpointId"]})["recovery"]
    say(f"{RESCUER} sent to {target['code']} to take {len(scans)} box(es)")
    r = run.until(lambda s: (x := rec_get(w, job))["robotArrived"] and x, 600, f"{RESCUER} at {target['code']}")
    r = w.api.call("POST", f"{JOBS}/{job}/recovery/transfer-confirm",
                   {**ref(r), "robotId": w.robots[RESCUER], "scans": scans, "note": "lab: loaded on the rescue robot"})["recovery"]
    assert r["status"] == "TRANSFERRED", r["status"]
    say(f"boxes on {RESCUER}: {[c['holderType'] for c in r['containers']]}; resume the SAME job")
    w.api.call("POST", f"{JOBS}/{job}/resume", {})
    run.hold.discard(job)
    return {"target": target["code"], "rescued": [c["barcode"] for c in r["containers"]]}


def maintenance(w: World, start: bool) -> None:
    rid = w.robots[FAILED]
    if start:
        w.api.call("POST", f"{FLEET}/{rid}/maintenance/start", {"reason": "payload recovery lab: robot failed"}, ok=(200, 201, 409))
    else:
        w.api.call("POST", f"{FLEET}/{rid}/maintenance/complete",
                   {"sensorsOk": True, "localizationOk": True, "stopTestOk": True, "trafficClear": True, "notes": "lab: back in service"},
                   ok=(200, 201, 409))


def return_to_service(w: World, was_offline: bool) -> None:
    """RBT-002 back: stack up again if it was stopped, relocalized, maintenance completed, hold released, home."""
    rid = w.robots[FAILED]
    if was_offline:
        say(f"{FAILED} stack start: {rbt2_service('start')}")
        end = time.time() + 120
        while time.time() < end and not w.robot(FAILED).get("isOnline", True):
            time.sleep(3)
        time.sleep(8)
    try:
        w.api.call("POST", f"{FLEET}/{rid}/commands/auto-localize", {"allowSpin": False})
        time.sleep(12)
    except AssertionError as exc:
        say(f"auto-localize: {str(exc)[:120]}")
    r = w.robot(FAILED)
    say(f"{FAILED}: online={r.get('isOnline')} {r['localizationStatus']} ({r['poseX']:.2f}, {r['poseY']:.2f}) status={r.get('status')}")
    maintenance(w, False)
    w.api.call("POST", f"{FLEET}/{rid}/commands/resume", {}, ok=(200, 202, 409))
    say(f"{FAILED} returned to service")


def verify(w: World, name: str, tr: str, job: str, before_trs: int, rescued: dict, extra: dict) -> dict:
    j = job_get(w, job)
    req = w.api.call("GET", f"{TR}/{tr}")
    hist = [robot_code(w, r) for r, in sql(f"select robot_id from execution.job_robot_assignments where job_id='{job}' order by assigned_at")]
    trs_after = int(sql("select count(*) from transportation.transport_requests")[0][0])
    # Each container dropped once: count the drop-off confirmations per container on this Job.
    drops = sql("select c.barcode, count(*) from warehouse.container_movements m join warehouse.containers c on c.id=m.container_id "
                f"where m.created_at > (select created_at from transportation.transport_requests where id='{tr}') "
                f"and c.id in (select container_id from execution.job_containers where job_id='{job}') group by 1")
    res = {"scenario": name, "request": req["status"], "job": j["status"], "jobNo": j["jobNo"], "assignmentHistory": hist,
           "newRequestsCreated": trs_after - before_trs - 1, "containerMovements": {b: int(n) for b, n in drops},
           **rescued, **extra}
    res["pass"] = (req["status"] == "COMPLETED" and j["status"] == "COMPLETED" and res["newRequestsCreated"] == 0
                   and hist[:1] == [FAILED] and RESCUER in hist)
    return res


# ---------------------------------------------------------------- the cases
def _new_inbound(w: World, n: int, tag: str) -> list[str]:
    c = new_containers(w, n, tag)
    voucher(w, "INBOUND", c, f"real V024 recovery {tag}")
    return c


def case(w: World, name: str, workflow: str, rows: list[tuple], qi: bool, trigger, mode: str, verify_by: str | None,
         offline: bool, unload_first: bool) -> dict:
    before = int(sql("select count(*) from transportation.transport_requests")[0][0])
    tr, jobs = make_tr(w, workflow, name, rows, qi)
    j2, j1, rest = split_trips(w, jobs, rows[0][0], rows[3][0])
    dispatch(w, j2, FAILED)
    dispatch(w, j1, RESCUER)
    for j in rest:
        w.api.call("POST", f"{JOBS}/{j}/enqueue", {})
    run = Run(w, name, jobs)
    extra: dict = {}
    try:
        # 1) drive until the fault point of RBT-002's trip; from then its steps are not confirmed.
        def hit(states):
            j = states[j2]
            return j["status"] == "RUNNING" and trigger(w, j) and j
        run.hold.add(j2)  # its person steps are confirmed by the trigger logic below
        st = run.until(lambda s: _advance(w, s[j2], trigger) or hit(s), 1500, f"{FAILED} at its fault point")
        extra["faultAt"] = {"onboard": [c["barcode"] for c in onboard(st)], "delivered": [c["barcode"] for c in delivered(st)]}
        say(f"FAULT on {FAILED} ({mode}{', offline' if offline else ''}): on board {extra['faultAt']['onboard']}, "
            f"already delivered {extra['faultAt']['delivered']}")
        # 2) inject
        if offline:
            # Held first (it stands at its last pickup), then the last box is confirmed on board: it cannot drive off.
            stop_robot(w, FAILED, "lab: payload recovery fault injection")
            time.sleep(2)
            s = waiting_step(job_get(w, j2))
            if s:
                container = (s.get("resolvedInputs") or {}).get("containerId") or s["task"].get("containerId")
                w.api.call("POST", f"/api/workflow-execution/job-steps/{s['id']}/confirm",
                           {"containerId": container, "note": "real recovery lab staff"})
                say(f"last box {container[:8]} loaded on the held {FAILED}")
                time.sleep(2)
            extra["faultAt"]["onboard"] = [c["barcode"] for c in onboard(job_get(w, j2))]
            say(f"{FAILED} whole robot stack stopped: {rbt2_service('stop')}")
        else:
            w.api.call("POST", f"{JOBS}/{j2}/pause", {"reason": f"lab {name}: robot cannot go on"})
            run.until(lambda s: s[j2]["status"] == "PAUSED", 60, "the job paused")
            w.api.call("POST", f"{JOBS}/{j2}/recovery/request", {"reason": f"lab {name}: robot failed with cargo"})
        # 3) the case opens (offline: after the grace time, by itself)
        run.until(lambda s: s[j2]["status"] == "RECOVERY_REQUIRED", 400, "the recovery case to open")
        r = secure_flow(w, j2, mode, verify_by)
        extra["phaseAfterSecure"] = r["phase"]
        # 4) RBT-002 may not take work: maintenance until return to service
        # (it still holds the Job until the transfer; maintenance starts right after it)
        rescued = rescue(run, j2, unload_first)
        maintenance(w, True)
        extra["failedRobotAfterTransfer"] = {"status": w.robot(FAILED).get("status"), "activeJobs": active_jobs_of(w, FAILED)}
        say(f"{FAILED} in maintenance: {extra['failedRobotAfterTransfer']}")
        # 5) RBT-001 finishes the SAME job (inspection if still due, destination)
        run.until(lambda s: all(v["status"] in TERMINAL for v in s.values()), 1500, "every trip to finish")
        extra["failedRobotTookWork"] = bool(active_jobs_of(w, FAILED))
    finally:
        run.stop()
    res = verify(w, name, tr, j2, before, rescued, extra)
    res["minBodyGapM"] = round(run.watch.min_gap, 3)
    res["jobs"] = {v["jobNo"]: {"status": v["status"], "robot": robot_code(w, v.get("assignedRobotId")),
                                "containers": len(v.get("containers") or [])} for v in run.states.values()}
    return_to_service(w, offline)
    return res


def _advance(w: World, j: dict, trigger) -> bool:
    """Confirm RBT-002's person steps by hand (it is held from auto-confirm) unless the trigger wants that stop."""
    if j["status"] != "RUNNING" or trigger(w, j):
        return False
    s = waiting_step(j)
    if s:
        container = (s.get("resolvedInputs") or {}).get("containerId") or s["task"].get("containerId")
        if container or s.get("operationCode") == "INSPECTION_CONFIRM":
            body = {"containerId": container, "note": "real recovery lab staff"} if container else {"note": "real recovery lab QI"}
            container = container or "all-boxes"
            if "INSPECTION" in str(s.get("stepKey")):
                body["inspectionResult"] = "PASSED"
            try:
                w.api.call("POST", f"/api/workflow-execution/job-steps/{s['id']}/confirm", body)
                print(f"  confirmed {j['jobNo']} [{FAILED}] {s.get('name')} {container[:8]}", flush=True)
            except AssertionError as exc:
                print(f"  confirm refused {str(exc)[:120]}", flush=True)
    return False


def at_inspection(w, j):
    s = waiting_step(j)
    return bool(s and "INSPECTION" in str(s.get("stepKey")) and onboard(j))


def at_last_pickup(w, j):
    """Standing still at its last pickup, every other box on board (the fault leaves it at the pickup shelf, not on
    a destination that the rescued boxes still need)."""
    s = waiting_step(j)
    return bool(s and not delivered(j) and len(onboard(j)) == len(j["containers"]) - 1)


def leaving_after_one_drop(w, j):
    m = moving_to(j)
    return bool(m and delivered(j) and onboard(j))


START: dict = {}


def before_pickup_moving(w, j):
    """Driving to its first pickup, 0.7 m away from where it started, nothing on board."""
    if not moving_to(j) or onboard(j) or delivered(j):
        return False
    r = w.robot(FAILED)
    x0, y0 = START.setdefault("p", (r["poseX"], r["poseY"]))
    return math.hypot(r["poseX"] - x0, r["poseY"] - y0) >= 0.7


def P1(w):  # INBOUND + QI, RBT-002 fails at QI, can drive
    c = _new_inbound(w, 7, "P1")
    dst = ["SHELF-B", "SHELF-B", "SHELF-B", "SHELF-A-2-V2", "SHELF-A-2-V2", "SHELF-C", "SHELF-C"]
    rows = [(x, "EP-INBOUND", None, w.loc_ep[d], d) for x, d in zip(c, dst)]
    return case(w, "P1", "INBOUND", rows, True, at_inspection, "SELF_EVAC", None, offline=False, unload_first=False)


def _reloc_rows(w, pairs, used):
    return [(_pick(w, s, 1, used)[0], w.loc_ep[s], s, w.loc_ep[d], d) for s, d in pairs]


def P2(w):  # RELOCATION, total disconnection after its pickups
    used: set = set()
    pairs = [("SHELF-B", "SHELF-C")] * 3 + [("SHELF-A-1-V2", "SHELF-A-2-V2")] * 2 + [("SHELF-A-2-V2", "SHELF-A-1-V2")] * 2
    return case(w, "P2", "INTERNAL_RELOCATION", _reloc_rows(w, pairs, used), False, at_last_pickup, "MANUAL", "ENDPOINT",
                offline=True, unload_first=True)


def P3(w):  # OUTBOUND + QI, offline, last known pose
    used: set = set()
    shelves = ["SHELF-B"] * 3 + ["SHELF-A-1-V2", "SHELF-A-2-V2", "SHELF-C", "SHELF-C"]
    boxes = [(_pick(w, s, 1, used)[0], s) for s in shelves]
    voucher(w, "OUTBOUND", [b for b, _ in boxes], "real V024 recovery P3")
    rows = [(b, w.loc_ep[s], s, "EP-OUTBOUND", None) for b, s in boxes]
    return case(w, "P3", "OUTBOUND_RETRIEVAL", rows, True, at_last_pickup, "MANUAL", "LAST_POSE", offline=True, unload_first=True)


def P4(w):  # RELOCATION, partial delivery then failure while driving (can drive)
    used: set = set()
    pairs = [("SHELF-C", "SHELF-A-2-V2"), ("SHELF-C", "SHELF-B"), ("SHELF-C", "SHELF-B")] + \
            [("SHELF-A-1-V2", "SHELF-C")] * 2 + [("SHELF-B", "SHELF-A-1-V2")] * 2
    return case(w, "P4", "INTERNAL_RELOCATION", _reloc_rows(w, pairs, used), False, leaving_after_one_drop, "SELF_EVAC", None,
                offline=False, unload_first=False)


def P5(w):  # OUTBOUND, lost before any pickup -> back to the queue, no recovery
    used: set = set()
    shelves = ["SHELF-B"] * 3 + ["SHELF-A-1-V2", "SHELF-A-2-V2", "SHELF-C", "SHELF-C"]
    boxes = [(_pick(w, s, 1, used)[0], s) for s in shelves]
    voucher(w, "OUTBOUND", [b for b, _ in boxes], "real V024 recovery P5")
    tr, jobs = make_tr(w, "OUTBOUND_RETRIEVAL", "P5", [(b, w.loc_ep[s], s, "EP-OUTBOUND", None) for b, s in boxes], False)
    START.clear()
    START["p"] = (w.robot(FAILED)["poseX"], w.robot(FAILED)["poseY"])
    j2, j1, rest = split_trips(w, jobs, boxes[0][0], boxes[3][0])
    dispatch(w, j2, FAILED)
    dispatch(w, j1, RESCUER)
    for j in rest:
        w.api.call("POST", f"{JOBS}/{j}/enqueue", {})
    run = Run(w, "P5", jobs)
    extra: dict = {}
    try:
        run.hold.add(j2)
        run.until(lambda s: s[j2]["status"] == "RUNNING" and before_pickup_moving(w, s[j2]), 600, f"{FAILED} on its way to the pickup")
        # Lost while driving: the stack goes down mid-drive (the motor board's command watchdog stops the wheels).
        say(f"FAULT on {FAILED} before any pickup: whole stack stopped while driving")
        say(f"{FAILED} stack: {rbt2_service('stop')}")
        st = run.until(lambda s: s[j2]["status"] in ("QUEUED", "RUNNING") and robot_code(w, s[j2].get("assignedRobotId")) != FAILED
                       and s[j2], 400, "the trip back in the queue")
        extra["requeued"] = {"status": st["status"], "blockingReason": st.get("blockingReason")}
        extra["recoveryOpened"] = bool(sql(f"select 1 from execution.job_recovery_cases where job_id='{j2}'"))
        say(f"trip back in the queue: {extra['requeued']}; recovery case opened: {extra['recoveryOpened']}")
        run.hold.discard(j2)
        maintenance(w, True)
        run.until(lambda s: all(v["status"] in TERMINAL for v in s.values()), 2400, "every trip to finish")
    finally:
        run.stop()
    j = job_get(w, j2)
    hist = [robot_code(w, r) for r, in sql(f"select robot_id from execution.job_robot_assignments where job_id='{j2}' order by assigned_at")]
    req = w.api.call("GET", f"{TR}/{tr}")
    res = {"scenario": "P5", "request": req["status"], "job": j["status"], "jobNo": j["jobNo"], "assignmentHistory": hist,
           "minBodyGapM": round(run.watch.min_gap, 3), **extra}
    res["pass"] = req["status"] == "COMPLETED" and not extra["recoveryOpened"] and hist[0] == FAILED and hist[-1] == RESCUER
    return_to_service(w, True)
    return res


CASES = {"P1": P1, "P2": P2, "P3": P3, "P4": P4, "P5": P5}


def main(names: list[str]) -> int:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    w = World()
    summary = {}
    for n in names:
        print(f"\n===== {n}", flush=True)
        try:
            res = CASES[n](w)
        except Exception as exc:  # noqa: BLE001
            res = {"scenario": n, "pass": False, "error": str(exc)[:400]}
        print(json.dumps(res, indent=1, default=str), flush=True)
        summary[n] = res
        OUT.mkdir(parents=True, exist_ok=True)
        (OUT / f"{n}_recovery.json").write_text(json.dumps(res, indent=1, default=str), encoding="utf-8")
        if not res["pass"]:
            print(f"===== {n} FAILED: stop here", flush=True)
            break
    print("\nSUMMARY", json.dumps({k: v["pass"] for k, v in summary.items()}), "\nartifacts:", OUT, flush=True)
    return 0 if all(v["pass"] for v in summary.values()) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
