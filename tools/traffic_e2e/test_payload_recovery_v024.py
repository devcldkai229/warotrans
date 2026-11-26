"""Payload recovery on MAP-ONE-V024 with the real Host and two SimRobots (WAROTRANS_PAYLOAD_RECOVERY_DESIGN.md).

PR-A  self-evacuation: R1 carries a container and cannot go on (its Job is paused). Staff ask for a payload recovery;
      a technician assesses R1 (every check passed) and confirms it secured where it stands; staff claim, send R2 to the recovery point = the queue point (EP-QUEUE,
      the only one on V024), scan the container onto R2, resume: R2 delivers on the SAME Job and the request completes.
PR-B  the rescue robot is refused before the failed robot is secured, and never sent to the failed robot's place.

    WARO_E2E_HOST_DLL=<Host build> python -m pytest tools/traffic_e2e/test_payload_recovery_v024.py -s
"""

from __future__ import annotations

import json
import time

import test_flows_v024 as flows
from test_flows_v024 import api, host, pair  # noqa: F401 - pytest fixtures

base = flows.base
JOBS = "/api/workflow-execution/jobs"
TR = "/api/transportation/transport-requests"
CHECKLIST = [{"item": item, "ok": True} for item in (
    "Motor and driver stop safely", "Encoders / odometry trusted", "Lidar and safety sensors work", "Pose is right on the map",
    "Connection is stable", "Cargo sits firmly", "STOP, collision monitor and watchdog active", "Battery enough for the way")]


def _recovery(api, job_id: str) -> dict:
    return api.call("GET", f"{JOBS}/{job_id}/recovery")["recovery"]


def _ref(rec: dict) -> dict:
    return {"caseId": rec["id"], "version": rec["version"]}


def _allowed(rec: dict, action: str) -> tuple[bool, str | None]:
    entry = next((a for a in rec["availableActions"] if a["action"] == action), None)
    return (bool(entry and entry["allowed"]), entry and entry.get("reason"))


def _onboard(job: dict) -> bool:
    return any(c["status"] == "ONBOARD" for c in job.get("containers", []))


def _moving(job: dict) -> bool:
    return any(st.get("stepType") == "MOVE" and st.get("status") == "EXECUTING" for t in job["tasks"] for st in t["steps"])


def _wait(cond, timeout: float, what: str, every: float = 1.0):
    end = time.time() + timeout
    last = None
    while time.time() < end:
        last = cond()
        if last:
            return last
        time.sleep(every)
    raise AssertionError(f"timed out after {timeout:.0f}s waiting for {what}")


def _sim(pair, robot_id: str):
    return pair[1] if pair[1].robot_id == robot_id else pair[2]


def test_PR_A_self_evacuation_then_a_second_robot_delivers_on_the_same_job(api, pair):
    flows._free_both_or_explain(api, pair[1], pair[2], "PR-A")
    eps = base.endpoints()
    tr, jobs = base.request(api, pair[0], [("EP-INBOUND", "EP-SHELF-B")])
    job_id = jobs[0]
    w = base.Watch(api, [pair[1], pair[2]], "PR-A").start()
    paused = {}

    def pause_with_cargo(states):
        job = states[job_id]
        if not paused and job["status"] == "RUNNING" and _onboard(job) and _moving(job):
            api.call("POST", f"{JOBS}/{job_id}/pause", {"reason": "e2e PR-A: robot cannot go on"})
            paused["at"] = time.time()

    w.run_until(lambda s: bool(paused), 400, jobs, on_tick=pause_with_cargo)
    failed_id = base.assignments(job_id)[-1][0]
    failed = _sim(pair, failed_id)
    other = pair[2] if failed is pair[1] else pair[1]
    steps: dict[str, object] = {}

    # Staff ask for a payload recovery (twice: one case).
    first = api.call("POST", f"{JOBS}/{job_id}/recovery/request", {"reason": "e2e"})["recovery"]
    again = api.call("POST", f"{JOBS}/{job_id}/recovery/request", {"reason": "pressed twice"})["recovery"]
    assert first["id"] == again["id"] and first["phase"] == "ASSESSMENT_REQUIRED", first
    assert first["fault"]["x"] is not None, first["fault"]

    # Before the technician: no second robot (PR-11).
    rec = _recovery(api, job_id)
    rec = api.call("POST", f"{JOBS}/{job_id}/recovery/claim", _ref(rec))["recovery"]
    allowed, reason = _allowed(rec, "ASSIGN")
    assert not allowed and reason == "recovery_source_not_secured", (allowed, reason)

    # Technician: assessment, self-evacuation to the free recovery point (QUEUE).
    rec = api.call("POST", f"{JOBS}/{job_id}/recovery/assessment",
                   {**_ref(rec), "result": "SELF_EVACUATION_APPROVED", "checklist": CHECKLIST, "note": "e2e"})["recovery"]
    # On V024 the parking column is a dead end with the queue point below it: a failed robot driven into it would block
    # the rescue robot. It stays where it failed (secured there); the rescue robot comes to the queue point.
    source_points = api.call("GET", f"{JOBS}/{job_id}/recovery/endpoint-candidates?for=SOURCE")["items"]
    steps["sourcePoints"] = [(p["code"], p["pathMeters"], p["unsuitable"]) for p in source_points]
    rec = api.call("POST", f"{JOBS}/{job_id}/recovery/source-secure", {**_ref(rec), "note": "e2e: held where it failed"})["recovery"]
    assert rec["phase"] == "SOURCE_SECURED", rec["phase"]

    # Floor: the second robot to the best free transfer point, never the failed robot's place.
    targets = api.call("GET", f"{JOBS}/{job_id}/recovery/endpoint-candidates?for=TARGET")["items"]
    steps["targetPoints"] = [(p["code"], p["pathMeters"], p["unsuitable"]) for p in targets]
    assert rec["sourceEndpointId"] is None  # the failed robot did not move
    target = next(p for p in targets if p["unsuitable"] is None)
    assert target["code"] == "EP-QUEUE", targets  # the recovery point is the queue point
    rec = api.call("POST", f"{JOBS}/{job_id}/recovery/assign",
                   {**_ref(rec), "robotId": other.robot_id, "endpointId": target["endpointId"]})["recovery"]
    assert rec["phase"] == "TARGET_EN_ROUTE", rec["phase"]
    rec = _wait(lambda: (r := _recovery(api, job_id))["robotArrived"] and r, 300, "the second robot at the transfer point")
    assert rec["phase"] == "AWAITING_TRANSFER", rec["phase"]

    scans = [{"containerId": c["containerId"], "barcode": c["barcode"]} for c in rec["containers"]]
    api.call("POST", f"{JOBS}/{job_id}/recovery/transfer-confirm", {**_ref(rec), "robotId": other.robot_id, "scans": scans, "note": "e2e"})
    done = _recovery(api, job_id)
    assert done["status"] == "TRANSFERRED", done["status"]
    assert all(c["holderType"] == "ROBOT" and c["holderId"] == other.robot_id for c in done["containers"]), done["containers"]
    assert base.assignments(job_id)[-1][0] == other.robot_id

    api.call("POST", f"{JOBS}/{job_id}/resume", {})
    states = w.run_until(base.terminal, 400, jobs)
    w.stop()
    w.assert_safe()
    request = api.call("GET", f"{TR}/{tr}")
    summary = {"job": states[job_id]["status"], "request": request["status"], **steps}
    print("[PR-A] " + json.dumps(summary))
    (flows.ARTIFACTS / "payload_recovery_summary.json").write_text(json.dumps(summary, indent=1), encoding="utf-8")
    assert states[job_id]["status"] == "COMPLETED", states[job_id]["status"]
    assert request["status"] == "COMPLETED", request["status"]

    # The failed robot stays held until a technician resumes it (maintenance); release it for the next scenario.
    api.call("POST", f"/api/fleet/robots/{failed.robot_id}/commands/resume", {}, ok=(200, 202, 409))
