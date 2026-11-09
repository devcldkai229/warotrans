"""WFX-04, FL-06, WEB-FLEET-06, FL-08, WH-06 against the real backend and two simulated robots.

Robot A goes silent with a Container on board. The Job waits in RECOVERY_REQUIRED with A's zones still held, so robot B is
held back until a person moves A and frees them. B reaches a safe endpoint, a person confirms the transfer, the same Job delivers.
B goes back to parking on request, and nothing moves by itself. Real robots: not involved, no command to them.
"""

from __future__ import annotations

import json
import math
import time

from harness import Api, Host, SYSTEM_REPO, wait_for
from test_frontend_flow import PATHS, _seed, _sql
from test_traffic_e2e import ARTIFACTS, FINGERPRINT, MAP_CODE, MAP_ID, PREFIX, RUN, ZONES, SimRobot

A = f"E2E-{RUN}-RCA"
B = f"E2E-{RUN}-RCB"
JOBS = "/api/workflow-execution/jobs"


def _publish(api: Api, code: str) -> str:
    body = json.loads((SYSTEM_REPO / "samples" / "workflows" / f"{code}.request.json").read_text(encoding="utf-8"))
    existing = [w for w in api.call("GET", "/api/workflow-execution/workflows")["items"] if w["code"] == code and w["status"] == "PUBLISHED"]
    if existing:
        return existing[0]["id"]
    wid = api.call("POST", "/api/workflow-execution/workflows", body)["id"]
    api.call("POST", f"/api/workflow-execution/workflows/{wid}/publish", {})
    return wid


def test_cargo_of_a_failed_robot_is_fetched_by_another_and_the_same_job_delivers():
    ARTIFACTS.mkdir(exist_ok=True)
    _seed(["--cleanup-e2e-robots"])
    host = Host(ARTIFACTS, name="recovery", extra_env={
        f"Traffic__RobotHomeEndpoints__{A}": "EP-PARKING-A",
        f"Traffic__RobotHomeEndpoints__{B}": "EP-PARKING-B",
        "Recovery__OfflineGraceSeconds": "5",
    })
    host.start()
    _seed([])
    sims: dict[str, SimRobot] = {}
    try:
        api = Api(host.base)
        api.login()
        eps = {c: (i, float(x), float(y)) for c, i, x, y in
               _sql(f"select code, id, x, y from navigation.endpoints where map_version_id = '{MAP_ID}'")}
        wid = _sql(f"select warehouse_id from navigation.map_versions where id = '{MAP_ID}'")[0][0]
        code_at = {(round(v[1], 3), round(v[2], 3)): c for c, v in eps.items()}

        def real_path(x, y, goal):
            to = code_at[(round(goal.x, 3), round(goal.y, 3))]
            start = min((c for c in eps if c != to), key=lambda c: math.hypot(eps[c][1] - x, eps[c][2] - y))
            return [tuple(q) for q in next(p for p in PATHS if p["from"] == start and p["to"] == to)["poses"]]

        ids = {}
        for code, home in ((A, "EP-PARKING-A"), (B, "EP-PARKING-B")):
            sims[code] = SimRobot(code=code, x=eps[home][1], y=eps[home][2], fingerprint=FINGERPRINT, zones=ZONES, prefix=PREFIX,
                                  map_code=MAP_CODE, path_provider=real_path, speed=0.5)
            ids[code] = api.call("POST", "/api/fleet/robots", {"warehouseId": wid, "currentMapVersionId": MAP_ID, "code": code, "name": code})["id"]
            sims[code].start()
            wait_for(lambda rid=ids[code]: (lambda r: r.get("isOnline") and r.get("localizationStatus") == "LOCALIZED")(
                api.call("GET", f"/api/fleet/robots/{rid}")), 15, what=f"{code} online and localized")
            api.call("POST", f"/api/fleet/robots/{ids[code]}/put-in-service", {})

        cat = api.call("POST", "/api/warehouse/product-categories", {"code": f"RC-{RUN}", "name": "E2E"})["id"]
        prod = api.call("POST", "/api/warehouse/products", {"categoryId": cat, "sku": f"RC-{RUN}", "name": "E2E"})["id"]
        cid = api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"]
        wf = next(w for w in api.call("GET", "/api/workflow-execution/workflows")["items"] if w["code"] == "INBOUND")["id"]
        tr = api.call("POST", "/api/transportation/transport-requests", {
            "warehouseId": wid, "mapVersionId": MAP_ID, "workflowId": wf, "workflowInputs": {}, "note": "recovery",
            "details": [{"sequenceNo": 1, "containerId": cid, "sourceEndpointId": eps["EP-INBOUND"][0], "destinationEndpointId": eps["EP-SHELF-C-01"][0]}]})["id"]
        job = api.call("POST", f"/api/transportation/transport-requests/{tr}/queue", {})["jobs"][0]["jobId"]
        assert api.call("POST", f"{JOBS}/{job}/start", {"robotId": ids[A]})["status"] == "RUNNING"

        def waiting_step():
            j = api.call("GET", f"{JOBS}/{job}")
            for t in sorted(j["tasks"], key=lambda t: t["sequenceNo"]):
                for s in t["steps"]:
                    if s["status"] == "WAITING" and s["stepType"] == "HUMAN_INTERACTION":
                        return s
            return None

        pickup = wait_for(waiting_step, 120, what="pickup confirmation")
        api.call("POST", f"/api/workflow-execution/job-steps/{pickup['id']}/confirm", {"containerId": cid, "note": "e2e"})
        wait_for(lambda: api.call("GET", f"/api/warehouse/containers/{cid}")["status"] == "IN_TRANSIT", 10, what="on board")

        # A goes silent with the Container on board.
        sims[A].stop()
        wait_for(lambda: api.call("GET", f"{JOBS}/{job}")["status"] == "RECOVERY_REQUIRED", 60, what="recovery opens")
        recovery = api.call("GET", f"{JOBS}/{job}/recovery")["recovery"]
        assert recovery["status"] == "OPEN" and recovery["fromRobotId"] == ids[A] and recovery["containerIds"] == [cid]
        detail = api.call("GET", f"{JOBS}/{job}")
        assert all(c["status"] == "ONBOARD" for c in detail["containers"]), "cargo is not failed"

        # Its zones stay held: it may still be standing in them, and A still holds the Job.
        held = lambda: next((r for r in api.snapshot()["robots"] if r["robotId"] == ids[A]), {}).get("occupiedResourceIds", [])
        assert held(), "a silent robot keeps what it occupies"

        # B comes to a safe endpoint; the transfer is refused until it has arrived and until a person names the cargo.
        api.call("POST", f"{JOBS}/{job}/recovery/assign", {"robotId": ids[B], "endpointId": eps["EP-CHARGE"][0]})

        # What A was granted before it fell silent still blocks B, and rightly: A may be standing in it.
        time.sleep(8)
        assert not api.call("GET", f"{JOBS}/{job}/recovery")["recovery"]["robotArrived"], "traffic holds B back from A's zones"
        assert held()

        # Someone moves A by hand and says so. Allowed while the Job waits for recovery, else this could never finish.
        api.call("POST", f"/api/fleet/traffic/robots/{ids[A]}/release-after-clearance",
                 {"robotCode": A, "clearanceConfirmed": True, "reason": "pushed A aside, checked on site"})
        assert not held()
        wait_for(lambda: api.call("GET", f"{JOBS}/{job}/recovery")["recovery"]["robotArrived"], 180, what="B arrives")
        assert api.call("GET", f"{JOBS}/{job}")["assignedRobotId"] == ids[A], "the Job stays with A until the transfer"
        api.call("POST", f"{JOBS}/{job}/recovery/transfer-confirm", {"containerIds": [cid], "note": "moved by hand"})

        moved = api.call("GET", f"{JOBS}/{job}")
        assert moved["status"] == "PAUSED" and moved["assignedRobotId"] == ids[B]
        api.call("POST", f"{JOBS}/{job}/resume", {})

        dropoff = wait_for(lambda: (lambda s: s if s and s["operationCode"] == "DROPOFF_CONFIRM" else None)(waiting_step()), 180, what="delivery by B")
        api.call("POST", f"/api/workflow-execution/job-steps/{dropoff['id']}/confirm", {"containerId": cid, "note": "e2e"})
        wait_for(lambda: api.call("GET", f"{JOBS}/{job}")["status"] == "COMPLETED", 15, what="same Job completes")
        wait_for(lambda: api.call("GET", f"/api/transportation/transport-requests/{tr}")["status"] == "COMPLETED", 15, what="request completes")
        assert len(api.call("GET", f"/api/transportation/transport-requests/{tr}")["jobs"]) == 1, "no new Job, no new request"
        wait_for(lambda: api.call("GET", f"/api/warehouse/containers/{cid}")["status"] != "IN_TRANSIT", 15, what="delivered")

        # Nothing moved B by itself after it finished (automatic return is off).
        history = api.call("GET", f"/api/fleet/robots/{ids[B]}/state-history?take=200")
        before = sum(1 for i in history["items"] if i["kind"] == "COMMAND" and i["commandType"] == "NAVIGATE_TO_POSE")

        _publish(api, "ROBOT_RETURN_TO_PARK")
        park = api.call("POST", f"/api/workflow-execution/robots/{ids[B]}/return-to-park", {"endpointId": eps["EP-PARKING-B"][0]})
        assert park["transportRequestIds"] == []
        wait_for(lambda: api.call("GET", f"{JOBS}/{park['id']}")["status"] == "COMPLETED", 180, what="back at parking")
        assert math.hypot(sims[B].x - eps["EP-PARKING-B"][1], sims[B].y - eps["EP-PARKING-B"][2]) < 0.3
        after = api.call("GET", f"/api/fleet/robots/{ids[B]}/state-history?take=200")
        assert sum(1 for i in after["items"] if i["kind"] == "COMMAND" and i["commandType"] == "NAVIGATE_TO_POSE") > before
        assert any(i["kind"] == "STATE" for i in api.call("GET", f"/api/fleet/robots/{ids[A]}/state-history?take=200")["items"]), "A going offline is history"

        # WH-06: a point-to-point request from the sample, on B.
        p2p = _publish(api, "POINT_TO_POINT_TRANSPORT")
        c2 = api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"]
        tr2 = api.call("POST", "/api/transportation/transport-requests", {
            "warehouseId": wid, "mapVersionId": MAP_ID, "workflowId": p2p, "workflowInputs": {}, "note": "p2p",
            "details": [{"sequenceNo": 1, "containerId": c2, "sourceEndpointId": eps["EP-INBOUND"][0], "destinationEndpointId": eps["EP-OUTBOUND"][0]}]})["id"]
        job = api.call("POST", f"/api/transportation/transport-requests/{tr2}/queue", {})["jobs"][0]["jobId"]

        # FL-07: goes through the queue, and only because a person enqueued it.
        api.call("POST", f"{JOBS}/{job}/enqueue", {})
        wait_for(lambda: api.call("GET", f"{JOBS}/{job}")["status"] == "RUNNING", 30, what="the queue starts it")
        for operation in ("PICKUP_CONFIRM", "DROPOFF_CONFIRM"):
            s = wait_for(lambda: (lambda x: x if x and x["operationCode"] == operation else None)(waiting_step()), 180, what=operation)
            api.call("POST", f"/api/workflow-execution/job-steps/{s['id']}/confirm", {"containerId": c2, "note": "e2e"})
        wait_for(lambda: api.call("GET", f"{JOBS}/{job}")["status"] == "COMPLETED", 15, what="p2p completes")
        assert not sims[B].moved_while_gate_closed()
    finally:
        for sim in sims.values():
            sim.stop()
        host.stop()
