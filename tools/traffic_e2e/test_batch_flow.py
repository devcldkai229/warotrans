"""WH-08 + WH-01 against the real backend, one simulated robot, with the limit the real config uses (1 Container per Job).

Queue a request for two Containers → two Jobs → start the first → it runs → the second starts by itself from where the
robot stands, and the Containers are IN_TRANSIT while on board. Real robot: not involved, no command to it.
"""

from __future__ import annotations

import math

from harness import Api, Host, wait_for
from test_frontend_flow import PATHS, _seed, _sql
from test_traffic_e2e import ARTIFACTS, FINGERPRINT, MAP_CODE, MAP_ID, PREFIX, RUN, ZONES, SimRobot

ROBOT = f"E2E-{RUN}-BT1"


def test_a_request_split_into_two_jobs_runs_them_in_turn():
    ARTIFACTS.mkdir(exist_ok=True)
    _seed(["--cleanup-e2e-robots"])
    host = Host(ARTIFACTS, name="batches", extra_env={
        f"Traffic__RobotHomeEndpoints__{ROBOT}": "EP-PARKING-A",
        "JobPlanning__MaxContainersPerJob": "1",
    })
    host.start()
    _seed([])
    sim = None
    try:
        api = Api(host.base)
        api.login()
        eps = {c: (i, float(x), float(y)) for c, i, x, y in
               _sql(f"select code, id, x, y from navigation.endpoints where map_version_id = '{MAP_ID}'")}
        wid = _sql(f"select warehouse_id from navigation.map_versions where id = '{MAP_ID}'")[0][0]
        src, d1, d2, park = eps["EP-INBOUND"], eps["EP-SHELF-A-01"], eps["EP-SHELF-B-01"], eps["EP-PARKING-A"]
        code_at = {(round(v[1], 3), round(v[2], 3)): c for c, v in eps.items()}

        def real_path(x, y, goal):
            to = code_at[(round(goal.x, 3), round(goal.y, 3))]
            start = min(eps, key=lambda c: math.hypot(eps[c][1] - x, eps[c][2] - y))
            match = next(p for p in PATHS if p["from"] == start and p["to"] == to)
            used.append((start, to))
            return [tuple(q) for q in match["poses"]]

        used: list[tuple[str, str]] = []
        sim = SimRobot(code=ROBOT, x=park[1], y=park[2], fingerprint=FINGERPRINT, zones=ZONES, prefix=PREFIX,
                       map_code=MAP_CODE, path_provider=real_path, speed=0.5)
        rid = api.call("POST", "/api/fleet/robots", {"warehouseId": wid, "currentMapVersionId": MAP_ID, "code": ROBOT,
                                                     "name": ROBOT})["id"]
        sim.start()
        wait_for(lambda: (lambda r: r.get("isOnline") and r.get("localizationStatus") == "LOCALIZED")(
            api.call("GET", f"/api/fleet/robots/{rid}")), 15, what="online and localized")
        api.call("POST", f"/api/fleet/robots/{rid}/put-in-service", {})

        cat = api.call("POST", "/api/warehouse/product-categories", {"code": f"BT-{RUN}", "name": "E2E"})["id"]
        prod = api.call("POST", "/api/warehouse/products", {"categoryId": cat, "sku": f"BT-{RUN}", "name": "E2E"})["id"]
        c1 = api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"]
        c2 = api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"]
        wf = next(w for w in api.call("GET", "/api/workflow-execution/workflows")["items"] if w["code"] == "INBOUND")["id"]
        tr = api.call("POST", "/api/transportation/transport-requests", {
            "warehouseId": wid, "mapVersionId": MAP_ID, "workflowId": wf, "workflowInputs": {}, "note": "batches",
            "details": [
                {"sequenceNo": 1, "containerId": c1, "sourceEndpointId": src[0], "destinationEndpointId": d1[0]},
                {"sequenceNo": 2, "containerId": c2, "sourceEndpointId": src[0], "destinationEndpointId": d2[0]},
            ]})["id"]

        queued = api.call("POST", f"/api/transportation/transport-requests/{tr}/queue", {})
        assert [j["batchKey"] for j in queued["jobs"]] == ["BATCH-001", "BATCH-002"]
        assert [j["jobContainerCount"] for j in queued["jobs"]] == [1, 1]
        first, second = (j["jobId"] for j in queued["jobs"])

        # Nothing moves by itself before someone starts the request.
        assert api.call("GET", f"/api/workflow-execution/jobs/{second}")["status"] == "CREATED"
        assert api.call("POST", f"/api/workflow-execution/jobs/{first}/start", {})["status"] == "RUNNING"

        def waiting(job):
            j = api.call("GET", f"/api/workflow-execution/jobs/{job}")
            if j["status"] != "RUNNING":
                return j
            for t in sorted(j["tasks"], key=lambda t: t["sequenceNo"]):
                for s in t["steps"]:
                    if s["status"] == "WAITING":
                        return {**s, "_container": t.get("containerId")}
            return None

        def container(cid):
            return api.call("GET", f"/api/warehouse/containers/{cid}")

        def run(job, cid):
            confirmed = []
            while True:
                s = wait_for(lambda: waiting(job), 120, what=f"next confirmation of {job}")
                if "tasks" in s:
                    assert s["status"] == "COMPLETED", s.get("failureMessage")
                    return confirmed
                assert s["_container"] == cid
                api.call("POST", f"/api/workflow-execution/job-steps/{s['id']}/confirm",
                         {"containerId": cid, "note": "e2e"})
                confirmed.append(s["operationCode"])
                if s["operationCode"] == "PICKUP_CONFIRM":
                    wait_for(lambda: container(cid)["status"] == "IN_TRANSIT", 10, what="on board means IN_TRANSIT")

        assert run(first, c1) == ["PICKUP_CONFIRM", "DROPOFF_CONFIRM"]
        wait_for(lambda: container(c1)["status"] != "IN_TRANSIT", 10, what="delivered leaves IN_TRANSIT")
        assert api.call("GET", f"/api/workflow-execution/jobs/{second}")["status"] in ("CREATED", "RUNNING")

        # The continuation service starts the second Job, from where the robot stands now.
        wait_for(lambda: api.call("GET", f"/api/workflow-execution/jobs/{second}")["status"] == "RUNNING", 30,
                 what="the second Job starts by itself")
        assert run(second, c2) == ["PICKUP_CONFIRM", "DROPOFF_CONFIRM"]
        wait_for(lambda: container(c2)["status"] != "IN_TRANSIT", 10, what="delivered leaves IN_TRANSIT")

        request = api.call("GET", f"/api/transportation/transport-requests/{tr}")
        assert request["status"] == "COMPLETED"
        assert len(request["jobs"]) == 2
        assert not sim.moved_while_gate_closed()
        # Parking → pickup → drop, then from that drop → pickup → drop.
        assert used == [("EP-PARKING-A", "EP-INBOUND"), ("EP-INBOUND", "EP-SHELF-A-01"),
                        ("EP-SHELF-A-01", "EP-INBOUND"), ("EP-INBOUND", "EP-SHELF-B-01")]
    finally:
        if sim is not None:
            sim.stop()
        host.stop()
