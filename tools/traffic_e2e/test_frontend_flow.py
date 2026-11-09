"""Frontend flow against the real backend, one robot: the exact API sequence of web/src/.../runOnRobot.ts queueAndStart.

Create Transport Request → queue → start Job (no robot id: the backend picks) → robot runs the workflow → staff confirm.
The robot is the motion-free simulator of this harness, but it follows REAL Nav2 paths of MAP-ONE-V004
(real_nav2_paths.json) with the real reservation geometry, zones and lanes. Real robot: not involved, no command to it.
"""

from __future__ import annotations

import json
import math
import subprocess
import sys
import time
from pathlib import Path

from harness import Api, Host, wait_for
from test_traffic_e2e import ARTIFACTS, FINGERPRINT, MAP_CODE, MAP_ID, PREFIX, RUN, ZONES, SimRobot

HERE = Path(__file__).resolve().parent
PATHS = json.loads((HERE / "real_nav2_paths.json").read_text(encoding="utf-8"))["paths"]
ROBOT = f"E2E-{RUN}-FE1"


def _sql(query: str) -> list[list[str]]:
    out = subprocess.run(["docker", "exec", "warotrans-postgres", "psql", "-U", "warotrans", "-d", "warotrans_test",
                          "-t", "-A", "-F", ",", "-c", query], capture_output=True, text=True, check=True).stdout
    return [line.split(",") for line in out.strip().splitlines()]


def _seed(args: list[str]) -> None:
    r = subprocess.run([sys.executable, str(HERE / "seed_test_db.py"), *args], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_transport_request_runs_the_workflow_from_parking_a():
    ARTIFACTS.mkdir(exist_ok=True)
    _seed(["--cleanup-e2e-robots"])
    host = Host(ARTIFACTS, name="frontend", extra_env={f"Traffic__RobotHomeEndpoints__{ROBOT}": "EP-PARKING-A"})
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
            match = next((p for p in PATHS if p["from"] == start and p["to"] == to), None)
            assert match is not None, (start, to)
            used.append((start, to))
            return [tuple(q) for q in match["poses"]]

        used: list[tuple[str, str]] = []
        # Placed somewhere else first: EP-SHELF-C-01, 0.8 m from Parking A.
        sim = SimRobot(code=ROBOT, x=eps["EP-SHELF-C-01"][1], y=eps["EP-SHELF-C-01"][2], fingerprint=FINGERPRINT,
                       zones=ZONES, prefix=PREFIX, map_code=MAP_CODE, path_provider=real_path, speed=0.5)
        rid = api.call("POST", "/api/fleet/robots", {"warehouseId": wid, "currentMapVersionId": MAP_ID, "code": ROBOT,
                                                     "name": ROBOT})["id"]
        sim.start()
        wait_for(lambda: (lambda r: r.get("isOnline") and r.get("localizationStatus") == "LOCALIZED")(
            api.call("GET", f"/api/fleet/robots/{rid}")), 15, what="online and localized")
        api.call("POST", f"/api/fleet/robots/{rid}/put-in-service", {})

        cat = api.call("POST", "/api/warehouse/product-categories", {"code": f"FE-{RUN}", "name": "E2E"})["id"]
        prod = api.call("POST", "/api/warehouse/products", {"categoryId": cat, "sku": f"FE-{RUN}", "name": "E2E"})["id"]
        c1 = api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"]
        c2 = api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"]
        wf = next(w for w in api.call("GET", "/api/workflow-execution/workflows")["items"] if w["code"] == "INBOUND")["id"]

        # What TransportRequestComposer submits.
        tr = api.call("POST", "/api/transportation/transport-requests", {
            "warehouseId": wid, "mapVersionId": MAP_ID, "workflowId": wf, "workflowInputs": {}, "note": "frontend flow",
            "details": [
                {"sequenceNo": 1, "containerId": c1, "sourceEndpointId": src[0], "destinationEndpointId": d1[0]},
                {"sequenceNo": 2, "containerId": c2, "sourceEndpointId": src[0], "destinationEndpointId": d2[0]},
            ]})["id"]
        # queueAndStart: queue, then start without a robot id.
        queued = api.call("POST", f"/api/transportation/transport-requests/{tr}/queue", {})
        job = queued["jobs"][0]["jobId"]
        assert queued["jobs"][0]["jobContainerCount"] == 2

        # Refused: not at Parking A. Nothing was sent to the robot.
        refused = api.call("POST", f"/api/workflow-execution/jobs/{job}/start", {}, ok=(409,))
        assert refused.get("code") == "robot_not_at_home", refused
        assert sim.acks == [] and sim.results == []
        assert api.call("GET", f"/api/workflow-execution/jobs/{job}")["status"] != "RUNNING"

        # Operator places it at Parking A; its pose reports there.
        sim.x, sim.y = park[1], park[2]
        wait_for(lambda: abs(api.call("GET", f"/api/fleet/robots/{rid}")["poseX"] - park[1]) < 0.01, 5, what="pose at Parking A")
        time.sleep(0.5)
        started = api.call("POST", f"/api/workflow-execution/jobs/{job}/start", {})
        assert started["status"] == "RUNNING" and started["assignedRobotId"] == rid, (started["status"], started.get("failureMessage"))

        def waiting():
            j = api.call("GET", f"/api/workflow-execution/jobs/{job}")
            if j["status"] != "RUNNING":
                return j
            for t in sorted(j["tasks"], key=lambda t: t["sequenceNo"]):
                for s in t["steps"]:
                    if s["status"] == "WAITING":
                        return {**s, "_container": t.get("containerId")}
            return None

        confirmed = []
        order = []
        while True:
            s = wait_for(waiting, 120, what="next confirmation")
            if "tasks" in s:
                final = s
                break
            order.append((s["operationCode"], s["_container"], round(sim.x, 2), round(sim.y, 2)))
            # Staff can only confirm once the robot really is at the stop.
            api.call("POST", f"/api/workflow-execution/job-steps/{s['id']}/confirm", {"containerId": s["_container"], "note": "e2e"})
            confirmed.append(s["operationCode"])
            assert len(confirmed) <= 4
        assert final["status"] == "COMPLETED", final.get("failureMessage")
        assert confirmed == ["PICKUP_CONFIRM", "PICKUP_CONFIRM", "DROPOFF_CONFIRM", "DROPOFF_CONFIRM"]
        def at(row, ep):
            return math.hypot(row[2] - ep[1], row[3] - ep[2]) < 0.05  # stopped at that endpoint

        assert at(order[0], src) and at(order[1], src)
        assert order[2][1] == c1 and at(order[2], d1)
        assert order[3][1] == c2 and at(order[3], d2)
        assert used == [("EP-PARKING-A", "EP-INBOUND"), ("EP-INBOUND", "EP-SHELF-A-01"), ("EP-SHELF-A-01", "EP-SHELF-B-01")]
        assert all(c["status"] == "DELIVERED" for c in final["containers"])
        assert all(t["status"] == "COMPLETED" for t in final["tasks"])
        assert api.call("GET", f"/api/transportation/transport-requests/{tr}")["status"] == "COMPLETED"
        assert not sim.moved_while_gate_closed()
        (ARTIFACTS / "frontend_flow.json").write_text(json.dumps(
            {"job": job, "tr": tr, "confirmOrder": order, "routes": used, "results": sim.results,
             "blocked": sorted({r for _, ok, r in sim.gate_log if not ok})}, indent=1), encoding="utf-8")
    finally:
        if sim is not None:
            sim.stop()
        host.stop()
