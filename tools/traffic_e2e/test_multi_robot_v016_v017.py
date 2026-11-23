"""Two simulated robots on MAP-ONE-V016 or MAP-ONE-V017: which map lets both run in parallel without touching.

V017 = MAP-ONE-V015 as drawn (ZONE-AREA-INBOUND around Queue B, ZONE-SINGLE-CENTER over the crossroads).
V016 = V015 without those two single-occupancy zones, Queue A typed QUEUE, plus a one-way exit lane at x = 2.30 in the
INBOUND dead end (so a robot leaving INBOUND passes one waiting at Queue B).

Same Host settings, same scenarios, same robots for both maps; pick the map with WARO_E2E_MAP=map_one_v016|map_one_v017.
Every scenario records (artifacts/v016_v017/<map>/<scenario>.json + summary.json):
  * safety: smallest body gap, capacity breaches, motion without authority (asserted);
  * parallelism: seconds both robots moved, seconds both had work, wall time;
  * where a robot with work stood still >= 2 s: its goal, Queue B / Queue A, parking, another endpoint, on the way;
  * deadlock back-offs (Host log "backs off").

Run on its own (owns port 5199 and warotrans_test):
    WARO_E2E_MAP=map_one_v016 WARO_E2E_HOST_DLL=<host build> python -m pytest tools/traffic_e2e/test_multi_robot_v016_v017.py -s
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

import pytest

import test_multi_robot_waitpoints_v013 as base
from harness import ROBOT_REPO, Host

SLUG = os.environ.get("WARO_E2E_MAP", "map_one_v016")
BUNDLE = ROBOT_REPO / "ros2_ws" / "src" / "warotrans_navigation" / "maps" / SLUG
MANIFEST = json.loads((BUNDLE / "semantic_manifest.json").read_text(encoding="utf-8"))
ARTIFACTS = base.HERE / "artifacts" / "v016_v017" / SLUG
RESULTS: dict[str, dict] = {}
RELOCATION = "E2E_RELOCATION"  # INTERNAL_RELOCATION: any pickup (INBOUND always picks up at EP-INBOUND)


def _request(api, warehouse_id: str, moves: list[tuple[str, str]], enqueue: bool = True) -> tuple[str, list[str]]:
    """As the V013 harness, plus the staff answer INBOUND asks for (no quality check: straight to the shelf)."""
    eps = base.endpoints()
    cat = api.call("POST", "/api/warehouse/product-categories", {"code": f"E2E-{base.RUN}-{uuid.uuid4().hex[:4]}", "name": "E2E"})["id"]
    prod = api.call("POST", "/api/warehouse/products", {"categoryId": cat, "sku": f"E2E-{uuid.uuid4().hex[:8]}", "name": "E2E box"})["id"]
    containers = [api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"] for _ in moves]
    code = base.WORKFLOW if all(s == "EP-INBOUND" for s, _ in moves) else RELOCATION
    wf = next(w for w in api.call("GET", "/api/workflow-execution/workflows?pageSize=100")["items"] if w["code"] == code)["id"]
    tr = api.call("POST", "/api/transportation/transport-requests", {
        "warehouseId": warehouse_id, "mapVersionId": base.MAP_ID, "workflowId": wf, "workflowInputs": {"checkQuality": False},
        "note": f"E2E v016/v017 {base.RUN}",
        "details": [{"sequenceNo": i + 1, "containerId": c, "sourceEndpointId": eps[s][0], "destinationEndpointId": eps[d][0]}
                    for i, (c, (s, d)) in enumerate(zip(containers, moves))]})["id"]
    jobs = [j["jobId"] for j in api.call("POST", f"/api/transportation/transport-requests/{tr}/queue", {})["jobs"]]
    if enqueue:
        for job in jobs:
            api.call("POST", f"/api/workflow-execution/jobs/{job}/enqueue", {}, ok=(200, 201, 202, 204, 409))
    return tr, jobs


base.request = _request

# Point the shared helpers of the V013 harness at this map.
base.BUNDLE = BUNDLE
base.MANIFEST = MANIFEST
base.ZONES = json.loads((BUNDLE / "single_occupancy.json").read_text(encoding="utf-8"))["zones"]
base.NO_STOP = json.loads((BUNDLE / "no_stop.json").read_text(encoding="utf-8"))["zones"]
base.MAP_ID, base.MAP_CODE, base.FINGERPRINT = MANIFEST["mapVersionId"], MANIFEST["mapCode"], MANIFEST["semanticMapFingerprint"]
base.PATHS = base.MapPaths(BUNDLE, lane_prefer=float(os.environ.get("WARO_E2E_LANE_PREFER", "0.15")))  # like LaneAwareNavfn
base.ARTIFACTS = ARTIFACTS
base.WAIT_POINT_CODES = {"EP-QUEUE-A", "EP-QUEUE-B"}
RUN = base.RUN


@pytest.fixture(scope="module")
def host():
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    seed = base.HERE / "seed_test_db.py"
    for args in (["--cleanup-e2e-robots"],
                 ["--map-code", base.MAP_CODE, "--allow-draft", "--workflow-code", "INBOUND", "--workflow-alias", base.WORKFLOW],
                 ["--map-code", base.MAP_CODE, "--allow-draft", "--workflow-code", "INTERNAL_RELOCATION",
                  "--workflow-alias", RELOCATION]):
        done = subprocess.run([sys.executable, str(seed), *args], capture_output=True, text=True)
        assert done.returncode == 0, done.stderr
    # The INBOUND workflow's Admin-configured gate / inspection endpoints belong to the map the source DB was set up
    # with; in the test copy point them at the same codes on the map under test.
    eps = base.endpoints()
    for wf in base.psql(f"select id, variables_schema::text from execution.workflows where code in ('{base.WORKFLOW}', '{RELOCATION}')"):
        variables = json.loads(wf[1])
        for v in variables:
            code = {"inboundEndpointId": "EP-INBOUND", "inspectionEndpointId": "EP-INSPECTION"}.get(v["key"])
            if code:
                v["configuredValue"] = eps[code][0]
        base.psql(f"update execution.workflows set variables_schema = $v${json.dumps(variables)}$v$::jsonb where id = '{wf[0]}'")
    h = Host(ARTIFACTS, name="host", extra_env={
        # = launchSettings "RealRobot", minus hardware.
        "Traffic__SemanticMapDirectory": str(BUNDLE),
        "Traffic__SemanticHorizonMeters": "2.0",
        "Traffic__SemanticZoneMarginMeters": "0.05",
        "Traffic__SemanticLaneHalfWidthMeters": os.environ.get("WARO_E2E_LANE_HALF", "0.25"),
        "Traffic__SemanticPrefixGrant": "true",
        "Traffic__SemanticCellSizeMeters": "0.10",
        "Traffic__SemanticPathToleranceMeters": "0.15",
        f"Traffic__RobotHomeEndpoints__E2E-{RUN}-R1": "EP-PARKING-A",
        f"Traffic__RobotHomeEndpoints__E2E-{RUN}-R2": "EP-PARKING-B",
        "FleetIdle__AutoReturnToPark": "false",
        "FleetIdle__IdleSeconds": "3",
        "FleetIdle__FallbackEndpointCodes": "EP-QUEUE-A",
        "FleetIdle__GoalCorridors": "EP-INBOUND=1.10,-1.15,2.65,1.60",
        "JobPlanning__MaxContainersPerJob": "3",
    })
    h.start()
    base.HOST_REF = h  # type: ignore[attr-defined]
    yield h
    for r in base.ROBOTS:
        r.stop()
    h.stop()
    (ARTIFACTS / "summary.json").write_text(json.dumps(RESULTS, indent=1), encoding="utf-8")
    print("\n=== " + SLUG + " ===")
    for name, r in RESULTS.items():
        print(f"{name:<22} {json.dumps(r)}")


@pytest.fixture(scope="module")
def api(host):
    a = base.Api()
    a.login()
    return a


@pytest.fixture(scope="module")
def pair(api):
    wh = base.warehouse()
    eps = base.endpoints()
    return wh, base.robot(api, wh, "R1", eps["EP-PARKING-A"]), base.robot(api, wh, "R2", eps["EP-PARKING-B"])


def _kind(place: str, goals: set[str]) -> str:
    code = place.split(" ")[0]
    if place.startswith("on the way"):
        return "way"
    if code in goals:
        return "goal"
    if code == "EP-QUEUE-B":
        return "queueB"
    if code == "EP-QUEUE-A":
        return "queueA"
    if "PARKING" in code:
        return "parking"
    return "otherEndpoint"


def scenario(api, pair, host, name: str, requests: list[list[tuple[str, str]]], timeout: float = 900) -> dict:
    wh, r1, r2 = pair
    base.free_both(api, r1, r2)
    log_before = len(host.log_text())
    jobs: list[str] = []
    goals: set[str] = set()
    for moves in requests:
        _, js = base.request(api, wh, moves)
        jobs += js
        for s, d in moves:
            goals |= {s, d}
    t0 = time.time()
    w = base.Watch(api, [r1, r2], name).start()
    error = None
    try:
        states = w.run_until(base.terminal, timeout, jobs)
    except AssertionError as e:
        error, states = str(e)[:300], {}
        try:
            (ARTIFACTS / f"{name}_fail_snapshot.json").write_text(json.dumps(api.snapshot(), indent=1), encoding="utf-8")
        except Exception:  # noqa: BLE001 - diagnostics only
            pass
        for s in (r1, r2):
            last = [g for g in s.gate_log[-400:] if not g[1]]
            grant = s.client.grant
            print(f"  {s.code[-2:]} at ({s.x:.2f}, {s.y:.2f}, {s.yaw:.2f}) gate closed {len(last)}/400 last={last[-1][2] if last else None} "
                  f"grant={'yes' if grant else 'no'} cells={len(grant.resource_ids) if grant else 0} path={len(s.nav.path or [])}")
    w.stop()
    print("\n" + w.report())
    log = host.log_text()[log_before:]
    stops: dict[str, float] = {}
    for st in w.stops:
        if st["busy"]:
            k = _kind(st["place"], goals)
            stops[k] = round(stops.get(k, 0) + st["seconds"], 1)
    result = {
        "ok": error is None and all(s["status"] == "COMPLETED" for s in states.values()),
        "error": error,
        "seconds": round(time.time() - t0, 1),
        "minGapM": round(w.min_gap, 3),
        "bothMovingS": round(w.both_moving_s, 1),
        "bothBusyS": round(w.both_busy_s, 1),
        "movedM": {k[-2:]: round(v, 2) for k, v in w.moved.items()},
        "busyStopSeconds": stops,
        "backOffs": log.count("backs off"),
        "goalWaits": [l.split("robot waits ", 1)[1].strip() for l in log.splitlines() if "is taken" in l and "robot waits " in l],
        "backOffTo": [l.rsplit("backs off to ", 1)[1].strip() for l in log.splitlines() if "backs off to " in l],
        "capacityBreaches": len(w.capacity_breaches),
    }
    RESULTS[name] = result
    (ARTIFACTS / f"{name}.result.json").write_text(json.dumps(result, indent=1), encoding="utf-8")
    w.assert_safe()
    assert result["ok"], result
    return result


def test_1_two_inbound_requests_at_once(api, pair, host):
    """Both pick up at INBOUND: the second waits (where it is, or at Queue B), the first leaves past it."""
    scenario(api, pair, host, "1_two_inbound", [[("EP-INBOUND", "EP-SHELF-C-1")], [("EP-INBOUND", "EP-SHELF-B-01-02")]])


def test_2_inbound_and_outbound_at_once(api, pair, host):
    """One goes up to INBOUND, the other down to OUTBOUND: lanes LTR / RTL side by side."""
    scenario(api, pair, host, "2_in_and_out", [[("EP-SHELF-C-1", "EP-INBOUND")], [("EP-SHELF-B-01-02", "EP-OUTBOUND")]])


def test_3_cross_relocation_shelf_b_and_c(api, pair, host):
    """B -> C and C -> B at once: lane B down and lane B up side by side."""
    scenario(api, pair, host, "3_cross_b_c", [[("EP-SHELF-B-01-02", "EP-SHELF-C-1")], [("EP-SHELF-C-1", "EP-SHELF-B-01-02")]])


def test_4_shelf_a_and_inbound(api, pair, host):
    scenario(api, pair, host, "4_shelf_a_and_inbound", [[("EP-SHELF-A-1", "EP-SHELF-A-2")], [("EP-INBOUND", "EP-SHELF-C-1")]])


def test_5_head_on_inbound_outbound(api, pair, host):
    scenario(api, pair, host, "5_head_on", [[("EP-INBOUND", "EP-OUTBOUND")], [("EP-OUTBOUND", "EP-INBOUND")]])


def test_6_three_requests_two_into_inbound(api, pair, host):
    scenario(api, pair, host, "6_three_requests", [
        [("EP-SHELF-C-1", "EP-INBOUND")], [("EP-SHELF-B-01-02", "EP-INBOUND")], [("EP-INBOUND", "EP-SHELF-A-1")]], timeout=1200)
