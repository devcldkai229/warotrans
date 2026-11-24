"""Transport flows on MAP-ONE-V022 (MAP-ONE-V021 + zone partition) with the zone-chain coordinator: every point is a
pickup and a drop-off at least once, two requests at a time on two robots, and the conflicting pairs on purpose
(two INBOUND at once, INBOUND against OUTBOUND in the right column, relocations crossing).

Host = launchSettings "RealRobotZones": Traffic Semantic with SemanticResourceModel=ZoneChain (a plan holds every zone
it runs through, all or nothing; a busy bay is waited for at its queue point). Robots = SimRobot (the robot gateway's
own traffic code from this repo) on A* lane-aware paths at RBT-001's / RBT-002's speeds. Staff confirm every
hand-over. Every scenario is watched at 20 Hz: no resource over capacity, no motion with the gate shut, body gap.

Acceptance (master plan 2026-11-24, section 5): every request COMPLETED, no back-off (deadlock retreat), no timeout,
robots wait at queue points / slots / bays only (never on a corridor).

    WARO_E2E_HOST_DLL=<Host build> python -m pytest tools/traffic_e2e/test_flows_v022.py -s
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import uuid

import pytest

import test_multi_robot_waitpoints_v013 as base
from harness import ROBOT_REPO, Host

SLUG = os.environ.get("WARO_E2E_MAP", "map_one_v022")
BUNDLE = ROBOT_REPO / "ros2_ws" / "src" / "warotrans_navigation" / "maps" / SLUG
MANIFEST = json.loads((BUNDLE / "semantic_manifest.json").read_text(encoding="utf-8"))
TRAFFIC_ZONES = json.loads((BUNDLE / "traffic_zones.json").read_text(encoding="utf-8"))["zones"]
ARTIFACTS = base.HERE / "artifacts" / "flows_v022" / time.strftime("%Y%m%d_%H%M%S")
RESULTS: dict[str, dict] = {}
RELOCATION = "E2E_RELOCATION"  # INTERNAL_RELOCATION without its business rules
OUTBOUND = "E2E_OUTBOUND"  # OUTBOUND_RETRIEVAL without its business rules


def _request(api, warehouse_id: str, moves: list[tuple[str, str]], enqueue: bool = True) -> tuple[str, list[str]]:
    eps = base.endpoints()
    cat = api.call("POST", "/api/warehouse/product-categories", {"code": f"E2E-{base.RUN}-{uuid.uuid4().hex[:4]}", "name": "E2E"})["id"]
    prod = api.call("POST", "/api/warehouse/products", {"categoryId": cat, "sku": f"E2E-{uuid.uuid4().hex[:8]}", "name": "E2E box"})["id"]
    containers = [api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"] for _ in moves]
    if all(s == "EP-INBOUND" for s, _ in moves):
        code = base.WORKFLOW
    elif all(d == "EP-OUTBOUND" for _, d in moves):
        code = OUTBOUND
    else:
        code = RELOCATION
    wf = next(w for w in api.call("GET", "/api/workflow-execution/workflows?pageSize=100")["items"] if w["code"] == code)["id"]
    tr = api.call("POST", "/api/transportation/transport-requests", {
        "warehouseId": warehouse_id, "mapVersionId": base.MAP_ID, "workflowId": wf, "workflowInputs": {"checkQuality": False},
        "note": f"E2E flows v022 {base.RUN}",
        "details": [{"sequenceNo": i + 1, "containerId": c, "sourceEndpointId": eps[s][0], "destinationEndpointId": eps[d][0]}
                    for i, (c, (s, d)) in enumerate(zip(containers, moves))]})["id"]
    jobs = [j["jobId"] for j in api.call("POST", f"/api/transportation/transport-requests/{tr}/queue", {})["jobs"]]
    if enqueue:
        for job in jobs:
            api.call("POST", f"/api/workflow-execution/jobs/{job}/enqueue", {}, ok=(200, 201, 202, 204, 409))
    return tr, jobs


base.request = _request
base.BUNDLE = BUNDLE
base.MANIFEST = MANIFEST
base.ZONES = json.loads((BUNDLE / "single_occupancy.json").read_text(encoding="utf-8"))["zones"]
base.NO_STOP = json.loads((BUNDLE / "no_stop.json").read_text(encoding="utf-8"))["zones"]
base.MAP_ID, base.MAP_CODE, base.FINGERPRINT = MANIFEST["mapVersionId"], MANIFEST["mapCode"], MANIFEST["semanticMapFingerprint"]
base.PATHS = base.MapPaths(BUNDLE, lane_prefer=float(os.environ.get("WARO_E2E_LANE_PREFER", "0.15")))
base.ARTIFACTS = ARTIFACTS
base.WAIT_POINT_CODES = set()
RUN = base.RUN


def _zone_at(x: float, y: float) -> dict | None:
    """The traffic zone owning the point: slot first, then bay, then corridors / yard (smallest first)."""
    order = {"SLOT": 0, "BAY": 1}
    best = None
    for z in TRAFFIC_ZONES:
        if any(base._in_ring(x, y, ring) for ring in z["rings"]):
            key = (order.get(z["kind"], 2),)
            if best is None or key < best[0]:
                best = (key, z)
    return best[1] if best else None


@pytest.fixture(scope="module")
def host():
    ARTIFACTS.mkdir(parents=True, exist_ok=True)
    seed = base.HERE / "seed_test_db.py"
    for args in (["--cleanup-e2e-robots"],
                 ["--map-code", base.MAP_CODE, "--allow-draft", "--workflow-code", "INBOUND", "--workflow-alias", base.WORKFLOW],
                 ["--map-code", base.MAP_CODE, "--allow-draft", "--workflow-code", "INTERNAL_RELOCATION", "--workflow-alias", RELOCATION],
                 ["--map-code", base.MAP_CODE, "--allow-draft", "--workflow-code", "OUTBOUND_RETRIEVAL", "--workflow-alias", OUTBOUND]):
        done = subprocess.run([sys.executable, str(seed), *args], capture_output=True, text=True)
        assert done.returncode == 0, done.stderr
    eps = base.endpoints()
    for wf in base.psql(f"select id, variables_schema::text from execution.workflows where code in ('{base.WORKFLOW}', '{RELOCATION}', '{OUTBOUND}')"):
        variables = json.loads(wf[1])
        for v in variables:
            code = {"inboundEndpointId": "EP-INBOUND", "inspectionEndpointId": "EP-INSPECTION",
                    "outboundEndpointId": "EP-OUTBOUND"}.get(v["key"])
            if code:
                v["configuredValue"] = eps[code][0]
        base.psql(f"update execution.workflows set variables_schema = $v${json.dumps(variables)}$v$::jsonb where id = '{wf[0]}'")
    h = Host(ARTIFACTS, name="host", extra_env={
        # = launchSettings "RealRobotZones", minus hardware.
        "Traffic__SemanticMapDirectory": str(BUNDLE),
        "Traffic__SemanticResourceModel": "ZoneChain",
        "Traffic__SemanticHorizonMeters": "20",
        "Traffic__SemanticZoneMarginMeters": "0.05",
        "Traffic__SemanticLaneHalfWidthMeters": "0.25",
        "Traffic__SemanticPrefixGrant": "false",
        "Traffic__SemanticCellSizeMeters": "0.10",
        "Traffic__SemanticPathToleranceMeters": "0.15",
        "Traffic__StandingMarginMeters": "0.02",
        f"Traffic__RobotHomeEndpoints__E2E-{RUN}-R1": "EP-PARKING-A",
        f"Traffic__RobotHomeEndpoints__E2E-{RUN}-R2": "EP-PARKING-B",
        "FleetIdle__AutoReturnToPark": "true",
        "FleetIdle__IdleSeconds": "8",
        "FleetIdle__FallbackEndpointCodes": "",
        "JobPlanning__MaxContainersPerJob": "1",
    })
    h.start()
    yield h
    for r in base.ROBOTS:
        r.stop()
    h.stop()
    (ARTIFACTS / "summary.json").write_text(json.dumps(RESULTS, indent=1), encoding="utf-8")
    print("\n=== flows " + SLUG + " ===")
    for name, r in RESULTS.items():
        print(f"{name:<26} {json.dumps(r)}")


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


def _free_both_or_explain(api, r1, r2, name: str) -> None:
    try:
        base.free_both(api, r1, r2)
    except AssertionError:
        for s in (r1, r2):
            g = s.client.grant
            print(f"  NOT FREE {s.code[-2:]} at ({s.x:.2f}, {s.y:.2f}, {s.yaw:.2f}) block={s.client.last_block_reason} "
                  f"grant={'rev %s cells %d' % (g.plan_revision, len(g.resource_ids)) if g else 'no'} "
                  f"planRev={s.client.plan_revision} status={api.call('GET', f'/api/fleet/robots/{s.robot_id}')['status']}")
        (ARTIFACTS / f"{name}_not_free_snapshot.json").write_text(json.dumps(api.snapshot(), indent=1), encoding="utf-8")
        raise


def scenario(api, pair, host, name: str, requests: list[list[tuple[str, str]]], timeout: float = 900) -> dict:
    wh, r1, r2 = pair
    _free_both_or_explain(api, r1, r2, name)
    log_before = len(host.log_text())
    jobs: list[str] = []
    for moves in requests:
        _, js = base.request(api, wh, moves)
        jobs += js
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
            grant = s.client.grant
            print(f"  {s.code[-2:]} at ({s.x:.2f}, {s.y:.2f}, {s.yaw:.2f}) block={s.client.last_block_reason} "
                  f"grant={'yes' if grant else 'no'} path={len(s.nav.path or [])}")
    w.stop()
    print("\n" + w.report())
    log = host.log_text()[log_before:]
    # Where the robots stood while they had work: never on a corridor (TRANSIT) or in the yard's open floor.
    waits: dict[str, float] = {}
    eps = base.endpoints()
    for st in w.stops:
        if not st["busy"]:
            continue
        place = st["place"]
        if place.startswith("on the way ("):
            xy = place[len("on the way ("):].split(")")[0].split(",")
            z = _zone_at(float(xy[0]), float(xy[1]))
        else:
            code = place.split(" ")[0]
            z = _zone_at(eps[code][1], eps[code][2]) if code in eps else None
        kind = (z or {}).get("kind", "NONE")
        key = f"{kind}:{(z or {}).get('code', '?')}"
        waits[key] = round(waits.get(key, 0) + st["seconds"], 1)
    result = {
        "ok": error is None and all(s["status"] == "COMPLETED" for s in states.values()),
        "error": error,
        "seconds": round(time.time() - t0, 1),
        "minGapM": round(w.min_gap, 3),
        "bothMovingS": round(w.both_moving_s, 1),
        "waitsByZone": waits,
        "transitWaitS": round(sum(v for k, v in waits.items() if k.startswith(("TRANSIT", "YARD"))), 1),
        "backOffs": log.count("backs off"),
        "queueLegs": log.count("the robot goes to"),
        "waitHere": log.count("the robot waits where it stands"),
        "deadlockRejects": log.count("DEADLOCK_RISK"),
        "capacityBreaches": len(w.capacity_breaches),
    }
    RESULTS[name] = result
    (ARTIFACTS / f"{name}.result.json").write_text(json.dumps(result, indent=1), encoding="utf-8")
    w.assert_safe()
    assert result["ok"], result
    assert result["backOffs"] == 0, result
    return result


def test_1_two_inbound_at_once(api, pair, host):
    """Both pick up at INBOUND: the second waits at Queue B (or Queue A), then goes in when the first has left."""
    scenario(api, pair, host, "1_two_inbound", [[("EP-INBOUND", "EP-SHELF-A-1")], [("EP-INBOUND", "EP-SHELF-C-1")]])


def test_2_inbound_against_outbound(api, pair, host):
    """One up the right column to INBOUND, the other down to OUTBOUND: the corridor is taken in turn."""
    scenario(api, pair, host, "2_in_vs_out", [[("EP-INBOUND", "EP-SHELF-B-01-02")], [("EP-SHELF-C-1", "EP-OUTBOUND")]])


def test_3_outbound_and_relocation(api, pair, host):
    scenario(api, pair, host, "3_out_and_reloc", [[("EP-SHELF-A-2", "EP-OUTBOUND")], [("EP-SHELF-B-01-02", "EP-SHELF-C-1")]])


def test_4_two_relocations_crossing(api, pair, host):
    scenario(api, pair, host, "4_two_relocations", [[("EP-SHELF-C-1", "EP-SHELF-A-2")], [("EP-SHELF-A-1", "EP-SHELF-B-01-02")]])


def test_5_inbound_to_a_and_a_to_outbound(api, pair, host):
    scenario(api, pair, host, "5_in_a_out_a", [[("EP-INBOUND", "EP-SHELF-A-2")], [("EP-SHELF-A-1", "EP-OUTBOUND")]])


def test_6_b_to_outbound_and_inbound_to_c(api, pair, host):
    scenario(api, pair, host, "6_b_out_in_c", [[("EP-SHELF-B-01-02", "EP-OUTBOUND")], [("EP-INBOUND", "EP-SHELF-C-1")]])
