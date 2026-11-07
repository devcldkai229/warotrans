"""Two-robot, motion-free traffic E2E: real Host (Semantic) + real Mosquitto + robot-gateway logic, isolated.

Run with tools/traffic_e2e/run_traffic_e2e.ps1. Database: warotrans_test only. Topic tree: warotrans-e2e/v1 only
(the live robot RBT-001 listens on warotrans/v1 and never sees any of this). Robot codes: E2E-*.

Scenarios that need open floor use coordinates outside the drawn map, each in its own band, so they never share
cells; the zone scenario uses the real SINGLE_OCCUPANCY polygons of MAP-ONE-V002.
"""

from __future__ import annotations

import json
import math
import time
import uuid
from pathlib import Path

import paho.mqtt.client as paho
import pytest

from harness import BUNDLE, PREFIX, Api, Host, wait_for
from sim_robot import SimRobot

ARTIFACTS = Path(__file__).resolve().parent / "artifacts"
MANIFEST = json.loads((BUNDLE / "semantic_manifest.json").read_text(encoding="utf-8"))
ZONES = json.loads((BUNDLE / "single_occupancy.json").read_text(encoding="utf-8"))["zones"]
FINGERPRINT = MANIFEST["semanticMapFingerprint"]
MAP_ID = MANIFEST["mapVersionId"]
MAP_CODE = MANIFEST["mapCode"]
RUN = uuid.uuid4().hex[:4].upper()
ROBOTS: list[SimRobot] = []


# ---------------------------------------------------------------------- fixtures


@pytest.fixture(scope="session")
def host():
    ARTIFACTS.mkdir(exist_ok=True)
    import subprocess
    import sys

    clean = subprocess.run([sys.executable, str(Path(__file__).with_name("seed_test_db.py")), "--cleanup-e2e-robots"],
                           capture_output=True, text=True)
    assert clean.returncode == 0, clean.stderr
    h = Host(ARTIFACTS)
    h.start()  # migrates warotrans_test and seeds the Development admin
    HOST.append(h)

    seed = subprocess.run([sys.executable, str(Path(__file__).with_name("seed_test_db.py"))],
                          capture_output=True, text=True)
    assert seed.returncode == 0, seed.stderr
    (ARTIFACTS / "seed.json").write_text(seed.stdout, encoding="utf-8")
    yield h
    h.stop()


@pytest.fixture(scope="session")
def api(host):
    a = Api()
    a.login()
    return a


@pytest.fixture(scope="session")
def warehouse_id(api):
    robots = api.call("GET", "/api/fleet/robots?pageSize=1")
    _ = robots
    # The seeded map's warehouse (seed_test_db.py copies it from the dev database).
    import subprocess

    out = subprocess.run(
        ["docker", "exec", "warotrans-postgres", "psql", "-U", "warotrans", "-d", "warotrans_test", "-t", "-A", "-c",
         f"select warehouse_id from navigation.map_versions where id = '{MAP_ID}'"],
        capture_output=True, text=True, check=True)
    wid = out.stdout.strip()
    assert wid, "run seed_test_db.py first"
    return wid


def robot(api, warehouse_id, name: str, x: float, y: float, **kw) -> tuple[SimRobot, str]:
    code = f"E2E-{RUN}-{name}"
    rid = api.call("POST", "/api/fleet/robots", {
        "warehouseId": warehouse_id, "currentMapVersionId": MAP_ID, "code": code, "name": code})["id"]
    sim = SimRobot(code=code, x=x, y=y, fingerprint=FINGERPRINT, zones=ZONES, prefix=PREFIX, map_code=MAP_CODE, **kw)
    sim.start()
    ROBOTS.append(sim)
    wait_for(lambda: api.call("GET", f"/api/fleet/robots/{rid}").get("isOnline"), 15, what=f"{code} online")
    wait_for(lambda: api.call("GET", f"/api/fleet/robots/{rid}").get("localizationStatus") == "LOCALIZED", 15,
             what=f"{code} localized")
    api.call("POST", f"/api/fleet/robots/{rid}/put-in-service", {})
    sim.robot_id = rid  # type: ignore[attr-defined]
    return sim, rid


def navigate(api, rid: str, x: float, y: float, timeout: float = 60.0) -> str:
    body = api.call("POST", f"/api/fleet/robots/{rid}/commands/navigate",
                    {"x": x, "y": y, "yaw": 0.0, "frameId": "map", "timeoutSeconds": timeout})
    return body.get("commandId") or body.get("id")


def view(api, rid: str) -> dict:
    return next((r for r in api.snapshot()["robots"] if r["robotId"] == rid), {})


def holders(snap: dict) -> dict[str, list[str]]:
    return {r["resourceId"]: r["holders"] for r in snap["resources"] if r["holders"]}


def result_of(sim: SimRobot, command_id: str):
    return next((r for r in sim.results if r.get("commandId") == command_id), None)


def watch_exclusive(api, seconds: float, until=None) -> list[dict]:
    """Poll the backend snapshot: no resource ever has more holders than its capacity."""
    snaps = []
    end = time.time() + seconds
    while time.time() < end:
        snap = api.snapshot()
        snaps.append(snap)
        for r in snap["resources"]:
            assert len(r["holders"]) <= r["capacity"], f"{r['resourceId']} over capacity: {r['holders']}"
        if until is not None and until():
            break
        time.sleep(0.25)
    return snaps


HOST: list[Host] = []


def host_log_lines(fragment: str) -> list[str]:
    return [l for l in HOST[0].log_text().splitlines() if fragment in l]


def dist(a: SimRobot, b: SimRobot) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


# ---------------------------------------------------------------------- A


def test_A_single_robot_grant_renew_exit_release(api, warehouse_id):
    a, rid = robot(api, warehouse_id, "A1", 10.0, 0.0)
    cmd = navigate(api, rid, 13.0, 0.0)
    wait_for(lambda: result_of(a, cmd), 40, what="A1 result")
    assert result_of(a, cmd)["outcome"] == "SUCCEEDED"
    extends = [m for m in a.authority_msgs if m["type"] == "EXTEND_AUTHORITY"]
    renews = [m for m in a.authority_msgs if m["type"] == "RENEW_AUTHORITY"]
    assert extends and all(m["targetCommandId"] == cmd for m in extends)
    assert renews, "grant must be renewed while driving"
    assert abs(a.x - 13.0) < 0.05
    v = wait_for(lambda: (lambda r: r if r.get("authority") is None else None)(view(api, rid)), 5, what="authority released")
    # Released what it left (around x=10), kept what it stands on (around x=13).
    held = v["occupiedResourceIds"]
    assert held and all(int(c.split(":")[1]) * 0.25 > 11.5 for c in held if c.startswith("cell:"))
    assert not a.moved_while_gate_closed()


# ---------------------------------------------------------------------- B


def test_B_independent_paths_are_granted_concurrently(api, warehouse_id):
    a, ra = robot(api, warehouse_id, "B1", 10.0, 10.0)
    b, rb = robot(api, warehouse_id, "B2", 10.0, 13.0)
    ca, cb = navigate(api, ra, 13.0, 10.0), navigate(api, rb, 13.0, 13.0)
    both = wait_for(lambda: view(api, ra).get("authority") and view(api, rb).get("authority"), 10, what="both granted")
    assert both
    wait_for(lambda: result_of(a, ca) and result_of(b, cb), 40, what="both done")
    assert result_of(a, ca)["outcome"] == result_of(b, cb)["outcome"] == "SUCCEEDED"
    assert not any(m for m in a.gate_log if m[2] == "OUTSIDE_RESERVATION")


# ---------------------------------------------------------------------- C (+ L promotion)


def test_C_single_occupancy_zone_one_robot_at_a_time_waiter_promoted(api, warehouse_id):
    # Real zone ZONE-SINGLE-CENTER (x 0.25..1.73, y -0.71..0.35) of MAP-ONE-V002.
    a, ra = robot(api, warehouse_id, "C1", 0.0, -0.2)
    b, rb = robot(api, warehouse_id, "C2", 1.0, -1.3)
    ca = navigate(api, ra, 2.2, -0.2)
    wait_for(lambda: view(api, ra).get("authority"), 10, what="C1 granted")
    cb = navigate(api, rb, 1.0, 0.9)
    seen_wait = []

    def done():
        v = view(api, rb)
        if v.get("phase") in ("Waiting", 1):
            seen_wait.append(v.get("blockingOwner"))
        return result_of(a, ca) and result_of(b, cb)

    snaps = watch_exclusive(api, 60, until=done)
    assert result_of(a, ca)["outcome"] == "SUCCEEDED" and result_of(b, cb)["outcome"] == "SUCCEEDED"
    assert ra in seen_wait, "C2 must have waited for C1"
    zone = "zone:ZONE-SINGLE-CENTER"
    assert all(len(holders(s).get(zone, [])) <= 1 for s in snaps)
    assert any(ra in holders(s).get(zone, []) for s in snaps) and any(rb in holders(s).get(zone, []) for s in snaps)
    # Leave the drawn map free for the job scenario (a parked robot rightly blocks every route through it).
    pa, pb = navigate(api, ra, 2.2, -4.0), navigate(api, rb, 3.5, -4.0)
    wait_for(lambda: result_of(a, pa) and result_of(b, pb), 60, what="C robots parked off the map")


# ---------------------------------------------------------------------- D


def test_D_paths_crossing_in_free_space_are_serialized(api, warehouse_id):
    a, ra = robot(api, warehouse_id, "D1", 10.0, 20.0)
    b, rb = robot(api, warehouse_id, "D2", 12.0, 22.0)
    ca = navigate(api, ra, 14.0, 20.0)
    wait_for(lambda: view(api, ra).get("authority"), 10, what="D1 granted")
    cb = navigate(api, rb, 12.0, 18.0)
    min_d = [99.0]

    def done():
        min_d[0] = min(min_d[0], dist(a, b))
        return result_of(a, ca) and result_of(b, cb)

    watch_exclusive(api, 60, until=done)
    assert result_of(a, ca)["outcome"] == "SUCCEEDED" and result_of(b, cb)["outcome"] == "SUCCEEDED"
    assert min_d[0] > 2 * 0.2343, f"robots came within {min_d[0]:.2f} m"
    assert any(not ok for _, ok, _ in b.gate_log), "D2 must have been held at the crossing"


# ---------------------------------------------------------------------- E


def test_E_head_on_narrow_corridor_is_never_authorized_together(api, warehouse_id):
    a, ra = robot(api, warehouse_id, "E1", 10.0, 30.0)
    b, rb = robot(api, warehouse_id, "E2", 13.0, 30.0)
    ca, cb = navigate(api, ra, 13.0, 30.0, timeout=12), navigate(api, rb, 10.0, 30.0, timeout=12)
    min_d = [99.0]

    def done():
        min_d[0] = min(min_d[0], dist(a, b))
        return result_of(a, ca) and result_of(b, cb)

    watch_exclusive(api, 40, until=done)
    assert min_d[0] > 2 * 0.2343
    # They cannot pass each other: at most one gets through, the other times out — never a collision.
    outcomes = sorted([result_of(a, ca)["outcome"], result_of(b, cb)["outcome"]])
    assert "FAILED" in outcomes
    assert not a.moved_while_gate_closed() and not b.moved_while_gate_closed()


# ---------------------------------------------------------------------- F


def test_F_replan_onto_reserved_space_loses_authority_and_never_enters_it(api, warehouse_id):
    a, ra = robot(api, warehouse_id, "F1", 10.0, 90.0, speed=0.25)
    b, rb = robot(api, warehouse_id, "F2", 12.0, 92.0)
    held_b = set(wait_for(lambda: view(api, rb).get("occupiedResourceIds"), 10, what="F2 seeded"))
    cmd = navigate(api, ra, 14.0, 90.0, timeout=90)
    wait_for(lambda: a.x > 10.5, 20, what="F1 moving on its granted path")
    revokes_before = sum(1 for m in a.authority_msgs if m["type"] == "REVOKE_AUTHORITY")
    old_authority = view(api, ra)["authority"]["authorityId"]

    # Nav2 replans through the space F2 stands in.
    a.replan_through((12.0, 92.0))
    rev = a.client.plan_revision
    wait_for(lambda: sum(1 for m in a.authority_msgs if m["type"] == "REVOKE_AUTHORITY") > revokes_before, 5,
             what="old authority revoked")
    wait_for(lambda: not a.gate_log[-1][1], 2, what="gate closed")
    assert view(api, ra).get("authority") is None
    x, y = a.x, a.y
    snaps = watch_exclusive(api, 4)
    assert (a.x, a.y) == (x, y), "moved without authority for the new path"
    assert all(rb in holders(s).get(c, []) for s in snaps for c in held_b), "F2 lost what it stands on"
    assert all(not (set(c for c, h in holders(s).items() if ra in h) & held_b) for s in snaps)
    assert not [m for m in a.authority_msgs if m["type"] == "EXTEND_AUTHORITY" and m["planRevision"] == rev]
    assert dist(a, b) > 2 * 0.2343

    # Path clear again (back to the straight line): new revision, new authority, completes.
    a.replan_through(None)
    v = wait_for(lambda: view(api, ra).get("authority"), 10, what="new grant after replanning away")
    assert v["authorityId"] != old_authority
    wait_for(lambda: result_of(a, cmd), 60, what="F1 done")
    assert result_of(a, cmd)["outcome"] == "SUCCEEDED"
    assert not a.moved_while_gate_closed()


# ---------------------------------------------------------------------- G + H


def test_G_H_offline_robot_keeps_its_space_until_it_is_back(api, warehouse_id):
    a, ra = robot(api, warehouse_id, "G1", 10.0, 40.0)
    b, rb = robot(api, warehouse_id, "G2", 12.5, 42.0)
    ca = navigate(api, ra, 14.0, 40.0)
    wait_for(lambda: a.x > 11.0, 20, what="G1 moving")
    a.disconnect()
    x_at_loss = a.x
    wait_for(lambda: view(api, ra).get("quarantineReason") == "OFFLINE", 15, what="G1 OFFLINE quarantine")
    held = set(view(api, ra)["occupiedResourceIds"])
    assert held
    time.sleep(1.0)
    assert abs(a.x - x_at_loss) < 0.15, "robot without link must stop"
    cb = navigate(api, rb, 12.5, 38.0)
    snaps = watch_exclusive(api, 6)
    assert all(not (set(holders(s)) & held and rb in sum((holders(s)[k] for k in held if k in holders(s)), []))
               for s in snaps), "G2 was granted space the offline G1 holds"
    assert result_of(b, cb) is None
    # The backend aborted G1's NAVIGATE (ROBOT_OFFLINE) and revoked its authority.
    cmd_row = api.call("GET", f"/api/fleet/robots/{ra}")
    assert cmd_row.get("currentCommandId") in (None, "")
    # G1 comes back where it stopped: verified heartbeat + fresh localized pose → no longer OFFLINE, but its aborted
    # command gets no grant again; its authority_state reconcile frees the path it no longer stands on.
    a.reconnect()
    wait_for(lambda: view(api, ra).get("quarantineReason") is None, 15, what="G1 recovered after reconnect")
    x_back = a.x
    wait_for(lambda: result_of(b, cb), 60, what="G2 proceeds once G1's former path is reconciled")
    assert result_of(b, cb)["outcome"] == "SUCCEEDED"
    assert abs(a.x - x_back) < 1e-6, "G1 resumed a NAVIGATE the backend had aborted"
    assert view(api, ra).get("authority") is None


def test_H_robot_moved_while_offline_is_reconciled_only_from_its_verified_pose(api, warehouse_id):
    a, ra = robot(api, warehouse_id, "H1", 10.0, 50.0)
    wait_for(lambda: view(api, ra).get("occupiedResourceIds"), 10, what="H1 seeded from its heartbeat pose")
    a.disconnect()
    wait_for(lambda: view(api, ra).get("quarantineReason") == "OFFLINE", 15, what="H1 OFFLINE")
    old_spot = set(view(api, ra)["occupiedResourceIds"])
    a.x += 3.0  # pushed by hand while unreachable
    a.reconnect()
    # Heartbeat alone proves nothing (RECONNECT_UNVERIFIED); the robot's authority_state with its own localized pose
    # reconciles it: the old spot is released only because the robot is verifiably elsewhere.
    v = wait_for(lambda: (lambda r: r if r.get("quarantineReason") is None and set(r["occupiedResourceIds"]) != old_spot
                          else None)(view(api, ra)), 15, what="H1 reconciled at its new pose")
    assert not old_spot & set(v["occupiedResourceIds"])
    assert abs(v["x"] - 13.0) < 1e-6
    log = host_log_lines("authority_state robot=" + a.code)
    assert any("reconcile=True" in l for l in log)
    wait_for(lambda: api.call("GET", f"/api/fleet/robots/{ra}").get("isOnline"), 15, what="H1 online")
    cmd2 = navigate(api, ra, 15.0, 50.0)
    wait_for(lambda: result_of(a, cmd2), 30, what="H1 drives after reconcile")
    assert result_of(a, cmd2)["outcome"] == "SUCCEEDED"


def test_H_gateway_restart_before_the_operator_pose_recovers_once_localized(api, warehouse_id):
    a, ra = robot(api, warehouse_id, "H2", 10.0, 55.0)
    wait_for(lambda: view(api, ra).get("occupiedResourceIds"), 10, what="H2 seeded")
    # Gateway restarted: not localized yet, reports its state, the backend (holding its spot) quarantines it.
    a.localized = False
    a.publish_authority_state()
    wait_for(lambda: view(api, ra).get("quarantineReason") == "RECONCILE_LOCALIZATION", 10, what="quarantined")
    # Operator sets the pose: the robot reports again and the quarantine lifts by itself.
    a.localized = True
    wait_for(lambda: view(api, ra).get("quarantineReason") is None, 10, what="recovered from the robot's own report")
    assert view(api, ra)["occupiedResourceIds"]


# ---------------------------------------------------------------------- I


def test_I_cancel_stops_at_once_and_timeout_fails_the_command(api, warehouse_id):
    a, ra = robot(api, warehouse_id, "I1", 10.0, 60.0)
    cmd = navigate(api, ra, 16.0, 60.0)
    wait_for(lambda: a.x > 11.0, 20, what="I1 moving")
    api.call("POST", f"/api/fleet/robots/{ra}/commands/cancel", {"targetCommandId": cmd})
    wait_for(lambda: result_of(a, cmd), 10, what="I1 canceled")
    assert result_of(a, cmd)["outcome"] == "CANCELED"
    x = a.x
    time.sleep(1.0)
    assert abs(a.x - x) < 1e-6
    assert a.client.grant is None

    # Timeout: drive into space a parked robot holds → held by traffic → fails at timeoutSeconds, not before.
    b, rb = robot(api, warehouse_id, "I2", 17.5, 60.0)
    wait_for(lambda: view(api, rb).get("occupiedResourceIds"), 10, what="I2 seeded")
    wait_for(lambda: view(api, ra).get("quarantineReason") is None, 10, what="I1 back from CANCELED")
    t0 = time.time()
    cmd2 = navigate(api, ra, 17.0, 60.0, timeout=6)
    wait_for(lambda: result_of(a, cmd2), 30, what="I1 timeout")
    assert result_of(a, cmd2)["errorCode"] == "NAVIGATION_TIMEOUT"
    assert time.time() - t0 >= 5.5
    assert dist(a, b) > 2 * 0.2343


# ---------------------------------------------------------------------- J


def test_J_duplicate_old_and_foreign_messages_change_nothing(api, warehouse_id):
    a, ra = robot(api, warehouse_id, "J1", 10.0, 70.0, speed=0.15)
    cmd = navigate(api, ra, 14.0, 70.0)
    wait_for(lambda: a.client.grant is not None, 10, what="J1 granted")
    extend = next(m for m in reversed(a.authority_msgs) if m["type"] == "EXTEND_AUTHORITY")
    grant_id = a.client.grant.authority_id

    raw = paho.Client(paho.CallbackAPIVersion.VERSION2, client_id=f"e2e-raw-{RUN}")
    raw.connect("127.0.0.1", 1883)
    raw.loop_start()
    topic = f"{PREFIX}/robots/{a.code}/command"
    payload = {k: v for k, v in extend.items() if k != "type"}

    def send(p, kind="EXTEND_AUTHORITY"):
        raw.publish(topic, json.dumps({"schemaVersion": 1, "messageId": str(uuid.uuid4()), "commandId": str(uuid.uuid4()),
                                       "robotCode": a.code, "type": kind, "payload": p}), qos=1).wait_for_publish()

    injected = [str(uuid.uuid4()) for _ in range(5)]
    send(payload)                                                                   # duplicate (QoS1 redelivery)
    send({**payload, "authorityId": injected[0], "planRevision": 0})               # no revision
    send({**payload, "authorityId": injected[1], "epoch": payload["epoch"] - 1})   # old epoch
    send({**payload, "authorityId": injected[2], "targetCommandId": str(uuid.uuid4())})  # other command
    send({**payload, "authorityId": injected[3], "expiresAt": "2020-01-01T00:00:00Z"})   # expired
    send({**payload, "authorityId": injected[4], "planHash": "forged"})            # other plan
    send({"authorityId": str(uuid.uuid4()), "epoch": payload["epoch"]}, "REVOKE_AUTHORITY")  # revoke of another
    seen = set()
    for _ in range(20):
        if a.client.grant is not None:
            seen.add(a.client.grant.authority_id)
        time.sleep(0.05)
    assert a.client.grant is not None, "a revoke naming another authority must not stop the robot"
    assert not seen & set(injected), "an invalid grant was adopted"
    _ = grant_id
    # Delayed, older plan revision straight to the backend: no new grant for it.
    n_ext = sum(1 for m in a.authority_msgs if m["type"] == "EXTEND_AUTHORITY")
    raw.publish(f"{PREFIX}/robots/{a.code}/traffic_plan", json.dumps({
        "schemaVersion": 1, "messageId": str(uuid.uuid4()), "robotCode": a.code, "semanticMapFingerprint": FINGERPRINT,
        "planRevision": 1 if a.client.plan_revision > 1 else 0, "planHash": "old", "commandId": cmd,
        "poses": [{"x": 10, "y": 70, "yaw": 0}, {"x": 10, "y": 72, "yaw": 1.57}]}), qos=1).wait_for_publish()
    time.sleep(1.0)
    assert sum(1 for m in a.authority_msgs if m["type"] == "EXTEND_AUTHORITY"
               and m.get("planHash") == "old") == 0
    assert sum(1 for m in a.authority_msgs if m["type"] == "EXTEND_AUTHORITY") >= n_ext
    raw.loop_stop()
    raw.disconnect()
    wait_for(lambda: result_of(a, cmd), 60, what="J1 done")
    assert result_of(a, cmd)["outcome"] == "SUCCEEDED"


# ---------------------------------------------------------------------- K


def test_K_backend_restart_voids_old_grants_and_reconciles(host, api, warehouse_id):
    a, ra = robot(api, warehouse_id, "K1", 10.0, 80.0, speed=0.2)
    cmd = navigate(api, ra, 14.0, 80.0, timeout=120)
    wait_for(lambda: a.x > 10.5, 20, what="K1 moving")
    old_epoch = a.client.grant.epoch
    host.stop()
    t_stop = time.time()
    wait_for(lambda: not a.gate_log[-1][1], 5, what="gate closed after backend loss")
    assert time.time() - t_stop <= 3.5 + 0.5
    x = a.x
    time.sleep(1.0)
    assert abs(a.x - x) < 1e-6, "no motion without a live backend"
    host.start()
    api.login()
    wait_for(lambda: a.client.known_epoch > old_epoch, 20, what="new epoch seen by the robot")
    wait_for(lambda: result_of(a, cmd), 90, what="K1 completes under the new epoch")
    assert result_of(a, cmd)["outcome"] == "SUCCEEDED"
    new_extends = [m for m in a.authority_msgs if m["type"] == "EXTEND_AUTHORITY" and m["epoch"] > old_epoch]
    assert new_extends


# ---------------------------------------------------------------------- L


def test_L_wait_for_cycle_is_refused_and_nobody_is_authorized(host, api, warehouse_id):
    a, ra = robot(api, warehouse_id, "L1", 10.0, 100.0)
    b, rb = robot(api, warehouse_id, "L2", 13.0, 100.0)
    wait_for(lambda: view(api, ra).get("occupiedResourceIds") and view(api, rb).get("occupiedResourceIds"), 10, what="seeded")
    ca = navigate(api, ra, 13.0, 100.0, timeout=8)
    wait_for(lambda: view(api, ra).get("blockingOwner") == rb, 8, what="L1 waits for L2")
    cb = navigate(api, rb, 10.0, 100.0, timeout=8)
    snaps = watch_exclusive(api, 3)
    for s in snaps:
        for r in s["robots"]:
            if r["robotId"] in (ra, rb):
                assert r["authority"] is None
    assert host_log_lines("reason=DEADLOCK_RISK"), "the second robot must be refused for the cycle"
    assert (a.x, b.x) == (10.0, 13.0)
    wait_for(lambda: result_of(a, ca) and result_of(b, cb), 30, what="both time out")
    assert result_of(a, ca)["errorCode"] == result_of(b, cb)["errorCode"] == "NAVIGATION_TIMEOUT"


def test_L_earlier_waiter_goes_first_and_a_later_robot_cannot_barge_in(api, warehouse_id):
    h, rh = robot(api, warehouse_id, "L3", 23.0, 108.0, speed=0.2)     # crosses the corridor, slowly
    w, rw = robot(api, warehouse_id, "L4", 26.0, 105.0)                # wants the whole corridor
    late, rl = robot(api, warehouse_id, "L5", 25.0, 107.6)             # wants cells of it that are free right now
    ch = navigate(api, rh, 23.0, 102.5, timeout=90)
    wait_for(lambda: view(api, rh).get("authority"), 10, what="L3 granted")
    cw = navigate(api, rw, 21.0, 105.0, timeout=90)
    wait_for(lambda: view(api, rw).get("blockingOwner") == rh, 10, what="L4 queued behind L3")
    cl = navigate(api, rl, 25.0, 105.3, timeout=90)
    v = wait_for(lambda: (lambda r: r if r.get("blockingOwner") == rw else None)(view(api, rl)), 10,
                 what="L5 queued behind the earlier waiter L4, not granted")
    assert v["authority"] is None

    order: list[str] = []

    def done():
        for name, rid in (("L4", rw), ("L5", rl)):
            if name not in order and view(api, rid).get("authority"):
                order.append(name)
        return result_of(h, ch) and result_of(w, cw) and result_of(late, cl)

    watch_exclusive(api, 120, until=done)
    assert order[0] == "L4", f"grant order {order}"
    assert [result_of(h, ch)["outcome"], result_of(w, cw)["outcome"], result_of(late, cl)["outcome"]] == ["SUCCEEDED"] * 3


# ---------------------------------------------------------------------- M


def test_M_two_container_transport_request_runs_to_completion(api, warehouse_id):
    eps = {e["code"]: e for e in api.call("GET", f"/api/navigation/map-versions/{MAP_ID}/endpoints")
           .get("items", [])} if False else None  # endpoints read below from the DB copy
    import subprocess

    out = subprocess.run(
        ["docker", "exec", "warotrans-postgres", "psql", "-U", "warotrans", "-d", "warotrans_test", "-t", "-A", "-F", ",",
         "-c", f"select code, id, x, y from navigation.endpoints where map_version_id = '{MAP_ID}'"],
        capture_output=True, text=True, check=True)
    eps = {c: (i, float(x), float(y)) for c, i, x, y in (l.split(",") for l in out.stdout.strip().splitlines())}
    src, d1, d2 = eps["EP-INBOUND"], eps["EP-SHELF-A-01"], eps["EP-SHELF-B-01"]
    m, rm = robot(api, warehouse_id, "M1", src[1], src[2])

    cat = api.call("POST", "/api/warehouse/product-categories", {"code": f"E2E-{RUN}", "name": "E2E"})["id"]
    prod = api.call("POST", "/api/warehouse/products", {"categoryId": cat, "sku": f"E2E-{RUN}-SKU", "name": "E2E box"})["id"]
    c1 = api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"]
    c2 = api.call("POST", "/api/warehouse/containers", {"productId": prod})["id"]
    wf = next(w for w in api.call("GET", "/api/workflow-execution/workflows")["items"] if w["code"] == "INBOUND")["id"]
    tr = api.call("POST", "/api/transportation/transport-requests", {
        "warehouseId": warehouse_id, "mapVersionId": MAP_ID, "workflowId": wf, "workflowInputs": {}, "note": "E2E harness",
        "details": [
            {"sequenceNo": 1, "containerId": c1, "sourceEndpointId": src[0], "destinationEndpointId": d1[0]},
            {"sequenceNo": 2, "containerId": c2, "sourceEndpointId": src[0], "destinationEndpointId": d2[0]},
        ]})["id"]
    queued = api.call("POST", f"/api/transportation/transport-requests/{tr}/queue", {})
    assert len(queued["jobs"]) == 1
    job = queued["jobs"][0]["jobId"]
    api.call("POST", f"/api/workflow-execution/jobs/{job}/start", {"robotId": rm})

    def current_waiting_step():
        j = api.call("GET", f"/api/workflow-execution/jobs/{job}")
        if j["status"] == "COMPLETED":
            return "done"
        for t in sorted(j["tasks"], key=lambda t: t["sequenceNo"]):
            for s in t["steps"]:
                if s["status"] == "WAITING":
                    return {**s, "_containerId": t.get("containerId")}
        return None

    confirmations = 0
    while True:
        s = wait_for(current_waiting_step, 90, what="next confirmation or completion")
        if s == "done":
            break
        container = s["_containerId"]
        api.call("POST", f"/api/workflow-execution/job-steps/{s['id']}/confirm", {"containerId": container, "note": "e2e"})
        confirmations += 1
        assert confirmations <= 4
    j = api.call("GET", f"/api/workflow-execution/jobs/{job}")
    assert j["status"] == "COMPLETED"
    assert all(c["status"] == "DELIVERED" for c in j["containers"])
    assert all(t["status"] == "COMPLETED" for t in j["tasks"])
    assert confirmations == 4
    assert api.call("GET", f"/api/transportation/transport-requests/{tr}")["status"] == "COMPLETED"
    assert len([r for r in m.results if r["outcome"] == "SUCCEEDED"]) == 3  # pickup, drop A, drop B


# ---------------------------------------------------------------------- N


def test_N_no_robot_ever_moved_without_authority():
    assert ROBOTS
    for r in ROBOTS:
        assert not r.moved_while_gate_closed(), f"{r.code} moved while its traffic decision was 'blocked'"


def teardown_module(_module):
    ARTIFACTS.mkdir(exist_ok=True)
    report = {r.code: {"pose": [r.x, r.y], "results": r.results,
                       "authority": [{k: m.get(k) for k in ("type", "authorityId", "epoch", "seq", "planRevision", "targetCommandId")}
                                     for m in r.authority_msgs],
                       "blockedReasons": sorted({reason for _, ok, reason in r.gate_log if not ok})} for r in ROBOTS}
    (ARTIFACTS / "robots.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    for r in ROBOTS:
        r.stop()
