"""Two real Hosts on one database: one coordinator at a time, failover, stale-epoch rejection.

Run alone (it starts its own Hosts): python -m pytest test_two_backends.py -v -p no:cacheprovider
Database warotrans_test, topic tree warotrans-e2e/v1, robot codes E2E-*. No real robot involved.
"""

from __future__ import annotations

import json
import subprocess
import sys
import time
import uuid
from pathlib import Path

import paho.mqtt.client as paho

from harness import PREFIX, Api, Host, wait_for
from test_traffic_e2e import ARTIFACTS, FINGERPRINT, MAP_ID, RUN, navigate, result_of, robot


def _seed(args: list[str]) -> None:
    r = subprocess.run([sys.executable, str(Path(__file__).with_name("seed_test_db.py")), *args], capture_output=True, text=True)
    assert r.returncode == 0, r.stderr


def test_one_coordinator_failover_and_stale_epoch():
    ARTIFACTS.mkdir(exist_ok=True)
    _seed(["--cleanup-e2e-robots"])
    first = Host(ARTIFACTS, port=5199, name="leader")
    second = Host(ARTIFACTS, port=5198, name="standby")
    first.start()
    _seed([])
    second.start(wait_for_lease=False)
    sim = None
    try:
        api = Api(first.base)
        api.login()
        wid = subprocess.run(
            ["docker", "exec", "warotrans-postgres", "psql", "-U", "warotrans", "-d", "warotrans_test", "-t", "-A", "-c",
             f"select warehouse_id from navigation.map_versions where id = '{MAP_ID}'"],
            capture_output=True, text=True, check=True).stdout.strip()

        # Exactly one holds the lease.
        wait_for(lambda: "held by another host" in second.log_text(), 15, what="standby refused the lease")
        assert "grants open" in first.log_text() and "grants open" not in second.log_text()

        sim, rid = robot(api, wid, "HA1", 10.0, 120.0, speed=0.2)
        cmd = navigate(api, rid, 14.0, 120.0, timeout=180)
        wait_for(lambda: sim.x > 10.4, 20, what="moving under the leader")
        old_epoch = sim.client.grant.epoch
        old_extend = {k: v for k, v in next(m for m in reversed(sim.authority_msgs)
                                            if m["type"] == "EXTEND_AUTHORITY").items() if k != "type"}
        # The standby acts on nothing: no plan handled, no monitor verdict.
        assert "traffic_plan robot=" not in second.log_text()
        assert "connectivity OFFLINE" not in second.log_text()

        # Leader dies. Robot stops on backend silence; standby takes over with a higher epoch.
        first.stop()
        wait_for(lambda: not sim.gate_log[-1][1], 5, what="gate closed after leader loss")
        wait_for(lambda: "grants open" in second.log_text(), 15, what="standby took the lease")
        wait_for(lambda: sim.client.known_epoch > old_epoch, 10, what="robot saw the new epoch")
        assert sim.client.grant is None or sim.client.grant.epoch > old_epoch

        # A grant of the old coordinator, delivered late, is refused.
        raw = paho.Client(paho.CallbackAPIVersion.VERSION2, client_id=f"e2e-raw-ha-{RUN}")
        raw.connect("127.0.0.1", 1883)
        raw.loop_start()
        forged = str(uuid.uuid4())
        raw.publish(f"{PREFIX}/robots/{sim.code}/command", json.dumps({
            "schemaVersion": 1, "messageId": str(uuid.uuid4()), "commandId": str(uuid.uuid4()), "robotCode": sim.code,
            "type": "EXTEND_AUTHORITY", "payload": {**old_extend, "authorityId": forged, "planRevision": sim.client.plan_revision,
                                                    "planHash": sim.client.plan_hash}}), qos=1).wait_for_publish()
        time.sleep(0.5)
        raw.loop_stop()
        raw.disconnect()
        assert sim.client.grant is None or sim.client.grant.authority_id != forged

        # New leader: the robot reconciles and completes the same NAVIGATE under the new epoch.
        wait_for(lambda: result_of(sim, cmd), 120, what="completes under the new leader")
        assert result_of(sim, cmd)["outcome"] == "SUCCEEDED"
        assert [m for m in sim.authority_msgs if m["type"] == "EXTEND_AUTHORITY" and m["epoch"] > old_epoch]
        assert FINGERPRINT

        # The old leader comes back: it is now the standby.
        first.start(wait_for_lease=False)
        wait_for(lambda: "held by another host" in first.log_text(), 15, what="restarted host stays standby")
        assert "grants open" not in first.log_text()
    finally:
        if sim is not None:
            sim.stop()
        first.stop()
        second.stop()
