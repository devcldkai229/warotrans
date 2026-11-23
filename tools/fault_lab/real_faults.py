"""Real fault runs on the lab robots (MAP-ONE-V017). One Transport Request per fault, so each case is its own journal.

    python tools/fault_lab/real_faults.py F1 [F2 ...] [--robot RBT-001]

Faults are caused for real through the normal paths (cancel, pause, a wrong scan, nobody confirming, the robot's MQTT
link dropped on the Pi while it STANDS still). No safety function is bypassed; the body-gap watch stops both robots
below 6 cm. The ground truth of each run is written to artifacts/faults_<ts>/<fault>.truth.json, which the AI never
reads (it only sees the Recorder journal).

  F1 cancel before pickup (robot on its way to INBOUND)            -> expected: no case (normal cancel)
  F2 pause 30 s mid-trip, then resume                               -> expected: no case (operator pause)
  F3 wrong container confirmed at pickup, then the right one        -> expected: wrong-scan evidence
  F4 nobody confirms the pickup for 240 s, then the TR is cancelled -> expected: HUMAN_STEP_WAITING_TOO_LONG
  F5 robot's MQTT dropped 75 s while it waits at INBOUND (standing) -> expected: ROBOT_OFFLINE_DURING_TRIP
Pi access (F5) uses WARO_PI_PW from the environment only.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from datetime import UTC, datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "traffic_e2e"))
import real_two_robot_run as rr  # noqa: E402

OUT = HERE.parent / "traffic_e2e" / "artifacts" / f"faults_{datetime.now():%Y%m%d_%H%M%S}"
PI = {"RBT-001": "warotrans.local", "RBT-002": "warotrans-rbt002.local"}
SHELF = {"RBT-001": "SHELF-A-1", "RBT-002": "SHELF-C-1"}


def now() -> str:
    return datetime.now(UTC).isoformat()


def job(w: rr.World, job_id: str) -> dict:
    return w.api.call("GET", f"/api/workflow-execution/jobs/{job_id}")


def step(j: dict, key: str) -> dict | None:
    return next((s for t in j["tasks"] for s in t["steps"] if s.get("stepKey") == key), None)


def wait_for(what: str, fn, timeout: float = 600, every: float = 1.0):  # type: ignore[no-untyped-def]
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = fn()
        if v:
            return v
        time.sleep(every)
    raise TimeoutError(f"timed out waiting for {what}")


def inbound_tr(w: rr.World, robot: str, tag: str) -> tuple[str, str, str]:
    """One INBOUND TR (INBOUND -> robot's shelf), dispatched to `robot`. Returns (tr id, request code, job id)."""
    shelf = SHELF[robot]
    c = rr.new_containers(w, 1, f"{tag}-{robot}")
    rr.voucher(w, "INBOUND", c, f"fault lab {tag} {robot}")
    tr, jobs = rr.create_tr(w, "INBOUND", [{"container": c[0], "src": "EP-INBOUND", "dst": w.loc_ep[shelf], "dstLoc": shelf}],
                            f"fault lab {tag} {robot}")
    rr.dispatch(w, jobs[0], robot)
    code = w.api.call("GET", f"/api/transportation/transport-requests/{tr}").get("requestCode")
    print(f"  {tag}: TR {code} job {jobs[0][:8]} on {robot}")
    return tr, code, jobs[0]


def at_pickup(w: rr.World, job_id: str):  # type: ignore[no-untyped-def]
    s = step(job(w, job_id), "CONFIRM_GOODS_PICKUP")
    return s if s and s["status"] == "WAITING" else None


def finish(w: rr.World, job_id: str, name: str) -> dict:
    """Let the trip run to its end with normal staff confirmations (and the gap watch)."""
    return rr.run_jobs(w, name, [job_id], timeout_s=900)


def cancel_tr(w: rr.World, tr: str, why: str) -> None:
    w.api.call("POST", f"/api/transportation/transport-requests/{tr}/cancel",
               {"reason": why, "idempotencyKey": f"fault-{tr}"})


def pi_run(robot: str, cmd: str, sudo: bool = False) -> str:
    import paramiko

    pw = os.environ["WARO_PI_PW"]
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(PI[robot], username="waro", password=pw, allow_agent=False, look_for_keys=False, timeout=20)
    try:
        i, o, e = c.exec_command(("sudo -S -p '' " if sudo else "") + cmd, timeout=60)
        if sudo:
            i.write(pw + "\n")
            i.flush()
        return o.read().decode(errors="replace") + e.read().decode(errors="replace")
    finally:
        c.close()


# ---------------------------------------------------------------- faults
def f1(w: rr.World, robot: str) -> dict:
    tr, code, j = inbound_tr(w, robot, "F1")
    wait_for("robot driving", lambda: (s := step(job(w, j), "MOVE_TO_INBOUND_AREA")) and s["status"] == "EXECUTING", 120)
    time.sleep(6)
    t = now()
    cancel_tr(w, tr, "fault lab F1: cancelled before pickup")
    wait_for("job cancelled", lambda: job(w, j)["status"] in rr.TERMINAL, 180)
    return {"tr": code, "injected_at": t, "fault": "WFX-03 cancel before pickup", "expected_detectors": [],
            "expected": "no anomaly: a normal cancellation before pickup"}


def f2(w: rr.World, robot: str) -> dict:
    tr, code, j = inbound_tr(w, robot, "F2")
    wait_for("robot driving", lambda: (s := step(job(w, j), "MOVE_TO_INBOUND_AREA")) and s["status"] == "EXECUTING", 120)
    time.sleep(5)
    t = now()
    w.api.call("POST", f"/api/workflow-execution/jobs/{j}/pause", {"reason": "fault lab F2: operator pause"})
    time.sleep(30)
    w.api.call("POST", f"/api/workflow-execution/jobs/{j}/resume", {})
    res = finish(w, j, "F2")
    return {"tr": code, "injected_at": t, "fault": "NEG-03 operator pause 30 s then resume", "expected_detectors": [],
            "expected": "no anomaly: operator pause", "pass": res["pass"]}


def f3(w: rr.World, robot: str) -> dict:
    tr, code, j = inbound_tr(w, robot, "F3")
    s = wait_for("pickup confirm", lambda: at_pickup(w, j), 300)
    wrong = rr.new_containers(w, 1, f"F3-wrong-{robot}")[0]
    t = now()
    refused = []
    for _ in range(2):
        try:
            w.api.call("POST", f"/api/workflow-execution/job-steps/{s['id']}/confirm",
                       {"containerId": wrong, "note": "fault lab F3: staff scanned the wrong box"})
            refused.append("ACCEPTED?!")
        except Exception as exc:  # noqa: BLE001 - the refusal is the point
            refused.append(str(exc)[:160])
        time.sleep(10)
    res = finish(w, j, "F3")
    return {"tr": code, "injected_at": t, "fault": "OPS-01 wrong container scanned at pickup (x2), then the right one",
            "expected_detectors": ["WRONG_CONTAINER_SCAN"], "refusals": refused, "pass": res["pass"]}


def f4(w: rr.World, robot: str) -> dict:
    tr, code, j = inbound_tr(w, robot, "F4")
    wait_for("pickup confirm", lambda: at_pickup(w, j), 300)
    t = now()
    print("  F4: nobody confirms for 240 s")
    time.sleep(240)
    cancel_tr(w, tr, "fault lab F4: pickup never confirmed")
    wait_for("job ended", lambda: job(w, j)["status"] in rr.TERMINAL, 180)
    return {"tr": code, "injected_at": t, "fault": "WFX-04 pickup never confirmed (240 s), then cancelled",
            "expected_detectors": ["HUMAN_STEP_WAITING_TOO_LONG"], "expected_subsystem": "STAFF_HANDOVER"}


def f5(w: rr.World, robot: str) -> dict:
    tr, code, j = inbound_tr(w, robot, "F5")
    wait_for("pickup confirm (robot standing at INBOUND)", lambda: at_pickup(w, j), 300)
    t = now()
    rule = "OUTPUT -p tcp --dport 1883 -j DROP"
    print(f"  F5: dropping MQTT on {robot} for 75 s (robot stands still at INBOUND)")
    pi_run(robot, f"iptables -I {rule}", sudo=True)
    try:
        time.sleep(75)
    finally:
        print(pi_run(robot, f"iptables -D {rule}; iptables -S OUTPUT | grep 1883 || echo restored", sudo=True).strip())
    restored = now()
    wait_for("robot online", lambda: w.robot(robot).get("isOnline"), 180, 3)
    res = finish(w, j, "F5")
    return {"tr": code, "injected_at": t, "restored_at": restored,
            "fault": "IOT-04 robot MQTT link dropped 75 s while standing at INBOUND",
            "expected_detectors": ["ROBOT_OFFLINE_DURING_TRIP"], "expected_subsystem": "MQTT_LINK", "pass": res["pass"]}


FAULTS = {"F1": f1, "F2": f2, "F3": f3, "F4": f4, "F5": f5}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("faults", nargs="+", choices=sorted(FAULTS))
    ap.add_argument("--robot", default="RBT-002", choices=sorted(PI))
    args = ap.parse_args()
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    rr.OUT = OUT
    OUT.mkdir(parents=True, exist_ok=True)
    w = rr.World()
    for f in args.faults:
        print(f"\n===== {f} on {args.robot}")
        try:
            truth = FAULTS[f](w, args.robot)
        except Exception as exc:  # noqa: BLE001 - record and stop: a person looks first
            truth = {"error": f"{type(exc).__name__}: {exc}"}
            (OUT / f"{f}.truth.json").write_text(json.dumps(truth, indent=1), encoding="utf-8")
            print(f"===== {f} FAILED: {truth['error']}")
            return 1
        truth.update(robot=args.robot, finished_at=now())
        (OUT / f"{f}.truth.json").write_text(json.dumps(truth, indent=1), encoding="utf-8")
        print(f"===== {f} done: {json.dumps(truth)[:300]}")
    print("artifacts:", OUT)
    return 0


if __name__ == "__main__":
    sys.exit(main())
