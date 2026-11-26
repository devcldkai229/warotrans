"""Cancel / pause / resume of transport requests on MAP-ONE-V024, real Host + two SimRobots (RealRobot traffic settings).

One request on one robot, cancelled in each state it can be in:
  K1  before any robot took it              K2  robot driving to the pickup (nothing on board)
  K3  robot waiting at the pickup for staff  K4  cargo on board: the robot holds, staff choose "deliver anyway"
One request on two robots (two Jobs):
  K5  both driving, nothing on board         K6  one carries cargo, the other not: the empty one ends at once
Pause / resume:
  P1  pause while driving, resume            P2  pause while waiting for staff, confirm refused, resume
  P3  pause, then cancel the request         P4  pause one Job of two: the other one finishes, then resume
  P5  emergency stop from the console, then resume the Job

After every scenario: no Job left RUNNING or PAUSED, both robots free, not motion-held, back on a parking place.

    WARO_E2E_HOST_DLL=<Host build> python -m pytest tools/traffic_e2e/test_cancel_pause_v024.py -s
"""

from __future__ import annotations

import json
import time

import test_flows_v024 as flows
from test_flows_v024 import api, host, pair  # noqa: F401 - pytest fixtures

base = flows.base
RESULTS: dict[str, dict] = {}
TR = "/api/transportation/transport-requests"
JOBS = "/api/workflow-execution/jobs"


# ---------------------------------------------------------------------- helpers


def _job(api, job_id: str) -> dict:
    return api.call("GET", f"{JOBS}/{job_id}")


def _moving_step(job: dict) -> bool:
    return any(st.get("stepType") == "MOVE" and st.get("status") == "EXECUTING" for t in job["tasks"] for st in t["steps"])


def _onboard(job: dict) -> bool:
    return any(c["status"] == "ONBOARD" for c in job.get("containers", []))


def _robot(api, robot_id: str) -> dict:
    return api.call("GET", f"/api/fleet/robots/{robot_id}")


def _settled(api, pair, timeout: float = 180) -> dict:
    """Both robots free, not held, parked (the idle return runs by itself). Returns where they stand."""
    _, r1, r2 = pair
    end = time.time() + timeout
    eps = base.endpoints()
    parks = {code: (x, y) for code, (_, x, y, _yaw) in eps.items() if "PARKING" in code}
    last = {}
    while time.time() < end:
        last = {}
        for s in (r1, r2):
            robot = _robot(api, s.robot_id)
            near = min(parks, key=lambda c: (parks[c][0] - s.x) ** 2 + (parks[c][1] - s.y) ** 2)
            dist = ((parks[near][0] - s.x) ** 2 + (parks[near][1] - s.y) ** 2) ** 0.5
            last[s.code[-2:]] = {"free": base.robot_free(s.robot_id), "held": robot.get("motionHeld"), "status": robot["status"],
                                 "park": near if dist < 0.25 else None}
        if all(v["free"] and not v["held"] and v["park"] for v in last.values()):
            return last
        time.sleep(1.0)
    raise AssertionError(f"robots did not settle: {last}")


def _watch(api, pair, name: str, staff: bool = True):
    _, r1, r2 = pair
    w = base.Watch(api, [r1, r2], name).start()
    if not staff:
        w.staff = lambda jobs: None  # nobody on the floor confirms anything
    return w


def _request(api, pair, moves, enqueue: bool = True):
    wh, _, _ = pair
    return base.request(api, wh, moves, enqueue=enqueue)


def _cancellation(api, tr: str) -> dict:
    return api.call("GET", f"{TR}/{tr}/cancellation")


def _record(name: str, **values) -> None:
    RESULTS[name] = values
    print(f"[{name}] {json.dumps(values)}")
    (flows.ARTIFACTS / "cancel_pause_summary.json").write_text(json.dumps(RESULTS, indent=1), encoding="utf-8")


def _start(api, pair, name):
    flows._free_both_or_explain(api, pair[1], pair[2], name)


# ---------------------------------------------------------------------- one request, one robot


def test_K1_cancel_before_a_robot_took_it(api, pair):
    _start(api, pair, "K1")
    tr, jobs = _request(api, pair, [("EP-INBOUND", "EP-SHELF-A-1")], enqueue=False)
    t0 = time.time()
    api.call("POST", f"{TR}/{tr}/cancel", {"reason": "e2e K1: entered by mistake", "reasonCode": "WRONG_REQUEST"})
    request = api.call("GET", f"{TR}/{tr}")
    assert request["status"] == "CANCELLED", request["status"]
    assert all(_job(api, j)["status"] == "CANCELLED" for j in jobs)
    assert _cancellation(api, tr)["status"] == "FINALIZED"
    _record("K1", seconds=round(time.time() - t0, 1), request=request["status"], robots=_settled(api, pair))


def test_K2_cancel_while_driving_to_the_pickup(api, pair):
    _start(api, pair, "K2")
    tr, jobs = _request(api, pair, [("EP-SHELF-C", "EP-OUTBOUND")])
    w = _watch(api, pair, "K2")
    sent = {}

    def cancel_when_moving(states):
        job = states[jobs[0]]
        if not sent and job["status"] == "RUNNING" and _moving_step(job) and not _onboard(job):
            api.call("POST", f"{TR}/{tr}/cancel", {"reason": "e2e K2: not needed", "reasonCode": "CUSTOMER_CANCELLED"})
            sent["at"] = time.time()

    states = w.run_until(base.terminal, 300, jobs, on_tick=cancel_when_moving)
    w.stop()
    w.assert_safe()
    assert sent, "the robot never drove to the pickup"
    assert states[jobs[0]]["status"] == "CANCELLED", states[jobs[0]]["status"]
    assert api.call("GET", f"{TR}/{tr}")["status"] == "CANCELLED"
    _record("K2", seconds_to_cancelled=round(time.time() - sent["at"], 1), robots=_settled(api, pair))


def test_K3_cancel_while_the_robot_waits_for_staff_at_the_pickup(api, pair):
    _start(api, pair, "K3")
    tr, jobs = _request(api, pair, [("EP-INBOUND", "EP-SHELF-B")])
    w = _watch(api, pair, "K3", staff=False)

    def ready(states):
        stops = api.call("GET", "/api/workflow-execution/handover-stops").get("items", [])
        return any(s["jobId"] == jobs[0] and s.get("robotReadyAt") for s in stops)

    w.run_until(ready, 300, jobs)
    api.call("POST", f"{TR}/{tr}/cancel", {"reason": "e2e K3: goods not there", "reasonCode": "STOCK_PROBLEM"})
    states = w.run_until(base.terminal, 120, jobs)
    w.stop()
    assert states[jobs[0]]["status"] == "CANCELLED", states[jobs[0]]["status"]
    stops = [s for s in api.call("GET", "/api/workflow-execution/handover-stops").get("items", []) if s["jobId"] == jobs[0]]
    assert not stops, f"no handover is offered for a cancelled Job: {stops}"
    _record("K3", request=api.call("GET", f"{TR}/{tr}")["status"], robots=_settled(api, pair))


def test_K4_cancel_with_cargo_on_board_then_deliver_anyway(api, pair):
    _start(api, pair, "K4")
    tr, jobs = _request(api, pair, [("EP-INBOUND", "EP-SHELF-A-2")])
    w = _watch(api, pair, "K4")
    sent = {}

    def cancel_once_onboard(states):
        job = states[jobs[0]]
        if not sent and job["status"] == "RUNNING" and _onboard(job):
            reply = api.call("POST", f"{TR}/{tr}/cancel", {"reason": "e2e K4: customer cancelled", "reasonCode": "CUSTOMER_CANCELLED"})
            sent["status"] = reply["status"]

    def held(states):
        return bool(sent) and _cancellation(api, tr)["status"] == "RECOVERING"

    w.run_until(held, 300, jobs, on_tick=cancel_once_onboard)
    assert sent["status"] == "CANCELLING", sent
    cancellation = _cancellation(api, tr)
    item = cancellation["items"][0]
    assert "CONTINUE_DELIVERY" in item["actions"], item
    assert _job(api, jobs[0])["status"] == "PAUSED"
    api.call("POST", f"{TR}/{tr}/cancellation/jobs/{jobs[0]}/deliver", {})
    states = w.run_until(base.terminal, 300, jobs)
    w.stop()
    w.assert_safe()
    assert states[jobs[0]]["status"] in ("COMPLETED", "CANCELLED"), states[jobs[0]]["status"]
    final = _cancellation(api, tr)
    _record("K4", cancellation=final["status"], job=states[jobs[0]]["status"], request=api.call("GET", f"{TR}/{tr}")["status"],
            robots=_settled(api, pair))
    assert final["status"] == "FINALIZED", final["status"]


# ---------------------------------------------------------------------- one request, two robots


def test_K5_cancel_a_request_running_on_both_robots_before_pickup(api, pair):
    _start(api, pair, "K5")
    # Six containers = two Jobs of three (JobPlanning__MaxContainersPerJob=3): one per robot.
    tr, jobs = _request(api, pair, [("EP-INBOUND", f"EP-SHELF-{s}") for s in ("A-1", "A-1", "A-2", "B", "B", "C")])
    assert len(jobs) == 2, jobs
    w = _watch(api, pair, "K5", staff=False)
    sent = {}

    def cancel_when_both_run(states):
        if not sent and all(v["status"] == "RUNNING" for v in states.values()) and not any(_onboard(v) for v in states.values()):
            api.call("POST", f"{TR}/{tr}/cancel", {"reason": "e2e K5", "reasonCode": "CUSTOMER_CANCELLED"})
            sent["at"] = time.time()

    states = w.run_until(base.terminal, 300, jobs, on_tick=cancel_when_both_run)
    w.stop()
    w.assert_safe()
    assert sent, "both Jobs never ran together"
    assert all(v["status"] == "CANCELLED" for v in states.values()), {k[:8]: v["status"] for k, v in states.items()}
    _record("K5", seconds_to_cancelled=round(time.time() - sent["at"], 1), request=api.call("GET", f"{TR}/{tr}")["status"],
            robots=_settled(api, pair))


def test_K6_cancel_when_one_robot_carries_cargo_and_the_other_does_not(api, pair):
    _start(api, pair, "K6")
    tr, jobs = _request(api, pair, [("EP-INBOUND", "EP-SHELF-A-1")] * 3 + [("EP-SHELF-C", "EP-OUTBOUND")] * 3)
    assert len(jobs) == 2, jobs
    w = _watch(api, pair, "K6")
    sent = {}

    def cancel_on_mixed(states):
        loaded = [j for j, v in states.items() if v["status"] == "RUNNING" and _onboard(v)]
        empty = [j for j, v in states.items() if v["status"] == "RUNNING" and not _onboard(v)]
        if not sent and loaded and empty:
            api.call("POST", f"{TR}/{tr}/cancel", {"reason": "e2e K6", "reasonCode": "CUSTOMER_CANCELLED"})
            sent.update(loaded=loaded[0], empty=empty[0])
            w.staff = lambda jobs: None  # from now on nobody confirms: the empty one must end by itself

    def empty_done_loaded_held(states):
        return bool(sent) and states[sent["empty"]]["status"] == "CANCELLED" and _cancellation(api, tr)["status"] == "RECOVERING"

    w.run_until(empty_done_loaded_held, 400, jobs, on_tick=cancel_on_mixed)
    assert sent, "never one loaded and one empty Job at the same time"
    api.call("POST", f"{TR}/{tr}/cancellation/jobs/{sent['loaded']}/deliver", {})
    w.staff = flows._staff.__get__(w)  # staff back on the floor for the delivery
    states = w.run_until(base.terminal, 400, jobs)
    w.stop()
    w.assert_safe()
    _record("K6", empty=states[sent["empty"]]["status"], loaded=states[sent["loaded"]]["status"],
            cancellation=_cancellation(api, tr)["status"], request=api.call("GET", f"{TR}/{tr}")["status"], robots=_settled(api, pair))
    assert states[sent["empty"]]["status"] == "CANCELLED"
    assert states[sent["loaded"]]["status"] in ("COMPLETED", "CANCELLED")


# ---------------------------------------------------------------------- pause / resume


def _held_still(sim, seconds: float = 4.0) -> bool:
    x, y = sim.x, sim.y
    time.sleep(seconds)
    return abs(sim.x - x) < 0.01 and abs(sim.y - y) < 0.01


def _sim_of(pair, job_id: str):
    robot_id = base.assignments(job_id)[-1][0]
    return pair[1] if pair[1].robot_id == robot_id else pair[2]


def test_P1_pause_while_driving_then_resume(api, pair):
    _start(api, pair, "P1")
    tr, jobs = _request(api, pair, [("EP-SHELF-C", "EP-SHELF-A-2")])
    w = _watch(api, pair, "P1")
    paused = {}

    def pause_when_moving(states):
        job = states[jobs[0]]
        if not paused and job["status"] == "RUNNING" and _moving_step(job):
            api.call("POST", f"{JOBS}/{jobs[0]}/pause", {"reason": "e2e P1: person in the aisle"})
            paused["at"] = time.time()

    w.run_until(lambda s: bool(paused), 300, jobs, on_tick=pause_when_moving)
    sim = _sim_of(pair, jobs[0])
    assert _job(api, jobs[0])["status"] == "PAUSED"
    assert _held_still(sim), "a paused robot does not move"
    assert _robot(api, sim.robot_id)["motionHeld"] is True
    api.call("POST", f"{JOBS}/{jobs[0]}/resume", {})
    states = w.run_until(base.terminal, 300, jobs)
    w.stop()
    w.assert_safe()
    assert states[jobs[0]]["status"] == "COMPLETED", states[jobs[0]]["status"]
    _record("P1", job=states[jobs[0]]["status"], request=api.call("GET", f"{TR}/{tr}")["status"], robots=_settled(api, pair))


def test_P2_pause_while_waiting_for_staff_the_confirm_waits_for_the_resume(api, pair):
    _start(api, pair, "P2")
    tr, jobs = _request(api, pair, [("EP-INBOUND", "EP-SHELF-A-1")])
    w = _watch(api, pair, "P2", staff=False)

    def ready(states):
        stops = api.call("GET", "/api/workflow-execution/handover-stops").get("items", [])
        return any(s["jobId"] == jobs[0] and s.get("robotReadyAt") for s in stops)

    w.run_until(ready, 300, jobs)
    api.call("POST", f"{JOBS}/{jobs[0]}/pause", {"reason": "e2e P2: forklift in the bay"})
    assert _job(api, jobs[0])["status"] == "PAUSED"
    refused = None
    try:
        flows._staff.__get__(w)(jobs)
    except AssertionError as e:  # noqa: PERF203 - a refusal is what is expected
        refused = str(e)[:200]
    confirmed_while_paused = w.confirmations
    api.call("POST", f"{JOBS}/{jobs[0]}/resume", {})
    w.staff = flows._staff.__get__(w)
    states = w.run_until(base.terminal, 300, jobs)
    w.stop()
    _record("P2", confirmed_while_paused=confirmed_while_paused, refusal=refused or (w.notes[-1] if w.notes else None),
            job=states[jobs[0]]["status"], robots=_settled(api, pair))
    assert states[jobs[0]]["status"] == "COMPLETED", states[jobs[0]]["status"]


def test_P3_pause_then_cancel_the_request_releases_the_robot(api, pair):
    _start(api, pair, "P3")
    tr, jobs = _request(api, pair, [("EP-SHELF-B", "EP-OUTBOUND")])
    w = _watch(api, pair, "P3")
    paused = {}

    def pause_when_moving(states):
        job = states[jobs[0]]
        if not paused and job["status"] == "RUNNING" and _moving_step(job) and not _onboard(job):
            api.call("POST", f"{JOBS}/{jobs[0]}/pause", {"reason": "e2e P3"})
            paused["at"] = time.time()

    w.run_until(lambda s: bool(paused), 300, jobs, on_tick=pause_when_moving)
    sim = _sim_of(pair, jobs[0])
    api.call("POST", f"{TR}/{tr}/cancel", {"reason": "e2e P3: cancelled while paused", "reasonCode": "CUSTOMER_CANCELLED"})
    states = w.run_until(base.terminal, 120, jobs)
    w.stop()
    assert states[jobs[0]]["status"] == "CANCELLED", states[jobs[0]]["status"]
    robots = _settled(api, pair)
    _record("P3", job=states[jobs[0]]["status"], request=api.call("GET", f"{TR}/{tr}")["status"], robots=robots)
    assert not _robot(api, sim.robot_id)["motionHeld"], "a cancelled paused Job must not leave its robot held"


def test_P4_pause_one_job_of_two_the_other_finishes(api, pair):
    _start(api, pair, "P4")
    tr, jobs = _request(api, pair, [("EP-INBOUND", "EP-SHELF-A-1")] * 3 + [("EP-INBOUND", "EP-SHELF-B")] * 3)
    assert len(jobs) == 2, jobs
    w = _watch(api, pair, "P4")
    paused = {}

    def pause_first_when_both_run(states):
        if not paused and all(v["status"] == "RUNNING" for v in states.values()) and _moving_step(states[jobs[0]]):
            api.call("POST", f"{JOBS}/{jobs[0]}/pause", {"reason": "e2e P4"})
            paused["at"] = time.time()

    def other_done(states):
        return bool(paused) and states[jobs[1]]["status"] in ("COMPLETED", "FAILED", "CANCELLED")

    states = w.run_until(other_done, 600, jobs, on_tick=pause_first_when_both_run)
    assert states[jobs[1]]["status"] == "COMPLETED", states[jobs[1]]["status"]
    assert states[jobs[0]]["status"] == "PAUSED", states[jobs[0]]["status"]
    api.call("POST", f"{JOBS}/{jobs[0]}/resume", {})
    states = w.run_until(base.terminal, 600, jobs)
    w.stop()
    w.assert_safe()
    _record("P4", jobs=[v["status"] for v in states.values()], request=api.call("GET", f"{TR}/{tr}")["status"], robots=_settled(api, pair))
    assert all(v["status"] == "COMPLETED" for v in states.values())


def test_P5_emergency_stop_then_resume_the_job(api, pair):
    _start(api, pair, "P5")
    tr, jobs = _request(api, pair, [("EP-SHELF-A-1", "EP-SHELF-C")])
    w = _watch(api, pair, "P5")
    stopped = {}

    def estop_when_moving(states):
        job = states[jobs[0]]
        if not stopped and job["status"] == "RUNNING" and _moving_step(job):
            robot_id = base.assignments(jobs[0])[-1][0]
            stopped.update(api.call("POST", f"/api/workflow-execution/robots/{robot_id}/emergency-stop", {"reason": "e2e P5"}))

    w.run_until(lambda s: bool(stopped), 300, jobs, on_tick=estop_when_moving)
    sim = _sim_of(pair, jobs[0])
    assert stopped["pausedJobs"], stopped
    assert _job(api, jobs[0])["status"] == "PAUSED"
    assert _held_still(sim), "an emergency-stopped robot does not move"
    api.call("POST", f"{JOBS}/{jobs[0]}/resume", {})
    states = w.run_until(base.terminal, 300, jobs)
    w.stop()
    w.assert_safe()
    _record("P5", cleared=stopped["clearedCommands"], job=states[jobs[0]]["status"], robots=_settled(api, pair))
    assert states[jobs[0]]["status"] == "COMPLETED", states[jobs[0]]["status"]
