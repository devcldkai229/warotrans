"""RB-03 / RB-09: STOP / PAUSE / RESUME and SYSTEM_ACTION on the robot."""

from fleet_fakes import ROBOT, Harness, navigate

from warotrans_fleet.navigator import STATUS_CANCELED

CMD = "11111111-1111-1111-1111-111111111111"


def control(kind: str, command_id: str, **payload) -> dict:
    return {
        "schemaVersion": 1,
        "messageId": "m-" + command_id,
        "commandId": command_id,
        "robotCode": ROBOT,
        "type": kind,
        "issuedAt": "2026-11-08T08:00:00.000Z",
        "payload": payload,
    }


class Speed:
    def __init__(self) -> None:
        self.value = (0.3, 0.0)

    def __call__(self):
        return self.value


def test_stop_cancels_the_goal_and_is_verified_only_once_the_robot_stands_still():
    speed = Speed()
    h = Harness(speed_provider=speed)
    h.send(navigate(CMD))
    h.nav.accept()

    h.send(control("STOP", "33333333-3333-3333-3333-333333333333", mode="STOP", reason="person in aisle"))

    assert h.stack.hold.held
    assert h.nav.cancel_requests, "the running goal is canceled"
    stop_ack = h.recorder.acks[-1]
    assert stop_ack["accepted"] is True
    # Still moving: no RESULT for the STOP yet.
    assert [r for r in h.recorder.results if r["commandId"].startswith("3333")] == []

    speed.value = (0.0, 0.0)
    h.tick(0.3)   # first still sample
    h.tick(0.3)   # still for 0.3 s: not yet the 0.5 s settle time
    assert [r for r in h.recorder.results if r["commandId"].startswith("3333")] == []
    h.tick(0.3)   # still for 0.6 s
    stop_result = [r for r in h.recorder.results if r["commandId"].startswith("3333")][0]
    assert stop_result["outcome"] == "SUCCEEDED"
    assert stop_result["result"]["stoppedVerified"] is True
    assert stop_result["result"]["stoppedCommandId"] == CMD

    # The canceled move reports its own CANCELED result.
    h.nav.answer_cancel(True)
    h.nav.finish(STATUS_CANCELED)
    assert [r for r in h.recorder.results if r["commandId"] == CMD][0]["outcome"] == "CANCELED"


def test_a_held_robot_refuses_navigation_until_resume():
    h = Harness(speed_provider=lambda: (0.0, 0.0))
    h.send(control("STOP", "33333333-3333-3333-3333-333333333333", mode="PAUSE"))

    h.send(navigate("44444444-4444-4444-4444-444444444444"))
    refused = h.recorder.acks[-1]
    assert refused["accepted"] is False and refused["reasonCode"] == "MOTION_HELD"
    assert h.nav.goals == []

    h.send(control("RESUME", "55555555-5555-5555-5555-555555555555"))
    assert not h.stack.hold.held
    h.send(navigate("66666666-6666-6666-6666-666666666666"))
    assert h.recorder.acks[-1]["accepted"] is True
    assert len(h.nav.goals) == 1


def test_stop_without_odometry_is_reported_unverified_and_keeps_holding():
    h = Harness(speed_provider=lambda: None, stop_verify_timeout_sec=2.0)
    h.send(control("STOP", "33333333-3333-3333-3333-333333333333"))

    h.tick(2.5)

    result = h.recorder.results[-1]
    assert result["outcome"] == "FAILED" and result["errorCode"] == "STOP_UNVERIFIED"
    assert h.stack.hold.held, "never claims stopped, never releases on its own"


def test_stop_mode_must_be_stop_or_pause():
    h = Harness()
    h.send(control("STOP", "33333333-3333-3333-3333-333333333333", mode="BRAKE"))
    assert h.recorder.acks[-1]["reasonCode"] == "INVALID_PAYLOAD"
    assert not h.stack.hold.held


def test_system_action_needs_enabling_a_hold_and_no_goal():
    ran = []
    h = Harness(speed_provider=lambda: (0.0, 0.0), system_actions_enabled=("RESTART_STACK",), run_system_action=ran.append)

    h.send(control("SYSTEM_ACTION", "77777777-7777-7777-7777-777777777777", action="RESTART_STACK"))
    assert h.recorder.acks[-1]["reasonCode"] == "NOT_STOPPED"

    h.send(control("SYSTEM_ACTION", "88888888-8888-8888-8888-888888888888", action="REBOOT"))
    assert h.recorder.acks[-1]["reasonCode"] == "SYSTEM_ACTION_DISABLED"

    h.send(control("STOP", "33333333-3333-3333-3333-333333333333"))
    h.send(control("SYSTEM_ACTION", "99999999-9999-9999-9999-999999999999", action="RESTART_STACK"))
    assert h.recorder.acks[-1]["accepted"] is True
    assert ran == ["RESTART_STACK"]
    assert h.stack.system_actions.capabilities == ["RESTART_STACK"]
