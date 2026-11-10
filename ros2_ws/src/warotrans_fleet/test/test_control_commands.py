"""STOP / RESUME / LOCALIZE from the backend: the hold blocks MOVEs and is reported; LOCALIZE refuses a running MOVE."""

import logging

from fleet_fakes import ROBOT, Recorder

from warotrans_fleet.command_router import AckDecision, CommandRouter
from warotrans_fleet.control_command_handlers import (
    LocalizeCommandHandler,
    MotionHold,
    ResumeCommandHandler,
    StopCommandHandler,
)
from warotrans_fleet.payload_builder import build_telemetry


def _body(command_id: str, command_type: str, payload: dict | None = None) -> dict:
    return {
        "schemaVersion": 1,
        "messageId": "m",
        "commandId": command_id,
        "robotCode": ROBOT,
        "type": command_type,
        "payload": payload or {},
    }


class FakeMove:
    """Stands in for the MOVE handler: refuses while held, like MoveCommandHandler does."""

    command_type = "NAVIGATE_TO_POSE"

    def __init__(self, hold: MotionHold) -> None:
        self.hold = hold
        self.active: str | None = None
        self.halted: list[str | None] = []

    def handle(self, command, session):
        if self.hold.held:
            from warotrans_fleet.command_contract import CommandRejected

            raise CommandRejected("MOTION_HELD", "held")
        self.active = command.command_id
        return AckDecision.accept()

    def halt(self):
        self.halted.append(self.active)
        cancelled, self.active = self.active, None
        return cancelled


def _setup():
    rec = Recorder()
    router = CommandRouter(robot_code=ROBOT, publisher=rec, log=logging.getLogger("t"))
    hold = MotionHold()
    move = FakeMove(hold)
    gate: list[bool] = []
    poses: list[tuple[float, float, float]] = []
    router.register(move)
    router.register(StopCommandHandler(hold, move.halt, gate.append, logging.getLogger("t")))
    router.register(ResumeCommandHandler(hold, gate.append, logging.getLogger("t")))
    router.register(
        LocalizeCommandHandler(lambda x, y, yaw: poses.append((x, y, yaw)), busy=lambda: move.active is not None,
                               log=logging.getLogger("t"))
    )
    return rec, router, hold, move, gate, poses


def test_stop_holds_cancels_the_running_move_and_refuses_new_ones_until_resume():
    rec, router, hold, move, gate, _ = _setup()
    router.handle_message(_body("nav-1", "NAVIGATE_TO_POSE"))
    assert move.active == "nav-1"

    router.handle_message(_body("stop-1", "STOP", {"mode": "PAUSE", "reason": "operator"}))

    assert hold.held and hold.mode == "PAUSE"
    assert move.halted == ["nav-1"]
    assert gate == [True]
    stop_result = rec.results[-1]
    assert stop_result["commandId"] == "stop-1" and stop_result["outcome"] == "SUCCEEDED"

    router.handle_message(_body("nav-2", "NAVIGATE_TO_POSE"))
    refused = rec.acks[-1]
    assert refused["commandId"] == "nav-2" and refused["accepted"] is False and refused["reasonCode"] == "MOTION_HELD"

    router.handle_message(_body("resume-1", "RESUME"))
    assert not hold.held
    assert gate == [True, False]
    router.handle_message(_body("nav-3", "NAVIGATE_TO_POSE"))
    assert rec.acks[-1]["accepted"] is True


def test_an_unknown_stop_mode_is_treated_as_stop():
    rec, router, hold, *_ = _setup()
    router.handle_message(_body("stop-1", "STOP", {"mode": "WHATEVER"}))
    assert hold.mode == "STOP"


def test_localize_publishes_the_pose_and_is_refused_while_a_move_runs():
    rec, router, hold, move, _, poses = _setup()
    router.handle_message(_body("loc-1", "LOCALIZE", {"x": 0.495, "y": 3.722, "yaw": 3.14159, "frameId": "map"}))
    assert poses == [(0.495, 3.722, 3.14159)]
    assert rec.results[-1]["outcome"] == "SUCCEEDED"

    router.handle_message(_body("nav-1", "NAVIGATE_TO_POSE"))
    router.handle_message(_body("loc-2", "LOCALIZE", {"x": 1.0, "y": 1.0, "yaw": 0.0, "frameId": "map"}))
    assert rec.acks[-1]["accepted"] is False and rec.acks[-1]["reasonCode"] == "BUSY"
    assert len(poses) == 1


def test_localize_rejects_a_bad_payload():
    rec, router, _, _, _, poses = _setup()
    router.handle_message(_body("loc-1", "LOCALIZE", {"x": "a", "y": 1.0, "yaw": 0.0}))
    assert rec.acks[-1]["accepted"] is False and rec.acks[-1]["reasonCode"] == "INVALID_PAYLOAD"
    assert poses == []


def test_telemetry_reports_the_hold():
    common = dict(robot_code=ROBOT, boot_id="b", sequence=1, map_version_code="M", pose=None, battery_percent=None,
                  navigation_status="IDLE", localization_status="LOCALIZED", linear_velocity=0.0, angular_velocity=0.0)
    assert build_telemetry(**common, motion_hold=True)["motionHold"] is True
    assert build_telemetry(**common)["motionHold"] is None


def test_the_real_move_handler_refuses_while_held_and_halt_cancels_its_goal():
    from fleet_fakes import Harness, navigate
    from warotrans_fleet.navigator import STATUS_CANCELED

    held = {"value": False}
    h = Harness(motion_hold_active=lambda: held["value"])
    h.send(navigate("11111111-1111-1111-1111-111111111111"))
    h.nav.accept()
    assert h.move.active_command_id == "11111111-1111-1111-1111-111111111111"

    assert h.move.halt() == "11111111-1111-1111-1111-111111111111"
    assert "nav2_cancel" in h.recorder.kinds()
    h.nav.finish(STATUS_CANCELED)

    held["value"] = True
    h.send(navigate("33333333-3333-3333-3333-333333333333"))
    refused = h.recorder.acks[-1]
    assert refused["commandId"] == "33333333-3333-3333-3333-333333333333"
    assert refused["accepted"] is False and refused["reasonCode"] == "MOTION_HELD"
