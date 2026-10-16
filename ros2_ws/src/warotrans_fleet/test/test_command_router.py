"""CommandRouter: dispatch theo type, ledger dedup, ACK luôn đi trước RESULT."""

import logging

from fleet_fakes import ROBOT, Recorder

from warotrans_fleet.command_router import AckDecision, CommandRouter
from warotrans_fleet.payload_builder import build_command_ack, build_command_result


def _body(command_id: str, command_type: str = "ECHO") -> dict:
    return {
        "schemaVersion": 1,
        "messageId": "m",
        "commandId": command_id,
        "robotCode": ROBOT,
        "type": command_type,
        "payload": {},
    }


class EchoHandler:
    """Handler hoàn tất NGAY trong handle() — kiểm tra router hoãn RESULT tới sau ACK."""

    command_type = "ECHO"

    def __init__(self) -> None:
        self.calls = 0

    def handle(self, command, session):
        self.calls += 1
        session.complete("SUCCEEDED")
        return AckDecision.accept()


def _router(recorder: Recorder, capacity: int = 500) -> CommandRouter:
    return CommandRouter(robot_code=ROBOT, publisher=recorder, log=logging.getLogger("t"), ledger_capacity=capacity)


def test_new_handler_type_plugs_in_without_touching_router():
    rec = Recorder()
    router = _router(rec)
    handler = EchoHandler()
    router.register(handler)
    router.handle_message(_body("c1"))
    assert handler.calls == 1
    assert rec.kinds() == ["ack", "result"]


def test_ledger_eviction_keeps_running_commands():
    rec = Recorder()
    router = _router(rec, capacity=2)

    class Pending:
        command_type = "PENDING"

        def __init__(self):
            self.calls = 0

        def handle(self, command, session):
            self.calls += 1
            return AckDecision.accept()

    pending = Pending()
    router.register(pending)
    router.register(EchoHandler())
    router.handle_message(_body("running", "PENDING"))
    for i in range(5):
        router.handle_message(_body(f"done-{i}"))
    router.handle_message(_body("running", "PENDING"))
    assert pending.calls == 1  # vẫn nhận ra duplicate dù ledger đã evict command cũ


def test_ack_result_payload_shape_is_backward_compatible():
    ack = build_command_ack(robot_code="RBT-001", command_id="c1", accepted=True, message_id="m1", sent_at="t")
    for key in ("schemaVersion", "messageId", "commandId", "robotCode", "accepted", "reasonCode", "sentAt"):
        assert key in ack
    res = build_command_result(robot_code="RBT-001", command_id="c1", outcome="SUCCEEDED", message_id="m2", sent_at="t")
    for key in ("schemaVersion", "messageId", "commandId", "robotCode", "outcome", "errorCode", "sentAt"):
        assert key in res
