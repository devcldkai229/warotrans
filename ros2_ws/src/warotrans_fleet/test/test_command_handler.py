from warotrans_fleet.command_handler import CommandHandler
from warotrans_fleet.payload_builder import build_command_ack, build_command_result


class FakeNav2:
    def __init__(self):
        self.sent = []
        self.cancelled = False

    def send_navigate(self, *, frame_id, x, y, yaw, command_id):
        self.sent.append((frame_id, x, y, yaw, command_id))

    def cancel_active(self):
        self.cancelled = True


def test_reject_navigate_when_not_localized():
    results = []
    h = CommandHandler(
        localization_provider=lambda: "UNKNOWN",
        nav2=FakeNav2(),
        on_result=lambda *a: results.append(a),
    )
    d = h.handle_command(
        {
            "commandId": "c1",
            "type": "NAVIGATE_TO_POSE",
            "payload": {"frameId": "map", "x": 1.0, "y": 2.0, "yaw": 0.0},
        }
    )
    assert d.accepted is False
    assert d.reason_code == "NOT_LOCALIZED"


def test_accept_navigate_and_busy_second():
    nav = FakeNav2()
    h = CommandHandler(
        localization_provider=lambda: "LOCALIZED",
        nav2=nav,
        on_result=lambda *a: None,
    )
    d1 = h.handle_command(
        {
            "commandId": "c1",
            "type": "NAVIGATE_TO_POSE",
            "payload": {"frameId": "map", "x": 1.0, "y": 2.0, "yaw": 0.1},
        }
    )
    assert d1.accepted is True
    assert h.active_command_id == "c1"
    assert len(nav.sent) == 1

    d2 = h.handle_command(
        {
            "commandId": "c2",
            "type": "NAVIGATE_TO_POSE",
            "payload": {"frameId": "map", "x": 3.0, "y": 4.0, "yaw": 0.0},
        }
    )
    assert d2.accepted is False
    assert d2.reason_code == "BUSY"


def test_duplicate_command_idempotent():
    h = CommandHandler(
        localization_provider=lambda: "LOCALIZED",
        nav2=FakeNav2(),
        on_result=lambda *a: None,
    )
    msg = {
        "commandId": "c1",
        "type": "NAVIGATE_TO_POSE",
        "payload": {"frameId": "map", "x": 1.0, "y": 2.0, "yaw": 0.0},
    }
    assert h.handle_command(msg).accepted is True
    d2 = h.handle_command(msg)
    assert d2.accepted is True
    assert d2.reason_code == "ALREADY_ACCEPTED"


def test_cancel_unknown_target():
    h = CommandHandler(
        localization_provider=lambda: "LOCALIZED",
        nav2=FakeNav2(),
        on_result=lambda *a: None,
    )
    d = h.handle_command(
        {
            "commandId": "cancel-1",
            "type": "CANCEL",
            "payload": {"targetCommandId": "missing"},
        }
    )
    assert d.accepted is False
    assert d.reason_code == "UNKNOWN_TARGET"


def test_ack_result_payloads():
    ack = build_command_ack(robot_code="RBT-001", command_id="c1", accepted=True, message_id="m1", sent_at="t")
    assert ack["accepted"] is True
    assert ack["commandId"] == "c1"
    res = build_command_result(
        robot_code="RBT-001",
        command_id="c1",
        outcome="SUCCEEDED",
        message_id="m2",
        sent_at="t",
    )
    assert res["outcome"] == "SUCCEEDED"
