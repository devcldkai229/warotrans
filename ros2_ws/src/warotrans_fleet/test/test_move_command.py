"""MOVE (NAVIGATE_TO_POSE) + CANCEL qua CommandRouter với Nav2 giả — không cần ROS."""

import pytest
from fleet_fakes import ROBOT, Harness, cancel, navigate

from warotrans_fleet.navigator import STATUS_ABORTED, STATUS_CANCELED, STATUS_SUCCEEDED

CMD = "11111111-1111-1111-1111-111111111111"
CANCEL_CMD = "22222222-2222-2222-2222-222222222222"


# 1. valid NAVIGATE_TO_POSE → ACK ACCEPTED → Nav2 goal → RESULT SUCCEEDED
def test_valid_navigate_ack_then_goal_then_succeeded():
    h = Harness()
    h.send(navigate(CMD))

    # ACK được publish TRƯỚC khi gửi goal Nav2.
    assert h.recorder.kinds() == ["ack", "nav2_goal"]
    ack = h.recorder.acks[0]
    assert ack["accepted"] is True and ack["reasonCode"] is None
    assert ack["commandId"] == CMD and ack["robotCode"] == ROBOT
    assert ack["jobStepId"] == "bbbbbbbb-0000-0000-0000-000000000001"

    goal = h.nav.goals[0][0]
    assert (goal.frame_id, goal.x, goal.y, goal.yaw) == ("map", 4.25, 2.10, 1.5708)
    assert h.move.navigation_status == "NAVIGATING"
    assert h.move.active_command_id == CMD

    h.nav.accept()
    h.nav.feedback(3.0)
    assert h.recorder.results == []  # chưa có kết quả Nav2 → chưa có RESULT

    h.nav.finish(STATUS_SUCCEEDED)
    result = h.recorder.results[0]
    assert result["outcome"] == "SUCCEEDED" and result["errorCode"] is None
    assert result["commandId"] == CMD
    assert result["jobAssignmentId"] == "aaaaaaaa-0000-0000-0000-000000000001"
    assert result["jobStepId"] == "bbbbbbbb-0000-0000-0000-000000000001"
    assert result["result"]["navigationResult"] == "SUCCEEDED"

    assert h.move.active_command_id is None
    assert h.move.navigation_status == "SUCCEEDED"
    h.tick(3.0)
    assert h.move.navigation_status == "IDLE"


# 2. Nav2 goal rejected → RESULT FAILED
def test_goal_rejected_by_nav2_results_failed():
    h = Harness()
    h.send(navigate(CMD))
    h.nav.reject()

    assert h.recorder.acks[0]["accepted"] is True
    result = h.recorder.results[0]
    assert result["outcome"] == "FAILED"
    assert result["errorCode"] == "GOAL_REJECTED"
    assert "rejected" in result["errorMessage"]
    assert h.move.active_command_id is None
    assert h.move.navigation_status == "FAILED"


# 3. Nav2 execution failure → RESULT FAILED
def test_nav2_aborted_results_navigation_failed():
    h = Harness()
    h.send(navigate(CMD))
    h.nav.accept()
    h.nav.finish(STATUS_ABORTED, error_code=104, error_message="No valid path")

    result = h.recorder.results[0]
    assert result["outcome"] == "FAILED"
    assert result["errorCode"] == "NAVIGATION_FAILED"
    assert "No valid path" in result["errorMessage"]
    assert result["result"]["nav2ErrorCode"] == 104


# 4. timeout → Nav2 cancel → RESULT (FAILED / NAVIGATION_TIMEOUT)
def test_timeout_cancels_nav2_and_waits_for_confirmation():
    h = Harness()
    h.send(navigate(CMD, timeoutSeconds=30))
    h.nav.accept()

    h.tick(29.0)
    assert h.nav.cancel_requests == []
    h.tick(1.5)
    assert len(h.nav.cancel_requests) == 1

    # Chưa publish TIMEOUT khi Nav2 chưa xác nhận dừng goal.
    h.nav.answer_cancel(True)
    assert h.recorder.results == []
    assert h.move.active_command_id == CMD

    h.nav.finish(STATUS_CANCELED)
    result = h.recorder.results[0]
    assert result["outcome"] == "FAILED"  # backend chỉ nhận SUCCEEDED|FAILED|CANCELED
    assert result["errorCode"] == "NAVIGATION_TIMEOUT"
    assert result["result"]["navigationResult"] == "TIMEOUT"
    assert h.move.active_command_id is None


def test_default_timeout_param_applies_when_command_has_none():
    h = Harness(default_timeout_sec=5.0)
    h.send(navigate(CMD))
    h.nav.accept()
    h.tick(5.0)
    assert len(h.nav.cancel_requests) == 1


# 5. Backend CANCEL → Nav2 cancel → RESULT CANCELED
def test_backend_cancel_flow():
    h = Harness()
    h.send(navigate(CMD))
    h.nav.accept()

    h.send(cancel(CMD, CANCEL_CMD))
    cancel_ack = h.recorder.acks[-1]
    assert cancel_ack["commandId"] == CANCEL_CMD and cancel_ack["accepted"] is True
    assert len(h.nav.cancel_requests) == 1

    h.nav.answer_cancel(True)
    assert h.recorder.results == []  # chỉ đổi status local là không đủ — chờ Nav2

    h.nav.finish(STATUS_CANCELED)
    nav_result, cancel_result = h.recorder.results
    # RESULT của navigate đi trước (backend chờ navigate CANCELED).
    assert nav_result["commandId"] == CMD
    assert nav_result["outcome"] == "CANCELED"
    assert nav_result["errorCode"] == "NAVIGATION_CANCELED"
    assert cancel_result["commandId"] == CANCEL_CMD
    assert cancel_result["outcome"] == "SUCCEEDED"
    assert h.move.navigation_status == "CANCELED"
    h.tick(3.0)
    assert h.move.navigation_status == "IDLE"


def test_cancel_unknown_target_rejected():
    h = Harness()
    h.send(cancel("not-running", CANCEL_CMD))
    ack = h.recorder.acks[0]
    assert ack["accepted"] is False and ack["reasonCode"] == "UNKNOWN_TARGET"
    assert h.nav.cancel_requests == []


def test_cancel_rejected_by_nav2_keeps_tracking_real_goal():
    h = Harness()
    h.send(navigate(CMD))
    h.nav.accept()
    h.send(cancel(CMD, CANCEL_CMD))
    h.nav.answer_cancel(False, "ERROR_REJECTED")

    cancel_result = h.recorder.results[0]
    assert cancel_result["commandId"] == CANCEL_CMD
    assert cancel_result["outcome"] == "FAILED" and cancel_result["errorCode"] == "CANCEL_REJECTED"
    # Robot vẫn chạy → move vẫn active, kết quả thật vẫn được báo.
    assert h.move.active_command_id == CMD
    h.nav.finish(STATUS_SUCCEEDED)
    assert h.recorder.results[-1]["outcome"] == "SUCCEEDED"


def test_cancel_before_goal_accepted_is_sent_after_accept():
    h = Harness()
    h.send(navigate(CMD))
    h.send(cancel(CMD, CANCEL_CMD))
    assert h.nav.cancel_requests == []  # chưa có goal handle

    h.nav.accept()
    assert len(h.nav.cancel_requests) == 1
    h.nav.answer_cancel(True)
    h.nav.finish(STATUS_CANCELED)
    assert [r["outcome"] for r in h.recorder.results] == ["CANCELED", "SUCCEEDED"]


# 6. duplicate commandId → không tạo Nav2 goal thứ hai
def test_duplicate_command_does_not_send_second_goal():
    h = Harness()
    h.send(navigate(CMD))
    h.send(navigate(CMD))
    assert len(h.nav.goals) == 1
    dup_ack = h.recorder.acks[-1]
    assert dup_ack["accepted"] is True and dup_ack["reasonCode"] == "ALREADY_ACCEPTED"

    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    h.send(navigate(CMD))  # duplicate sau khi đã COMPLETE
    assert len(h.nav.goals) == 1
    assert h.recorder.acks[-1]["reasonCode"] == "ALREADY_ACCEPTED"
    # result được gửi lại (backend idempotent) chứ không execute lại
    assert [r["outcome"] for r in h.recorder.results] == ["SUCCEEDED", "SUCCEEDED"]


def test_duplicate_of_rejected_command_replays_same_rejection():
    h = Harness()
    h.localization = "LOST"
    h.send(navigate(CMD))
    h.localization = "LOCALIZED"
    h.send(navigate(CMD))
    assert [a["reasonCode"] for a in h.recorder.acks] == ["NOT_LOCALIZED", "NOT_LOCALIZED"]
    assert h.nav.goals == []


# 7. MOVE khi robot đang NAVIGATING → BUSY
def test_second_move_while_navigating_is_busy_and_does_not_cancel():
    h = Harness()
    h.send(navigate(CMD))
    h.nav.accept()
    h.send(navigate("33333333-3333-3333-3333-333333333333", x=1.0))

    ack = h.recorder.acks[-1]
    assert ack["accepted"] is False and ack["reasonCode"] == "BUSY"
    assert len(h.nav.goals) == 1
    assert h.nav.cancel_requests == []
    assert h.move.active_command_id == CMD


def test_move_rejected_when_foreign_nav2_goal_running():
    h = Harness()
    h.external_goal = True
    h.send(navigate(CMD))
    assert h.recorder.acks[0]["reasonCode"] == "BUSY"
    assert h.nav.goals == []


def test_move_allowed_again_after_result():
    h = Harness()
    h.send(navigate(CMD))
    h.nav.accept()
    h.nav.finish(STATUS_SUCCEEDED)
    h.send(navigate("44444444-4444-4444-4444-444444444444"))
    assert h.recorder.acks[-1]["accepted"] is True
    assert len(h.nav.goals) == 2


# 8. invalid payload → ACK REJECTED
@pytest.mark.parametrize(
    "mutate, reason",
    [
        (lambda b: b["payload"].pop("x"), "INVALID_PAYLOAD"),
        (lambda b: b["payload"].update(y="2.1"), "INVALID_PAYLOAD"),
        (lambda b: b["payload"].update(yaw=True), "INVALID_PAYLOAD"),
        (lambda b: b["payload"].update(x=float("nan")), "INVALID_PAYLOAD"),
        (lambda b: b["payload"].update(frameId="odom"), "INVALID_PAYLOAD"),
        (lambda b: b["payload"].update(timeoutSeconds=0), "INVALID_PAYLOAD"),
        (lambda b: b.update(payload=None), "INVALID_PAYLOAD"),
        (lambda b: b.update(schemaVersion=2), "INVALID_PAYLOAD"),
        (lambda b: b.update(robotCode="RBT-999"), "WRONG_ROBOT"),
        (lambda b: b.update(type="CHECK"), "UNSUPPORTED_COMMAND"),
    ],
)
def test_invalid_command_rejected(mutate, reason):
    h = Harness()
    body = navigate(CMD)
    mutate(body)
    h.send(body)
    ack = h.recorder.acks[0]
    assert ack["accepted"] is False
    assert ack["reasonCode"] == reason
    assert ack["reasonMessage"]
    assert ack["commandId"] == CMD
    assert h.nav.goals == []
    assert h.move.navigation_status == "IDLE"


def test_missing_command_id_is_dropped_without_ack():
    h = Harness()
    body = navigate(CMD)
    del body["commandId"]
    h.send(body)
    h.send("not even an object")
    assert h.recorder.events == []


def test_not_localized_rejected():
    h = Harness()
    h.localization = "LOST"
    h.send(navigate(CMD))
    assert h.recorder.acks[0]["reasonCode"] == "NOT_LOCALIZED"


def test_nav2_not_ready_rejected():
    h = Harness()
    h.nav.ready = False
    h.send(navigate(CMD))
    assert h.recorder.acks[0]["reasonCode"] == "NAV2_UNAVAILABLE"


def test_map_mismatch_rejected():
    h = Harness(robot_map_version_code="MAP-V2")
    h.send(navigate(CMD, mapVersionCode="MAP-V1"))
    assert h.recorder.acks[0]["reasonCode"] == "MAP_MISMATCH"
    h.send(navigate("55555555-5555-5555-5555-555555555555", mapVersionCode="MAP-V2"))
    assert h.recorder.acks[-1]["accepted"] is True


def test_handler_exception_becomes_internal_error():
    h = Harness()

    def boom() -> str:
        raise RuntimeError("tf buffer exploded")

    h.stack.move._localization_provider = boom
    h.send(navigate(CMD))
    ack = h.recorder.acks[0]
    assert ack["accepted"] is False and ack["reasonCode"] == "INTERNAL_ERROR"
    assert "tf buffer exploded" in ack["reasonMessage"]


# Nav2 không phản hồi
def test_goal_response_timeout_fails_and_orphan_goal_is_canceled():
    h = Harness()
    h.send(navigate(CMD))
    h.tick(10.0)
    result = h.recorder.results[0]
    assert result["outcome"] == "FAILED" and result["errorCode"] == "NAV2_UNAVAILABLE"
    assert h.move.active_command_id is None

    h.nav.accept()  # accept trễ → phải cancel ngay, không để robot chạy goal mồ côi
    assert len(h.nav.cancel_requests) == 1
    assert len(h.recorder.results) == 1


def test_cancel_unconfirmed_fails_command():
    h = Harness()
    h.send(navigate(CMD))
    h.nav.accept()
    h.send(cancel(CMD, CANCEL_CMD))
    h.nav.answer_cancel(True)
    h.tick(10.0)
    nav_result, cancel_result = h.recorder.results
    assert nav_result["errorCode"] == "NAV2_CANCEL_UNCONFIRMED"
    assert cancel_result["outcome"] == "FAILED"


# 9. (phần pure-python) router không block khi goal đang chạy, telemetry snapshot phản ánh NAVIGATING
def test_router_returns_immediately_and_snapshot_reflects_navigation():
    h = Harness()
    h.send(navigate(CMD))  # trả về ngay dù Nav2 chưa trả lời gì
    h.nav.accept()
    for _ in range(10):
        h.tick(0.2)  # timer tick vẫn chạy trong khi navigate
    assert h.move.navigation_status == "NAVIGATING"
    assert h.move.active_command_id == CMD
    assert h.recorder.results == []
