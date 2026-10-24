"""Pure-Python tests for InteractionController + ButtonEdgeDetector (no ROS / GPIO)."""

from __future__ import annotations

from warotrans_fleet.confirm_button import ButtonEdgeDetector
from warotrans_fleet.interaction_contract import (
    InteractionRequest,
    InteractionResult,
    parse_interaction_request,
)
from warotrans_fleet.interaction_controller import InteractionController, InteractionState


def _req(step: str = "step-1", typ: str = "PICKUP_CONFIRM") -> InteractionRequest:
    return InteractionRequest(
        message_id="m1",
        robot_code="RBT-001",
        job_id="job-1",
        job_step_id=step,
        interaction_type=typ,
        container_id="c1",
        endpoint_id="e1",
        sent_at="t",
    )


def _ctrl():
    published = []
    logs = []
    ctrl = InteractionController(
        robot_code="RBT-001",
        boot_id="boot",
        next_sequence=lambda: len(published) + 1,
        publish_confirm=published.append,
        log_info=logs.append,
        log_warn=logs.append,
        resend_interval_sec=5.0,
    )
    return ctrl, published, logs


def test_button_ignored_when_not_armed():
    ctrl, published, logs = _ctrl()
    ctrl.on_button_press()
    assert published == []
    assert any("BUTTON_IGNORED_NOT_ARMED" in m for m in logs)


def test_request_arms_button():
    ctrl, _, logs = _ctrl()
    ctrl.handle_request(_req())
    assert ctrl.state == InteractionState.ARMED
    assert ctrl.interaction_armed
    assert any("INTERACTION_ARMED" in m for m in logs)


def test_one_press_one_confirm():
    ctrl, published, _ = _ctrl()
    ctrl.handle_request(_req())
    ctrl.set_monotonic(0.0)
    ctrl.on_button_press()
    assert len(published) == 1
    assert published[0]["jobStepId"] == "step-1"
    assert published[0]["source"] == "ROBOT_BUTTON"
    assert ctrl.state == InteractionState.CONFIRM_PENDING


def test_duplicate_request_idempotent():
    ctrl, _, _ = _ctrl()
    ctrl.handle_request(_req())
    ctrl.handle_request(_req())
    assert ctrl.state == InteractionState.ARMED
    assert ctrl.active.job_step_id == "step-1"


def test_unsupported_type_not_parsed():
    try:
        parse_interaction_request(
            {
                "schemaVersion": 1,
                "messageId": "m",
                "robotCode": "RBT-001",
                "jobId": "j",
                "jobStepId": "s",
                "interactionType": "REPORT_ISSUE",
                "containerId": "c",
                "endpointId": "e",
                "sentAt": "t",
            },
            local_robot_code="RBT-001",
        )
        assert False, "expected error"
    except Exception as exc:
        assert "unsupported" in str(exc)


def test_accepted_result_clears():
    ctrl, published, logs = _ctrl()
    ctrl.handle_request(_req())
    ctrl.on_button_press()
    mid = published[0]["messageId"]
    ctrl.handle_result(
        InteractionResult("r", "RBT-001", "step-1", mid, True, None, "t")
    )
    assert ctrl.state == InteractionState.IDLE
    assert ctrl.active is None
    assert any("INTERACTION_ACCEPTED" in m for m in logs)


def test_terminal_reject_clears():
    ctrl, published, _ = _ctrl()
    ctrl.handle_request(_req())
    ctrl.on_button_press()
    mid = published[0]["messageId"]
    ctrl.handle_result(
        InteractionResult("r", "RBT-001", "step-1", mid, False, "interaction_wrong_robot", "t")
    )
    assert ctrl.state == InteractionState.IDLE


def test_resend_keeps_same_message_id():
    ctrl, published, _ = _ctrl()
    ctrl.handle_request(_req())
    ctrl.set_monotonic(0.0)
    ctrl.on_button_press()
    mid = published[0]["messageId"]
    ctrl.tick(5.1)
    assert len(published) == 2
    assert published[1]["messageId"] == mid


def test_bounce_one_event():
    det = ButtonEdgeDetector(active_low=True, debounce_ms=50, min_interval_ms=300)
    events = 0
    # bounce around press
    t = 0
    for level in [1, 0, 1, 0, 0, 0, 0, 0]:
        t += 10
        if det.sample(level, t):
            events += 1
    # hold
    for _ in range(20):
        t += 10
        if det.sample(0, t):
            events += 1
    assert events == 1


def test_long_hold_one_event():
    det = ButtonEdgeDetector(active_low=True, debounce_ms=50, min_interval_ms=300)
    events = 0
    t = 0
    # press and hold 2 seconds
    for level in [1] + [0] * 200:
        t += 10
        if det.sample(level, t):
            events += 1
    assert events == 1


def test_release_then_press_second_event():
    det = ButtonEdgeDetector(active_low=True, debounce_ms=50, min_interval_ms=300)
    events = 0
    t = 0
    # first press
    for level in [1, 0, 0, 0, 0, 0, 0]:
        t += 10
        if det.sample(level, t):
            events += 1
    # release
    for level in [1, 1, 1, 1, 1, 1]:
        t += 10
        if det.sample(level, t):
            events += 1
    # wait past min_interval
    t += 300
    for level in [0, 0, 0, 0, 0, 0]:
        t += 10
        if det.sample(level, t):
            events += 1
    assert events == 2
