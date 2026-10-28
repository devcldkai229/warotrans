"""Active Mode: kết quả TrafficClient → command_result (NAVIGATE + CANCEL) qua MoveCommandHandler."""

from fleet_fakes import Harness as StackHarness
from fleet_fakes import cancel, navigate
from traffic_fakes import GRAPH_HASH, MAP_CODE, authority_payload
from traffic_fakes import Harness as TrafficHarness
from warotrans_fleet import command_contract as cc
from warotrans_fleet import navigation_state as nav_state
from warotrans_fleet.segment_executor import RESULT_ABORTED, RESULT_CANCELED, RESULT_SUCCEEDED

NAV_ID = "11111111-1111-1111-1111-111111111111"


def _active_stack():
    traffic = TrafficHarness()
    stack = StackHarness(
        traffic_mode="active",
        traffic_client=traffic.client,
        robot_map_version_code=MAP_CODE,
        expected_graph_hash=GRAPH_HASH,
    )
    return traffic, stack


def _navigate_active(**authority_overrides):
    body = navigate(NAV_ID)
    body["payload"].update(authority_payload(**authority_overrides))
    return body


def test_final_segment_publishes_navigate_succeeded():
    traffic, stack = _active_stack()
    stack.send(_navigate_active())  # authority_payload mang finalGoal
    assert stack.recorder.acks[-1]["accepted"] is True
    assert stack.move.navigation_status == nav_state.NAVIGATING
    assert stack.move.active_command_id == NAV_ID

    traffic.executor.finish(RESULT_SUCCEEDED)

    result = stack.recorder.results[-1]
    assert result["commandId"] == NAV_ID and result["outcome"] == cc.OUTCOME_SUCCEEDED
    assert stack.recorder.kinds().index("ack") < stack.recorder.kinds().index("result")


def test_fault_publishes_navigate_failed():
    traffic, stack = _active_stack()
    stack.send(_navigate_active())
    traffic.executor.finish(RESULT_ABORTED)

    result = stack.recorder.results[-1]
    assert result["outcome"] == cc.OUTCOME_FAILED and result["errorCode"] == cc.ERROR_CONTROLLER_FAILED
    assert stack.move.navigation_status == nav_state.FAILED


def test_cancel_active_navigate_reports_both_results():
    traffic, stack = _active_stack()
    stack.send(_navigate_active())
    stack.send(cancel(NAV_ID))
    assert stack.recorder.acks[-1]["accepted"] is True

    traffic.executor.finish(RESULT_CANCELED)

    by_id = {r["commandId"]: r["outcome"] for r in stack.recorder.results}
    assert by_id[NAV_ID] == cc.OUTCOME_CANCELED
    assert by_id["22222222-2222-2222-2222-222222222222"] == cc.OUTCOME_SUCCEEDED


def test_cancel_unknown_target_rejected_in_active_mode():
    _, stack = _active_stack()
    stack.send(cancel(NAV_ID))
    ack = stack.recorder.acks[-1]
    assert ack["accepted"] is False and ack["reasonCode"] == cc.REASON_UNKNOWN_TARGET
