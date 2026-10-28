import pytest
from traffic_fakes import (
    ALL_EDGES,
    GRAPH_HASH,
    MAP_CODE,
    Harness,
    make_authority,
    make_envelope,
)
from warotrans_fleet import command_contract as cc
from warotrans_fleet.authority_store import compute_path_hash
from warotrans_fleet.segment_executor import RESULT_ABORTED, RESULT_CANCELED, RESULT_SUCCEEDED, RESULT_TIMEOUT
from warotrans_fleet.traffic_client import (
    STATE_EXECUTING,
    STATE_FAULTED,
    STATE_IDLE,
    STATE_STOPPED_AWAITING,
    STATE_STOPPING,
)


def _extend_env(edges, *, seq, authority_id="auth-1", target="cmd-1", path_hash=None, message_id=None, epoch=1):
    return make_envelope(
        command_id="cmd-ext",
        message_id=message_id or f"ext-{seq}",
        command_type=cc.EXTEND_AUTHORITY,
        payload={
            "targetCommandId": target,
            "authorityId": authority_id,
            "epoch": epoch,
            "seq": seq,
            "segmentEdgeIds": list(edges),
            "graphHash": GRAPH_HASH,
            "pathHash": path_hash or compute_path_hash(GRAPH_HASH, edges),
        },
    )


def _renew_env(*, seq, expires_at, authority_id="auth-1", target="cmd-1"):
    return make_envelope(
        command_id="cmd-renew",
        message_id=f"renew-{seq}",
        command_type=cc.RENEW_AUTHORITY,
        payload={
            "targetCommandId": target,
            "authorityId": authority_id,
            "epoch": 1,
            "seq": seq,
            "expiresAt": expires_at,
        },
    )


def _revoke_env(*, seq, mode="STOP_NOW", authority_id="auth-1", target="cmd-1"):
    return make_envelope(
        command_id="cmd-revoke",
        message_id=f"revoke-{seq}",
        command_type=cc.REVOKE_AUTHORITY,
        payload={"targetCommandId": target, "authorityId": authority_id, "epoch": 1, "seq": seq, "mode": mode},
    )


# ----------------------------------------------------------------------------- start


def test_start_ok_opens_only_granted_edges_and_emits_started():
    h = Harness()
    decision = h.start()
    assert decision.accepted, decision
    assert h.client.state == STATE_EXECUTING
    assert h.route_port.last_open == [1, 2]
    assert h.route_port.last_close == [3, 4, 5]
    assert h.executor.follow_calls == [(1, 2)]
    assert h.event_names() == ["STARTED"]
    first = h.events[0]
    assert first["authorityId"] == "auth-1" and first["commandId"] == "cmd-1" and first["epoch"] == 1
    assert first["progressSeq"] == 1 and first["poseX"] == 0.0
    assert h.client.authority_valid() is True
    assert h.client.next_edge_id() == 1


@pytest.mark.parametrize(
    "authority,reason",
    [
        (make_authority(map_code="OTHER"), cc.REASON_MAP_VERSION_MISMATCH),
        (make_authority(map_code=None), cc.REASON_MAP_VERSION_MISMATCH),
        (make_authority(graph_hash="other-graph"), cc.REASON_ROUTE_GRAPH_MISMATCH),
        (make_authority(edges=()), cc.REASON_ROUTE_NOT_AUTHORIZED),
        (make_authority(command_id="someone-else"), cc.REASON_AUTHORITY_MISMATCH),
    ],
)
def test_start_rejections(authority, reason):
    h = Harness()
    decision = h.start(authority)
    assert (decision.accepted, decision.reason_code) == (False, reason)
    assert h.client.state == STATE_IDLE and h.events == [] and h.executor.follow_calls == []


def test_start_rejects_path_hash_mismatch():
    from dataclasses import replace

    h = Harness()
    bad = replace(make_authority(), path_hash="0" * 64)
    assert h.start(bad).reason_code == cc.REASON_PATH_HASH_MISMATCH
    assert h.client.state == STATE_IDLE


def test_start_rejects_without_pose():
    h = Harness(with_pose=False)
    assert h.start().reason_code == cc.REASON_NOT_LOCALIZED
    assert h.store.last_seq == -1  # chưa đốt seq


def test_start_soft_continues_when_adjust_edges_fails():
    # DynamicEdges often times out under SingleThreadedExecutor; corridor still opens via FollowPath.
    h = Harness(route_ok=False)
    assert h.start().accepted
    assert h.client.state == STATE_EXECUTING and h.store.active is not None
    assert len(h.executor.follow_calls) == 1


def test_start_rejects_when_compute_route_differs_from_segment():
    h = Harness(segment_goal=lambda a: (5.0, 5.0, 0.0))
    h.executor.default_route = [1, 3]
    assert h.start().reason_code == cc.REASON_ROUTE_NOT_AUTHORIZED
    assert h.client.state == STATE_IDLE and h.executor.follow_calls == []

    h2 = Harness(segment_goal=lambda a: (5.0, 5.0, 0.0))
    h2.executor.default_route = [1, 2]
    assert h2.start().accepted
    assert h2.executor.compute_calls == [((0.0, 0.0, 0.0), (5.0, 5.0, 0.0))]


def test_start_follow_failure_is_controller_failed_and_idle():
    h = Harness()
    h.executor.fail_follow = True
    decision = h.start()
    assert decision.reason_code == cc.REASON_CONTROLLER_FAILED
    assert h.client.state == STATE_IDLE and h.store.active is None


def test_start_busy_and_stale_seq():
    h = Harness()
    assert h.start().accepted
    assert h.start(make_authority(seq=2), command_id="cmd-2", message_id="m2").reason_code == cc.REASON_BUSY

    h.client.cancel_command("cmd-1")
    assert h.client.state == STATE_IDLE
    stale = h.start(make_authority(seq=1), command_id="cmd-3", message_id="m3")
    assert stale.reason_code == cc.REASON_AUTHORITY_STALE_SEQ
    assert h.start(make_authority(seq=2), command_id="cmd-3", message_id="m3").accepted


def test_start_duplicate_message_rejected():
    h = Harness()
    assert h.start().accepted
    h.client.cancel_command("cmd-1")
    assert h.start(make_authority(seq=9), command_id="cmd-1", message_id="msg-1").reason_code == (
        cc.REASON_AUTHORITY_DUPLICATE
    )


def test_start_expired_authority_rejected():
    h = Harness()
    assert h.start(make_authority(expires_at=h.clock.now - 1)).reason_code == cc.REASON_AUTHORITY_EXPIRED


# ----------------------------------------------------------------------------- progress / safe stop


def test_edge_events_and_safe_stop_never_completes_command():
    h = Harness()
    h.start()
    h.executor.enter_edge(1)
    h.executor.exit_edge(1)
    h.executor.enter_edge(2)
    h.executor.exit_edge(2)
    h.executor.finish(RESULT_SUCCEEDED)
    assert h.event_names() == [
        "STARTED",
        "EDGE_ENTERED",
        "EDGE_EXITED",
        "EDGE_ENTERED",
        "EDGE_EXITED",
        "SAFE_STOP_REACHED",
        "STOPPED_AWAITING_AUTHORITY",
    ]
    assert [e["edgeId"] for e in h.events if e["event"].startswith("EDGE_")] == [1, 1, 2, 2]
    assert h.client.state == STATE_STOPPED_AWAITING
    assert h.client.authority_valid() is False  # đứng chờ ≠ được chạy
    assert h.results == []  # không bao giờ báo SUCCEEDED/release từ việc dừng
    assert [e["progressSeq"] for e in h.events] == list(range(1, 8))


def test_entering_ungranted_edge_faults_corridor_violation():
    h = Harness()
    h.start()
    h.executor.enter_edge(4)
    assert h.client.state == STATE_FAULTED
    assert h.client.last_fault == cc.ERROR_CORRIDOR_VIOLATION
    assert h.events[-1]["event"] == "FAULT" and h.events[-1]["faultCode"] == "CORRIDOR_VIOLATION"
    assert h.executor.cancel_calls == 1
    assert h.results == [("cmd-1", cc.OUTCOME_FAILED, "CORRIDOR_VIOLATION")]


@pytest.mark.parametrize("status", [RESULT_ABORTED, RESULT_TIMEOUT, RESULT_CANCELED])
def test_unexpected_executor_result_faults(status):
    h = Harness()
    h.start()
    h.executor.finish(status)
    assert h.client.state == STATE_FAULTED
    assert h.client.last_fault == cc.ERROR_CONTROLLER_FAILED


# ----------------------------------------------------------------------------- extend


def test_extend_while_stopped_resumes_remaining_edges_only():
    h = Harness()
    h.start()
    h.executor.finish(RESULT_SUCCEEDED)
    assert h.client.state == STATE_STOPPED_AWAITING

    decision = h.client.extend_authority(_extend_env((1, 2, 3), seq=2))
    assert decision.accepted, decision
    assert h.client.state == STATE_EXECUTING
    assert h.executor.follow_calls[-1] == (3,)
    assert h.route_port.last_open == [1, 2, 3] and h.route_port.last_close == [4, 5]
    assert h.client.granted_edge_ids() == (1, 2, 3)
    assert h.client.authority_valid()


def test_extend_while_executing_continues_without_safe_stop_event():
    h = Harness()
    h.start()
    assert h.client.extend_authority(_extend_env((1, 2, 3), seq=2)).accepted
    assert h.executor.follow_calls == [(1, 2)]  # chưa đổi ticket đang chạy
    h.executor.finish(RESULT_SUCCEEDED)
    assert h.executor.follow_calls[-1] == (3,)
    assert h.client.state == STATE_EXECUTING
    assert "SAFE_STOP_REACHED" not in h.event_names()
    h.executor.finish(RESULT_SUCCEEDED)
    assert h.client.state == STATE_STOPPED_AWAITING


@pytest.mark.parametrize(
    "env,reason",
    [
        (_extend_env((1, 2, 3), seq=2, target="other"), cc.REASON_AUTHORITY_MISMATCH),
        (_extend_env((1, 2, 3), seq=2, authority_id="other"), cc.REASON_AUTHORITY_MISMATCH),
        (_extend_env((1, 2, 3), seq=2, path_hash="0" * 64), cc.REASON_PATH_HASH_MISMATCH),
        (_extend_env((2, 3, 4), seq=2), cc.REASON_ROUTE_NOT_AUTHORIZED),  # không bắt đầu bằng segment cũ
        (_extend_env((1, 2), seq=2), cc.REASON_ROUTE_NOT_AUTHORIZED),  # không dài hơn
        (_extend_env((1, 2, 3), seq=1, message_id="x"), cc.REASON_AUTHORITY_STALE_SEQ),
        (_extend_env((1, 2, 3), seq=2, epoch=0), cc.REASON_AUTHORITY_STALE_EPOCH),
    ],
)
def test_extend_rejections_do_not_widen_authority(env, reason):
    h = Harness()
    h.start()
    decision = h.client.extend_authority(env)
    assert (decision.accepted, decision.reason_code) == (False, reason)
    assert h.client.granted_edge_ids() == (1, 2)
    assert h.client.state == STATE_EXECUTING


def test_extend_without_active_authority_rejected():
    h = Harness()
    assert h.client.extend_authority(_extend_env((1, 2, 3), seq=2)).reason_code == cc.REASON_AUTHORITY_REQUIRED


def test_extend_survives_dynamic_edges_mirror_failure_and_burns_seq():
    # Active Mode drives FollowPath on the granted polylines; DynamicEdges only mirrors the corridor into
    # route_server, so its failure must not refuse an otherwise valid extension.
    h = Harness()
    h.start()
    h.route_port.adjust_ok = False
    assert h.client.extend_authority(_extend_env((1, 2, 3), seq=2)).accepted
    assert h.client.granted_edge_ids() == (1, 2, 3)
    assert h.client.extend_authority(_extend_env((1, 2, 3, 4), seq=2, message_id="again")).reason_code == (
        cc.REASON_AUTHORITY_STALE_SEQ
    )
    assert h.client.extend_authority(_extend_env((1, 2, 3, 4), seq=3)).accepted


# ----------------------------------------------------------------------------- renew


def test_renew_extends_expiry():
    h = Harness()
    h.start(make_authority(expires_at=h.clock.now + 5))
    h.clock.advance(4)
    assert h.client.authority_valid()
    assert h.client.renew_authority(_renew_env(seq=2, expires_at=h.clock.now + 10)).accepted
    h.clock.advance(6)
    assert h.client.authority_valid()  # sẽ đã hết hạn nếu không renew
    h.clock.advance(10)
    assert h.client.authority_valid() is False


def test_renew_rejections():
    h = Harness()
    h.start()
    assert h.client.renew_authority(_renew_env(seq=2, expires_at=h.clock.now - 1)).reason_code == (
        cc.REASON_AUTHORITY_EXPIRED
    )
    assert h.client.renew_authority(_renew_env(seq=3, expires_at=h.clock.now + 5, authority_id="x")).reason_code == (
        cc.REASON_AUTHORITY_MISMATCH
    )
    missing = _renew_env(seq=4, expires_at=None)
    assert h.client.renew_authority(missing).reason_code == cc.REASON_INVALID_PAYLOAD


def test_renew_refreshes_last_authority_at():
    h = Harness()
    h.start()
    before = h.client.last_authority_at
    h.clock.advance(2)
    assert h.client.renew_authority(_renew_env(seq=2, expires_at=h.clock.now + 30)).accepted
    assert h.client.last_authority_at == before + 2


# ----------------------------------------------------------------------------- revoke / cancel


def test_revoke_stop_now_stops_and_waits_for_authority():
    h = Harness()
    h.start()
    assert h.client.revoke_authority(_revoke_env(seq=2)).accepted
    assert h.executor.cancel_calls == 1
    assert h.client.state == STATE_STOPPED_AWAITING
    assert h.client.authority_valid() is False
    assert h.event_names()[-1] == "STOPPED_AWAITING_AUTHORITY"
    assert h.results == []
    # đã revoke → không extend/renew được
    assert h.client.extend_authority(_extend_env((1, 2, 3), seq=3)).reason_code == cc.REASON_AUTHORITY_LOST
    assert h.client.renew_authority(_renew_env(seq=4, expires_at=h.clock.now + 5)).reason_code == (
        cc.REASON_AUTHORITY_LOST
    )


def test_revoke_stop_now_waits_for_executor_confirmation():
    h = Harness()
    h.executor.auto_cancel_result = False
    h.start()
    assert h.client.revoke_authority(_revoke_env(seq=2)).accepted
    assert h.client.state == STATE_STOPPING
    assert h.client.authority_valid() is False  # motion bị tước ngay, kể cả khi chưa dừng hẳn
    h.executor.finish(RESULT_CANCELED)
    assert h.client.state == STATE_STOPPED_AWAITING
    assert h.event_names()[-1] == "STOPPED_AWAITING_AUTHORITY"


def test_revoke_at_safe_stop_lets_segment_finish_without_continuing():
    h = Harness()
    h.start()
    assert h.client.revoke_authority(_revoke_env(seq=2, mode="STOP_AT_SAFE_STOP")).accepted
    assert h.client.state == STATE_EXECUTING and h.executor.cancel_calls == 0
    assert h.client.extend_authority(_extend_env((1, 2, 3), seq=3)).reason_code == cc.REASON_AUTHORITY_LOST
    h.executor.finish(RESULT_SUCCEEDED)
    assert h.client.state == STATE_STOPPED_AWAITING
    assert h.executor.follow_calls == [(1, 2)]


def test_revoke_invalid_mode_and_mismatch():
    h = Harness()
    h.start()
    assert h.client.revoke_authority(_revoke_env(seq=2, mode="PANIC")).reason_code == cc.REASON_INVALID_PAYLOAD
    assert h.client.revoke_authority(_revoke_env(seq=2, authority_id="x")).reason_code == cc.REASON_AUTHORITY_MISMATCH
    assert h.client.state == STATE_EXECUTING


def test_cancel_command_reports_canceled_and_goes_idle():
    h = Harness()
    h.start()
    assert h.client.cancel_command("nope").accepted is False
    assert h.client.cancel_command("cmd-1").accepted
    assert h.client.state == STATE_IDLE
    assert h.results == [("cmd-1", cc.OUTCOME_CANCELED, None)]
    assert h.store.active is None


# ----------------------------------------------------------------------------- fault


def test_fault_is_latched_no_auto_resume_until_synced_higher_seq():
    h = Harness()
    h.start()
    h.client.fault(cc.ERROR_SAFETY_STOP, "test")
    assert h.client.state == STATE_FAULTED
    assert h.client.authority_valid() is False

    # idempotent, không phát FAULT lần hai
    h.client.fault(cc.ERROR_SAFETY_STOP)
    assert h.event_names().count("FAULT") == 1

    # extend/renew không cứu được FAULTED
    assert h.client.extend_authority(_extend_env((1, 2, 3), seq=2)).reason_code == cc.REASON_AUTHORITY_LOST
    assert h.client.renew_authority(_renew_env(seq=3, expires_at=h.clock.now + 5)).reason_code == (
        cc.REASON_AUTHORITY_LOST
    )
    # chưa sync → chưa nhận authority mới, dù seq cao hơn
    assert h.start(make_authority(seq=5), command_id="cmd-2", message_id="m2").reason_code == (
        cc.REASON_AUTHORITY_REQUIRED
    )
    assert h.client.state == STATE_FAULTED

    h.client.mark_synced()
    # sau sync: seq cũ/bằng bị từ chối
    assert h.start(make_authority(seq=1), command_id="cmd-2", message_id="m3").reason_code == (
        cc.REASON_AUTHORITY_STALE_SEQ
    )
    assert h.client.state == STATE_FAULTED
    # seq cao hơn → chạy lại
    assert h.start(make_authority(seq=5), command_id="cmd-2", message_id="m4").accepted
    assert h.client.state == STATE_EXECUTING


def test_stale_executor_callbacks_ignored_after_fault():
    h = Harness()
    h.start()
    ticket_callbacks = h.executor._callbacks
    h.client.fault(cc.ERROR_AUTHORITY_LOST)
    count = len(h.events)
    ticket_callbacks.on_result(__import__("warotrans_fleet.segment_executor", fromlist=["SegmentResult"]).SegmentResult(RESULT_SUCCEEDED))
    ticket_callbacks.on_edge_entered(1)
    assert len(h.events) == count and h.client.state == STATE_FAULTED


def test_progress_publisher_exception_does_not_break_client():
    h = Harness()
    h.client._publish = lambda _p: (_ for _ in ()).throw(RuntimeError("mqtt down"))
    assert h.start().accepted
    assert h.client.state == STATE_EXECUTING


def test_authority_state_report_tracks_state():
    h = Harness()
    report = h.client.authority_state_report()
    assert report["state"] == STATE_IDLE and report["activeAuthorityId"] is None
    h.start()
    report = h.client.authority_state_report()
    assert report["state"] == STATE_EXECUTING and report["grantedEdgeIds"] == [1, 2]
    assert report["lastEpoch"] == 1 and report["lastSeq"] == 1 and report["activeCommandId"] == "cmd-1"


def test_authority_expiry_makes_authority_invalid():
    h = Harness()
    h.start(make_authority(expires_at=h.clock.now + 2))
    assert h.client.authority_valid()
    h.clock.advance(2)
    assert h.client.authority_valid() is False


def test_constants_sanity():
    assert MAP_CODE and ALL_EDGES


# ----------------------------------------------------------------------------- route completion (finalGoal)


def test_final_segment_with_final_goal_succeeds_command():
    from dataclasses import replace

    h = Harness()
    h.start(replace(make_authority(), final_goal=(1.0, 2.0, 0.5)))
    h.executor.finish(RESULT_SUCCEEDED)
    assert h.client.state == STATE_IDLE
    assert h.event_names()[-1] == "SAFE_STOP_REACHED"
    assert "STOPPED_AWAITING_AUTHORITY" not in h.event_names()
    assert h.results == [("cmd-1", cc.OUTCOME_SUCCEEDED, None)]
    assert h.store.active is None


def test_extend_carrying_final_goal_completes_after_last_edges():
    h = Harness()
    h.start()  # segment đầu không có finalGoal
    h.executor.finish(RESULT_SUCCEEDED)
    assert h.client.state == STATE_STOPPED_AWAITING and h.results == []

    env = _extend_env((1, 2, 3), seq=2)
    env.payload["finalGoal"] = {"x": 3.0, "y": 0.0, "yaw": 1.57}
    assert h.client.extend_authority(env).accepted
    assert h.client.authority.final_goal == (3.0, 0.0, 1.57)
    h.executor.finish(RESULT_SUCCEEDED)
    assert h.results == [("cmd-1", cc.OUTCOME_SUCCEEDED, None)]
    assert h.client.state == STATE_IDLE


def test_stop_at_safe_stop_overrides_final_goal():
    from dataclasses import replace

    h = Harness()
    h.start(replace(make_authority(), final_goal=(1.0, 2.0, 0.5)))
    assert h.client.revoke_authority(_revoke_env(seq=2, mode="STOP_AT_SAFE_STOP")).accepted
    h.executor.finish(RESULT_SUCCEEDED)
    assert h.client.state == STATE_STOPPED_AWAITING and h.results == []


def test_fault_then_sync_allows_higher_seq_start():
    h = Harness()
    h.start()
    h.executor.finish(RESULT_ABORTED)
    assert h.client.state == STATE_FAULTED and not h.client.synced
    assert h.start(make_authority(seq=2), command_id="cmd-2", message_id="m2").reason_code == cc.REASON_AUTHORITY_REQUIRED
    h.client.mark_synced()
    assert h.start(make_authority(seq=2), command_id="cmd-3", message_id="m3").accepted
