import hashlib

import pytest
from traffic_fakes import GRAPH_HASH, authority_payload, make_authority
from warotrans_fleet import command_contract as cc
from warotrans_fleet.authority_store import (
    LRU_CAPACITY,
    AuthorityStore,
    compute_path_hash,
    parse_authority,
    parse_expires_at,
)
from warotrans_fleet.command_contract import CommandRejected
from warotrans_fleet.route_grant import RouteGrant

NOW = 1_000.0


def _store(tmp_path=None, **kw):
    path = str(tmp_path / "authority.json") if tmp_path is not None else None
    return AuthorityStore(path, clock=lambda: NOW, persist=tmp_path is not None, **kw)


def test_compute_path_hash_formula():
    expected = hashlib.sha256(b"abc|1,2,3").hexdigest()
    assert compute_path_hash("abc", [1, 2, 3]) == expected
    assert compute_path_hash("abc", []) == hashlib.sha256(b"abc|").hexdigest()


def test_route_grant_and_authority_share_path_hash():
    grant = RouteGrant(graph_hash="abc", edge_ids=(4, 5))
    assert grant.path_hash == make_authority(edges=(4, 5), graph_hash="abc").expected_path_hash()


def test_parse_authority_ok():
    a = parse_authority(authority_payload(expires_at="2026-10-25T01:00:00Z"), command_id="cmd-1")
    assert a.authority_id == "auth-1"
    assert a.segment_edge_ids == (1, 2)
    assert a.final_goal == (1.0, 2.0, 0.5)
    assert a.speed_limits == {"linear": 0.1}
    assert a.goal_safe_stop_node_id == 7
    assert a.route_version == 2
    assert a.command_id == "cmd-1"
    assert a.expires_at == parse_expires_at("2026-10-25T01:00:00+00:00")


def test_parse_authority_missing_is_required():
    with pytest.raises(CommandRejected) as exc:
        parse_authority({})
    assert exc.value.reason_code == cc.REASON_AUTHORITY_REQUIRED


def test_parse_authority_path_hash_mismatch():
    with pytest.raises(CommandRejected) as exc:
        parse_authority(authority_payload(path_hash="0" * 64))
    assert exc.value.reason_code == cc.REASON_PATH_HASH_MISMATCH


@pytest.mark.parametrize("field,value", [("epoch", "x"), ("seq", True), ("segmentEdgeIds", "1,2")])
def test_parse_authority_invalid_types(field, value):
    payload = authority_payload()
    payload["authority"][field] = value
    with pytest.raises(CommandRejected) as exc:
        parse_authority(payload)
    assert exc.value.reason_code == cc.REASON_INVALID_PAYLOAD


def test_accept_then_stale_and_duplicate():
    store = _store()
    assert store.accept(make_authority(epoch=1, seq=5), command_id="c", message_id="m1").accepted
    assert store.last_epoch == 1 and store.last_seq == 5

    dup = store.accept(make_authority(epoch=1, seq=6), command_id="c", message_id="m1")
    assert dup.reason_code == cc.REASON_AUTHORITY_DUPLICATE

    stale_seq = store.accept(make_authority(epoch=1, seq=5), command_id="c", message_id="m2")
    assert stale_seq.reason_code == cc.REASON_AUTHORITY_STALE_SEQ
    lower_seq = store.accept(make_authority(epoch=1, seq=4), command_id="c", message_id="m3")
    assert lower_seq.reason_code == cc.REASON_AUTHORITY_STALE_SEQ

    stale_epoch = store.accept(make_authority(epoch=0, seq=99), command_id="c", message_id="m4")
    assert stale_epoch.reason_code == cc.REASON_AUTHORITY_STALE_EPOCH

    newer_epoch = store.accept(make_authority(epoch=2, seq=0), command_id="c", message_id="m5")
    assert newer_epoch.accepted and (store.last_epoch, store.last_seq) == (2, 0)


def test_accept_expired():
    store = _store()
    decision = store.accept(make_authority(expires_at=NOW - 1), command_id="c", message_id="m")
    assert decision.reason_code == cc.REASON_AUTHORITY_EXPIRED
    assert store.active is None and store.last_seq == -1  # không đốt seq


def test_rejected_authority_not_recorded_in_lru():
    store = _store()
    store.accept(make_authority(seq=2), command_id="c", message_id="m1")
    store.accept(make_authority(seq=1), command_id="c", message_id="m2")  # stale
    assert store.seen_count == 1


def test_lru_capacity_evicts_oldest():
    store = _store()
    for i in range(LRU_CAPACITY + 10):
        assert store.accept(make_authority(seq=i), command_id="c", message_id=f"m{i}").accepted
    assert store.seen_count == LRU_CAPACITY
    # message m0 đã bị evict → không còn là DUPLICATE mà là STALE_SEQ (seq cũ)
    assert store.accept(make_authority(seq=0), command_id="c", message_id="m0").reason_code == cc.REASON_AUTHORITY_STALE_SEQ
    # message gần nhất vẫn là DUPLICATE
    last = LRU_CAPACITY + 9
    assert store.accept(make_authority(seq=last + 1), command_id="c", message_id=f"m{last}").reason_code == (
        cc.REASON_AUTHORITY_DUPLICATE
    )


def test_persist_and_load_clear_motion_on_restart(tmp_path):
    store = _store(tmp_path)
    store.accept(make_authority(epoch=3, seq=7, command_id="cmd-1"), command_id="cmd-1", message_id="m1")
    assert store.motion_resumable is True

    reloaded = _store(tmp_path)
    assert (reloaded.last_epoch, reloaded.last_seq) == (3, 7)
    assert reloaded.active is not None and reloaded.active.authority_id == "auth-1"
    assert reloaded.active.segment_edge_ids == (1, 2)
    assert reloaded.motion_resumable is False  # chỉ để báo cáo
    report = reloaded.report(state="IDLE")
    assert report["activeAuthorityId"] == "auth-1" and report["grantedEdgeIds"] == [1, 2]
    assert report["motionResumable"] is False
    # LRU được persist: redelivery của message cũ vẫn là DUPLICATE
    assert reloaded.accept(make_authority(epoch=3, seq=8), command_id="cmd-1", message_id="m1").reason_code == (
        cc.REASON_AUTHORITY_DUPLICATE
    )
    # seq cao hơn mới cho phép chạy lại
    assert reloaded.accept(make_authority(epoch=3, seq=8), command_id="cmd-1", message_id="m2").accepted
    assert reloaded.motion_resumable is True


def test_restart_keeps_motion_when_flag_off(tmp_path):
    _store(tmp_path).accept(make_authority(), command_id="c", message_id="m")
    reloaded = AuthorityStore(str(tmp_path / "authority.json"), clock=lambda: NOW, clear_motion_on_restart=False)
    assert reloaded.motion_resumable is True


def test_corrupt_state_file_starts_clean(tmp_path):
    path = tmp_path / "authority.json"
    path.write_text("{not json", encoding="utf-8")
    store = AuthorityStore(str(path), clock=lambda: NOW)
    assert store.active is None and store.last_seq == -1


def test_clear_active_keeps_sequence(tmp_path):
    store = _store(tmp_path)
    store.accept(make_authority(seq=4), command_id="c", message_id="m")
    store.clear_active()
    assert store.active is None and store.last_seq == 4
    assert _store(tmp_path).active is None


def test_default_path_is_temp_file():
    from warotrans_fleet.authority_store import default_state_path

    assert default_state_path().endswith("warotrans_authority_state.json")


def test_reason_codes_match_backend_strings():
    assert cc.REASON_AUTHORITY_STALE_EPOCH == "AUTHORITY_STALE_EPOCH"
    assert cc.REASON_AUTHORITY_STALE_SEQ == "AUTHORITY_STALE_SEQ"
    assert cc.REASON_AUTHORITY_DUPLICATE == "AUTHORITY_DUPLICATE"
    assert cc.REASON_AUTHORITY_EXPIRED == "AUTHORITY_EXPIRED"
    assert cc.REASON_SAFETY_STOP == "SAFETY_STOP"
    assert (cc.NAVIGATE_TO_POSE, cc.CANCEL) == ("NAVIGATE_TO_POSE", "CANCEL")
    assert cc.EXTEND_AUTHORITY == "EXTEND_AUTHORITY"
    assert cc.RENEW_AUTHORITY == "RENEW_AUTHORITY"
    assert cc.REVOKE_AUTHORITY == "REVOKE_AUTHORITY"
    assert GRAPH_HASH  # fixture sanity
