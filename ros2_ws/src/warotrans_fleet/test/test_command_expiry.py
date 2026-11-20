"""TRN-03: a command that arrives after its expiresAt is not run, and a CANCEL never cancels another job's MOVE."""

from datetime import datetime, timedelta, timezone

import pytest
from fleet_fakes import ROBOT, Harness, cancel, navigate

from warotrans_fleet.command_contract import (
    CommandRejected,
    is_expired,
    parse_envelope,
    parse_timestamp,
)
from warotrans_fleet.navigator import STATUS_CANCELED

CMD = "11111111-1111-1111-1111-111111111111"
CANCEL_CMD = "22222222-2222-2222-2222-222222222222"
JOB = "aaaaaaaa-0000-0000-0000-000000000001"  # navigate()'s jobAssignmentId
OTHER_JOB = "aaaaaaaa-0000-0000-0000-000000000099"


def _iso(delta_sec: float) -> str:
    return (datetime.now(timezone.utc) + timedelta(seconds=delta_sec)).isoformat().replace("+00:00", "Z")


def _with(body: dict, **fields) -> dict:
    copy = dict(body)
    copy.update(fields)
    return copy


def test_dotnet_timestamps_with_seven_fraction_digits_parse_as_utc():
    parsed = parse_timestamp("2026-11-21T08:00:30.1234567Z")
    assert parsed == datetime(2026, 11, 21, 8, 0, 30, 123456, tzinfo=timezone.utc)
    assert parse_timestamp("2026-11-21T08:00:30+00:00").tzinfo is not None
    assert parse_timestamp("2026-11-21T08:00:30").tzinfo == timezone.utc


def test_an_unreadable_expiry_is_an_invalid_payload():
    with pytest.raises(CommandRejected) as rejected:
        parse_envelope(_with(navigate(CMD), expiresAt="tomorrow"), expected_robot_code=ROBOT)
    assert rejected.value.reason_code == "INVALID_PAYLOAD"


def test_expiry_allows_clock_skew_and_only_applies_to_navigate_cancel_localize():
    now = datetime(2026, 11, 21, 8, 0, 0, tzinfo=timezone.utc)
    late = parse_envelope(_with(navigate(CMD), expiresAt="2026-11-21T07:59:59Z"), expected_robot_code=ROBOT)
    assert not is_expired(late, now, skew_sec=2.0)  # 1 s late, within the skew
    assert is_expired(late, now + timedelta(seconds=5), skew_sec=2.0)
    assert not is_expired(late, now + timedelta(hours=1), skew_sec=-1.0)  # check turned off

    stop = dict(navigate(CMD), type="STOP", payload={"mode": "PAUSE"}, expiresAt="2026-11-21T07:00:00Z")
    assert not is_expired(parse_envelope(stop, expected_robot_code=ROBOT), now)


def test_an_expired_navigate_is_refused_and_no_goal_is_sent():
    h = Harness()
    h.send(_with(navigate(CMD), expiresAt=_iso(-60)))

    ack = h.recorder.acks[-1]
    assert ack["accepted"] is False and ack["reasonCode"] == "COMMAND_EXPIRED"
    assert h.nav.goals == []


def test_a_navigate_within_its_expiry_or_without_one_runs_as_before():
    h = Harness()
    h.send(_with(navigate(CMD), expiresAt=_iso(60)))
    assert h.recorder.acks[-1]["accepted"] is True

    legacy = Harness()
    legacy.send(navigate(CMD))  # an older backend sends no expiresAt
    assert legacy.recorder.acks[-1]["accepted"] is True


def test_a_late_cancel_does_not_cancel_the_move_now_running():
    h = Harness()
    h.send(navigate(CMD))
    h.nav.accept()

    h.send(_with(cancel(CMD, CANCEL_CMD), expiresAt=_iso(-30)))

    ack = h.recorder.acks[-1]
    assert ack["commandId"] == CANCEL_CMD and ack["accepted"] is False and ack["reasonCode"] == "COMMAND_EXPIRED"
    assert h.nav.cancel_requests == []


def test_a_cancel_for_another_job_is_refused():
    h = Harness()
    h.send(navigate(CMD))
    h.nav.accept()

    h.send(_with(cancel(CMD, CANCEL_CMD), jobAssignmentId=OTHER_JOB))

    ack = h.recorder.acks[-1]
    assert ack["accepted"] is False and ack["reasonCode"] == "JOB_MISMATCH"
    assert h.nav.cancel_requests == []


def test_a_cancel_for_the_running_job_still_cancels_it():
    h = Harness()
    h.send(navigate(CMD))
    h.nav.accept()

    h.send(_with(cancel(CMD, CANCEL_CMD), jobAssignmentId=JOB, expiresAt=_iso(30)))
    assert h.recorder.acks[-1]["accepted"] is True
    h.nav.answer_cancel(True)
    h.nav.finish(STATUS_CANCELED)

    nav_result, cancel_result = h.recorder.results
    assert nav_result["outcome"] == "CANCELED"
    assert cancel_result["outcome"] == "SUCCEEDED"
