"""Spool, uploader, sampling and recording decisions, without ROS and without a network."""

from __future__ import annotations

import hashlib
import math
import os

import pytest

from warotrans_evidence_agent.agent_core import AgentCore, CameraProfile
from warotrans_evidence_agent.recorder_api import ApiError
from warotrans_evidence_agent.sampling import TrajectorySampler, decimate, parse_chrony_tracking, path_hash
from warotrans_evidence_agent.spool import Spool
from warotrans_evidence_agent.uploader import Uploader, iso, segment_intent

NS = 1_000_000_000


def segment(tmp_path, key: str, seq: int, start_s: float = 0.0, gap: str | None = None) -> dict:
    path = tmp_path / f"{key}-{seq}.avi"
    body = os.urandom(1000 + seq)
    path.write_bytes(body)
    t0 = int(1_763_348_400 * NS + start_s * NS)
    return {"segment_id": f"00000000-0000-0000-0000-{seq:012d}", "recording_key": key, "sequence_number": seq,
            "boot_session_id": "11111111-1111-1111-1111-111111111111", "path": str(path),
            "sidecar_path": str(path) + ".timing.json", "captured_from_utc_ns": t0, "captured_to_utc_ns": t0 + 30 * NS,
            "start_monotonic_ns": 5 * NS + int(start_s * NS), "end_monotonic_ns": 35 * NS + int(start_s * NS),
            "duration_ms": 30000, "frame_count": 450, "dropped_frame_count": 0, "duplicated_frame_count": 1,
            "max_slot_error_ms": 9.5, "byte_size": len(body), "sha256": hashlib.sha256(body).hexdigest(),
            "width": 1280, "height": 720, "fps": 15, "container_format": "AVI", "codec": "MJPEG", "gap_before": gap}


class FakeApi:
    def __init__(self) -> None:
        self.run_known = True
        self.calls: list[tuple] = []
        self.registered: list[str] = []
        self.fail_upload_grants = 0

    def start_recording(self, body):
        self.calls.append(("start", body["edgeRecordingKey"]))
        if not self.run_known:
            raise ApiError(409, {"code": "no_run_yet"}, "start")
        return {"recordingId": "rec-1", "runId": "run-1"}

    def register_segments(self, recording_id, segments):
        self.calls.append(("register", [s["sequenceNumber"] for s in segments]))
        self.registered += [s["segmentId"] for s in segments]
        return {"segments": [{"segmentId": s["segmentId"]} for s in segments]}

    def upload_grant(self, segment_id):
        if self.fail_upload_grants:
            self.fail_upload_grants -= 1
            raise ApiError(0, "network down", "grant")
        return {"url": f"http://store/{segment_id}", "method": "PUT", "headers": {}}

    def finalize(self, recording_id, body):
        self.calls.append(("finalize", body["expectedSegmentCount"], len(body["gaps"])))
        return {}

    def trajectory(self, run_id, points):
        self.calls.append(("trajectory", run_id, len(points)))
        return {}

    def events(self, run_id, events):
        self.calls.append(("events", run_id, len(events)))
        return {}

    def plans(self, run_id, plans):
        self.calls.append(("plans", run_id, plans[0]["planVersion"]))
        return {}

    def clock_anchors(self, anchors):
        self.calls.append(("anchors", len(anchors)))
        return {}


class Clock:
    def __init__(self) -> None:
        self.t = 0.0

    def __call__(self) -> float:
        return self.t


@pytest.fixture
def spool(tmp_path):
    s = Spool(str(tmp_path / "spool" / "evidence.db"))
    yield s
    s.close()


PROFILE = {"cameraId": "front", "width": 1280, "height": 720, "fps": 15}


def test_segment_intent_is_what_the_recorder_expects(tmp_path):
    body = segment_intent(segment(tmp_path, "k", 3))
    assert body["sequenceNumber"] == 3 and body["frameCount"] == 450 and len(body["sha256"]) == 64
    assert body["capturedFromUtc"].endswith("Z") and body["capturedFromUtc"].startswith("2025-11-17T03:00:00")


def test_a_recording_waits_for_its_run_then_registers_uploads_and_finalizes(tmp_path, spool):
    api, puts = FakeApi(), []
    up = Uploader(spool, api, put=lambda grant, path, bps: puts.append(path), delete_uploaded=False, settle_s=0)
    spool.add_recording("k", 1_763_348_400 * NS, "boot", "step-1", str(tmp_path), PROFILE)
    for i in range(3):
        spool.add_segment(segment(tmp_path, "k", i, 30 * i))
    spool.add_segment(segment(tmp_path, "k", 1, 30))  # reported twice: stored once

    api.run_known = False
    assert up.pump()["uploaded"] == 0  # 409 no_run_yet: nothing sent, asked again later
    api.run_known = True
    spool._q("UPDATE recordings SET next_attempt_at = 0")
    spool.end_recording("k", 1_763_348_490 * NS)
    stats = up.pump(max_uploads=10)
    assert stats["registered"] == 3 and stats["uploaded"] == 3 and stats["finalized"] == 1
    assert ("register", [0, 1, 2]) in api.calls and ("finalize", 3, 0) in api.calls
    assert len(puts) == 3
    assert spool.pending_upload_count() == 0
    assert up.pump() == {"bound": 0, "registered": 0, "uploaded": 0, "finalized": 0, "batches": 0}


def test_a_network_failure_retries_later_and_nothing_is_lost(tmp_path, spool):
    api = FakeApi()
    api.fail_upload_grants = 1
    clock_puts = []
    up = Uploader(spool, api, put=lambda g, p, b: clock_puts.append(p), delete_uploaded=True, settle_s=0)
    spool.add_recording("k", 0, "boot", None, str(tmp_path), PROFILE)
    info = segment(tmp_path, "k", 0)
    spool.add_segment(info)
    up.pump()
    assert spool.pending_upload_count() == 1 and os.path.exists(info["path"])
    spool._q("UPDATE segments SET next_attempt_at = 0")
    up.pump()
    assert spool.pending_upload_count() == 0
    # Uploaded but the recording is not finalized yet: the file is kept.
    assert os.path.exists(info["path"])
    spool.end_recording("k", 30 * NS)
    up.pump()
    assert not os.path.exists(info["path"]), "deleted only after upload confirmed and the recording finalized"


def test_a_segment_already_stored_counts_as_uploaded(tmp_path, spool):
    api = FakeApi()

    def put(grant, path, bps):
        raise ApiError(412, "exists", "PUT")

    up = Uploader(spool, api, put=put, delete_uploaded=False)
    spool.add_recording("k", 0, "boot", None, str(tmp_path), PROFILE)
    spool.add_segment(segment(tmp_path, "k", 0))
    assert up.pump()["uploaded"] == 1


def test_gaps_reported_by_the_camera_reach_finalize(tmp_path, spool):
    api = FakeApi()
    up = Uploader(spool, api, put=lambda *a: None, delete_uploaded=False, settle_s=0)
    spool.add_recording("k", 0, "boot", None, str(tmp_path), PROFILE)
    spool.add_segment(segment(tmp_path, "k", 0, 0))
    spool.add_segment(segment(tmp_path, "k", 1, 45, gap="CAMERA_LOST"))
    spool.end_recording("k", 80 * NS)
    up.pump()
    assert ("finalize", 2, 1) in api.calls


def test_telemetry_batches_are_sent_and_a_refused_one_does_not_block_the_rest(spool):
    api = FakeApi()
    calls = []

    def trajectory(run_id, points):
        calls.append(len(points))
        if len(points) == 1:
            raise ApiError(400, "bad", "trajectory")
        return {}

    api.trajectory = trajectory
    up = Uploader(spool, api, put=lambda *a: None)
    spool.enqueue("trajectory", "run-1", [{"x": 1}])
    spool.enqueue("trajectory", "run-1", [{"x": 1}, {"x": 2}])
    spool.enqueue("anchors", None, [{"monotonicNs": 1}])
    assert up.pump()["batches"] == 3
    assert calls == [1, 2] and spool.outbox_size() == 0


def test_sampler_rates_validity_and_jumps():
    s = TrajectorySampler()
    t = 1_000 * NS
    keep, _ = s.offer(0, 0, 0, 0.3, 0, 0.05, True, t)
    assert keep is not None
    assert s.offer(0.01, 0, 0, 0.3, 0, 0.05, True, t + int(0.1 * NS))[0] is None  # moving: 5 Hz
    assert s.offer(0.02, 0, 0, 0.3, 0, 0.05, True, t + int(0.25 * NS))[0] is not None
    assert s.offer(0.02, 0, 0, 0.0, 0, 0.05, True, t + int(0.6 * NS))[0] is None  # still: 1 Hz
    lost, event = s.offer(0.02, 0, 0, 0.0, 0, 3.0, True, t + int(0.7 * NS))  # stale TF: kept as a gap at once
    assert lost is not None and not lost.valid and event == "localization.lost"
    back, event = s.offer(0.02, 0, 0, 0.0, 0, 0.0, True, t + int(0.8 * NS))
    assert back.valid and event == "localization.regained"
    jumped, event = s.offer(1.5, 0, 0, 0.3, 0, 0.0, True, t + int(1.1 * NS))
    assert event == "localization.jump"
    nan, _ = s.offer(math.nan, 0, 0, 0, 0, 0.0, True, t + int(3 * NS))
    assert not nan.valid and nan.x == 0.0


def test_sampler_keeps_5_hz_with_a_jittery_200_ms_timer():
    """The node offers a pose every 0.2 s; ticks a millisecond early must still be kept (not every other one)."""
    s = TrajectorySampler()
    t = 1_000 * NS
    kept = 0
    for i in range(50):  # 10 s of driving, ticks alternately 1 ms early and on time
        at = t + i * int(0.2 * NS) - (1_000_000 if i % 2 else 0)
        kept += s.offer(i * 0.07, 0, 0, 0.35, 0, 0.02, True, at)[0] is not None
    assert kept == 50


def test_decimate_keeps_the_ends_and_the_bound():
    line = [(i * 0.01, 0.0) for i in range(5001)]
    out = decimate(line, max_points=500)
    assert len(out) <= 500
    assert out[0] == [0.0, 0.0] and out[-1] == [50.0, 0.0]
    assert path_hash(out) == path_hash(decimate(line, max_points=500))


def test_chrony_tracking_bound_and_sync():
    synced = parse_chrony_tracking("C0A88901,192.168.137.1,3,1700000000.1,-0.000500,0.0,0.0,1.0,0.0,0.0,0.004000,0.001000,64.0,Normal")
    assert synced.synchronized and synced.offset_ms == -0.5 and synced.error_bound_ms == pytest.approx(3.5)
    unsynced = parse_chrony_tracking("00000000,,0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,1.0,0.0,Not synchronised")
    assert not unsynced.synchronized
    assert not parse_chrony_tracking("garbage").synchronized


class Mono:
    def __init__(self) -> None:
        self.t = 100.0

    def __call__(self) -> float:
        return self.t


def core(tmp_path, spool, sent, mono, free=10 * 1024 ** 3, quota=20 * 1024 ** 3):
    return AgentCore(spool, str(tmp_path / "spool"), CameraProfile(), "boot", sent.append, quota_bytes=quota,
                     monotonic=mono, wall_ns=lambda: int(mono() * NS), disk_free=lambda p: free)


def test_recording_follows_the_command_and_stops_after_the_grace(tmp_path, spool):
    sent, mono = [], Mono()
    c = core(tmp_path, spool, sent, mono)
    c.on_active_command({"commandId": "c1", "jobStepId": "s1"})
    assert sent[-1]["action"] == "start"
    key = sent[-1]["recordingKey"]
    c.on_active_command(None)
    mono.t += 10
    c.tick()
    assert sent[-1]["action"] == "start", "still within 15 s of the last command"
    mono.t += 6
    c.tick()
    assert sent[-1] == {"action": "stop", "recordingKey": key}
    assert spool.recording(key).state == "ENDED" and spool.recording(key).job_step_id == "s1"


def test_an_open_run_keeps_recording_and_a_new_run_starts_a_new_recording(tmp_path, spool):
    sent, mono = [], Mono()
    c = core(tmp_path, spool, sent, mono)
    c.on_active_run({"runId": "run-1"})
    first = sent[-1]["recordingKey"]
    mono.t += 60
    c.on_active_run({"runId": "run-1"})
    assert len(sent) == 1
    c.on_active_run({"runId": "run-2"})
    assert [m["action"] for m in sent] == ["start", "stop", "start"] and sent[-1]["recordingKey"] != first


def test_telemetry_is_kept_only_during_a_run_and_flushed_in_batches(tmp_path, spool):
    sent, mono = [], Mono()
    c = core(tmp_path, spool, sent, mono)
    c.on_pose(1, 2, 0, 0.3, 0, 0.0, True)
    assert not c.state.trajectory, "no run: nothing to replay"
    c.on_active_run({"runId": "run-1"})
    for i in range(10):
        mono.t += 0.25
        c.on_pose(1 + i * 0.05, 2, 0, 0.3, 0, 0.0, True)
    c.on_plan([(0, 0), (1, 0), (2, 0)])
    c.on_plan([(0, 0), (1, 0), (2, 0)])  # same path: not a replan
    c.on_plan([(0, 0), (1, 1), (2, 0)])
    mono.t += 6
    c.tick()
    kinds = [b[1] for b in spool.due_batches()]
    assert kinds.count("plans") == 2 and kinds.count("trajectory") == 1
    traj = next(b for b in spool.due_batches() if b[1] == "trajectory")
    assert traj[2] == "run-1" and len(traj[3]) == 10
    assert [p["sequenceNumber"] for p in traj[3]] == list(range(1, 11))


def test_a_full_spool_pauses_recording_instead_of_deleting(tmp_path, spool):
    sent, mono = [], Mono()
    c = core(tmp_path, spool, sent, mono, quota=1000)
    rec = tmp_path / "spool" / "recordings" / "old"
    rec.mkdir(parents=True)
    (rec / "x.avi").write_bytes(os.urandom(2000))
    c.on_active_command({"commandId": "c1"})
    assert not sent, "quota exceeded: recording paused"
    assert c.state.paused_for_quota
    (rec / "x.avi").unlink()
    c.tick()
    assert sent and sent[-1]["action"] == "start"


def test_iso_is_utc_with_microseconds():
    assert iso(1_763_348_400 * NS + 123_456_789) == "2025-11-17T03:00:00.123456Z"


def test_finalize_waits_for_the_last_segment_and_a_late_one_still_goes_up(tmp_path, spool):
    api, now = FakeApi(), [100 * NS]
    up = Uploader(spool, api, put=lambda *a: None, delete_uploaded=False, wall_ns=lambda: now[0])
    spool.add_recording("k", 0, "boot", None, str(tmp_path), PROFILE)
    spool.add_segment(segment(tmp_path, "k", 0, 0))
    spool.end_recording("k", 100 * NS)
    up.pump()
    assert not [c for c in api.calls if c[0] == "finalize"], "the camera may still close a segment"
    spool.add_segment(segment(tmp_path, "k", 1, 30))  # the camera's last segment
    now[0] += 11 * NS
    up.pump()
    assert ("finalize", 2, 0) in api.calls
    spool.add_segment(segment(tmp_path, "k", 2, 60))  # later still: registered all the same
    spool._q("UPDATE recordings SET next_attempt_at = 0")
    up.pump()
    assert ("register", [2]) in api.calls
