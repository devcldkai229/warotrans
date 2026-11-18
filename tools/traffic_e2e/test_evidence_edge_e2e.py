"""Evidence phase 2 end to end, motion-free: the robot side code against the real Recorder and Worker processes.

  real RecorderSink (fake camera frames, real AVI files on disk)
    -> real AgentCore + Spool + Uploader (real HTTP, robot credential)
    -> real Evidence Recorder (Local store: presigned PUT, checksum, S3-style notification)
    -> real Evidence Worker (queue consumer verifies size + SHA-256, seals the group, one media job, outbox published)

Plus: path, plans, robot events and clock anchors reach the run; a Worker that is down when the files arrive verifies
them once it is back; a Recorder unreachable for a while loses nothing (the spool keeps it).

Run on its own:  python -m pytest tools/traffic_e2e/test_evidence_edge_e2e.py -q
(needs Postgres on 5433 and Debug builds of WaroTrans.Evidence.Recorder and .Worker)
"""

from __future__ import annotations

import json
import shutil
import sys
import time
import uuid
from dataclasses import asdict
from pathlib import Path

import pytest

from harness import (EVIDENCE_DB, EVIDENCE_STORE, EVIDENCE_TOKEN, ROBOT_REPO, Api, EvidenceRecorder, EvidenceWorker,
                     mint_jwt, wait_for)
from seed_test_db import psql

sys.path.insert(0, str(ROBOT_REPO / "ros2_ws" / "src" / "warotrans_camera"))
sys.path.insert(0, str(ROBOT_REPO / "ros2_ws" / "src" / "warotrans_evidence_agent"))

from warotrans_camera.avi_writer import read_avi_index  # noqa: E402
from warotrans_camera.recorder_sink import RecorderSink  # noqa: E402
from warotrans_evidence_agent.agent_core import AgentCore, CameraProfile  # noqa: E402
from warotrans_evidence_agent.recorder_api import RecorderApi  # noqa: E402
from warotrans_evidence_agent.sampling import ClockQuality  # noqa: E402
from warotrans_evidence_agent.spool import Spool  # noqa: E402
from warotrans_evidence_agent.uploader import Uploader  # noqa: E402

HERE = Path(__file__).resolve().parent
ARTIFACTS = HERE / "artifacts"
NS = 1_000_000_000
RUN = uuid.uuid4().hex[:4].upper()


def sql(query: str) -> list:
    return json.loads(psql(EVIDENCE_DB, f"SELECT coalesce(jsonb_agg(t), '[]'::jsonb) FROM ({query}) t;") or "[]")


@pytest.fixture(scope="module")
def evidence():
    ARTIFACTS.mkdir(exist_ok=True)
    shutil.rmtree(EVIDENCE_STORE, ignore_errors=True)
    recorder = EvidenceRecorder(ARTIFACTS, core_base="http://127.0.0.1:9")  # no Host needed: events are pushed here
    worker = EvidenceWorker(ARTIFACTS)
    recorder.start()
    try:
        worker.start()
    except BaseException:
        recorder.stop()
        raise
    yield recorder, worker
    worker.stop()
    recorder.stop()


def open_run(recorder: EvidenceRecorder, robot: str, request: str) -> None:
    """The monolith's EvidenceBridge would push this when the robot takes a trip."""
    service = Api(recorder.base)
    service.token = ""
    now = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(time.time() - 5))
    event = {"eventId": str(uuid.uuid4()), "schemaVersion": 1, "eventType": "job.robot-assignment-changed",
             "occurredAtUtc": now, "source": "warotrans-core", "transportRequestId": request,
             "requestCode": f"REQ-E2E-{RUN}", "jobId": str(uuid.uuid4()), "jobNo": f"JOB-E2E-{RUN}",
             "robotId": str(uuid.uuid4()), "robotCode": robot, "assignmentId": str(uuid.uuid4()),
             "sourceStatusCode": "OPENED",
             "payload": {"startedAt": now, "startReason": "START", "containerIds": [str(uuid.uuid4())],
                         "mapVersionCode": "MAP-ONE-V014", "transportRequestStatus": "IN_PROGRESS"}}
    import urllib.request
    req = urllib.request.Request(f"{recorder.base}/api/evidence/integrations/transport-events",
                                 data=json.dumps({"events": [event]}).encode(), method="POST")
    req.add_header("Content-Type", "application/json")
    req.add_header("X-Evidence-Service-Token", EVIDENCE_TOKEN)
    with urllib.request.urlopen(req, timeout=10) as resp:
        assert json.loads(resp.read())["accepted"] == 1


def robot_credential(recorder: EvidenceRecorder, robot: str) -> str:
    admin = Api(recorder.base)
    admin.token = mint_jwt("ADMIN")
    return admin.call("POST", f"/api/evidence/admin/robots/{robot}/credentials")["token"]


class FakeCamera:
    def __init__(self) -> None:
        self.frame, self.seq, self.mono, self.wall, self.viewers = None, 0, 0, 0, 0

    def add_viewer(self):
        self.viewers += 1

    def remove_viewer(self):
        self.viewers -= 1

    def latest(self):
        return self.frame, self.seq, self.mono, self.wall


def record_segments(directory: str, key: str, seconds: float, on_segment) -> None:
    """Real RecorderSink on a simulated clock: 10 fps, 2 s segments, frames stamped 4 ms before their slot."""
    cam = FakeCamera()
    t = {"mono": 1 * NS}
    wall0 = time.time_ns() - int(seconds * NS) - 2 * NS
    sink_ref = []

    def mono():
        return t["mono"]

    def wall():
        return wall0 + t["mono"]

    def sleep(s):
        t["mono"] += int(s * NS)
        cam.seq += 1
        cam.frame = b"\xff\xd8" + bytes([cam.seq % 251]) * 3000 + b"\xff\xd9"
        cam.mono, cam.wall = t["mono"] - 4_000_000, wall() - 4_000_000
        if t["mono"] > (seconds + 1) * NS:
            sink_ref[0]._stop.set()

    sink = RecorderSink(cam, on_segment, 640, 480, 10, 2.0, clock_monotonic_ns=mono, clock_wall_ns=wall, sleep=sleep)
    sink_ref.append(sink)
    sleep(0)
    sink.start(key, directory)
    sink._thread.join(30)
    sink.stop()


def test_a_robot_recording_reaches_storage_verified_and_queued_for_transcoding(evidence, tmp_path):
    recorder, _ = evidence
    robot, request = f"E2E-EV-{RUN}", str(uuid.uuid4())
    open_run(recorder, robot, request)
    api = RecorderApi(recorder.base, robot, robot_credential(recorder, robot))
    assert api.whoami()["robotCode"] == robot
    run = api.active_run()
    assert run and run["transportRequestId"] == request

    spool = Spool(str(tmp_path / "spool" / "evidence.db"))
    controls = []
    core = AgentCore(spool, str(tmp_path / "spool"), CameraProfile(width=640, height=480, fps=10, nominal_fov_degrees=120),
                     str(uuid.uuid4()), controls.append, map_version="MAP-ONE-V014")
    core.on_active_command({"commandId": str(uuid.uuid4()), "jobStepId": str(uuid.uuid4()), "type": "NAVIGATE_TO_POSE"})
    core.on_active_run(run)
    start = controls[-1]
    assert start["action"] == "start"

    # The camera process records into the folder the agent chose; each closed segment is reported to the agent.
    record_segments(start["directory"], start["recordingKey"], 6.0, lambda info: core.on_camera_segment(asdict(info)))
    segments = spool.segments(start["recordingKey"])
    assert len(segments) >= 3
    assert read_avi_index(segments[0]["path"])["fps"] == 10

    # Path, a replan, an event and a clock anchor of the same run.
    for i in range(6):
        core.on_pose(0.1 * i, 1.0, 0.0, 0.3, 0.0, 0.05, True, force=True)
        time.sleep(0.01)
    core.on_plan([(0.0, 1.0), (1.0, 1.0), (2.0, 1.0)])
    core.on_plan([(0.0, 1.0), (1.0, 1.5), (2.0, 1.0)])
    core.add_event("nav2.goal.aborted", "NAVIGATION", "ERROR", {"why": "e2e"}, message="Nav2 goal ABORTED", x=0.5, y=1.0)
    core.add_clock_anchor(time.monotonic_ns(), ClockQuality("chrony", True, 0.3, 2.0))
    core.shutdown()  # the trip ended: recording stops, telemetry flushed
    assert controls[-1]["action"] == "stop"

    uploader = Uploader(spool, api, rate_limit_bps=None, delete_uploaded=True)
    deadline = time.time() + 60
    while time.time() < deadline:
        uploader.pump(max_uploads=10)
        if spool.pending_upload_count() == 0 and spool.recording(start["recordingKey"]).state == "FINALIZED" and spool.outbox_size() == 0:
            break
        time.sleep(0.5)
    assert spool.pending_upload_count() == 0, uploader.last_error
    assert spool.outbox_size() == 0, uploader.last_error

    # The Worker verified every file and queued exactly one transcode of the continuous group.
    count = len(segments)
    wait_for(lambda: sql(f"SELECT 1 FROM evidence.video_segments s JOIN evidence.camera_recordings r ON r.id = s.recording_id "
                         f"WHERE r.edge_recording_key = '{start['recordingKey']}' AND s.upload_status = 'VERIFIED'") .__len__() == count,
             60, interval=1.0, what="segments verified by the Worker")
    jobs = wait_for(lambda: sql(f"SELECT j.status FROM evidence.media_processing_jobs j JOIN evidence.media_assets a ON a.id = j.media_asset_id "
                                f"JOIN evidence.camera_recordings r ON r.id = a.recording_id WHERE r.edge_recording_key = '{start['recordingKey']}'"),
                    30, interval=1.0, what="media job")
    assert [j["status"] for j in jobs] == ["QUEUED"]
    wait_for(lambda: sql("SELECT 1 FROM evidence.outbox WHERE published_at_utc IS NULL") == [], 30, interval=1.0,
             what="outbox published to media-process")
    recording = sql(f"SELECT status, expected_segment_count, nominal_fov_degrees FROM evidence.camera_recordings "
                    f"WHERE edge_recording_key = '{start['recordingKey']}'")[0]
    assert recording["expected_segment_count"] == count and recording["status"] == "UPLOADING"
    assert recording["nominal_fov_degrees"] == 120

    run_id = run["runId"]
    assert len(sql(f"SELECT 1 FROM evidence.trajectory_points WHERE robot_run_evidence_id = '{run_id}'")) == 6
    assert len(sql(f"SELECT 1 FROM evidence.route_plans WHERE robot_run_evidence_id = '{run_id}'")) == 2
    assert sql(f"SELECT event_type FROM evidence.events WHERE robot_run_evidence_id = '{run_id}' AND source_system = 'robot:{robot}'") \
        == [{"event_type": "nav2.goal.aborted"}]
    assert sql(f"SELECT time_sync_error_bound_ms FROM evidence.robot_runs WHERE id = '{run_id}'")[0]["time_sync_error_bound_ms"] == 2.0
    # Uploaded and confirmed: the robot's disk is freed.
    assert not any(Path(s["path"]).exists() for s in segments)


def test_files_that_arrive_while_the_worker_is_down_are_verified_once_it_is_back(evidence, tmp_path):
    recorder, worker = evidence
    robot, request = f"E2E-EW-{RUN}", str(uuid.uuid4())
    open_run(recorder, robot, request)
    api = RecorderApi(recorder.base, robot, robot_credential(recorder, robot))
    spool = Spool(str(tmp_path / "spool" / "evidence.db"))
    controls = []
    core = AgentCore(spool, str(tmp_path / "spool"), CameraProfile(width=640, height=480, fps=10), str(uuid.uuid4()), controls.append)
    core.on_active_run(api.active_run())
    start = controls[-1]
    record_segments(start["directory"], start["recordingKey"], 4.0, lambda info: core.on_camera_segment(asdict(info)))
    core.shutdown()

    # The Recorder cannot be reached at first: nothing is lost, the spool keeps every file.
    offline = Uploader(spool, RecorderApi("http://127.0.0.1:9", robot, "nope", timeout=1), rate_limit_bps=None)
    offline.pump(max_uploads=10)
    assert spool.pending_upload_count() == len(spool.segments(start["recordingKey"]))

    worker.stop()
    try:
        spool._q("UPDATE recordings SET next_attempt_at = 0")
        spool._q("UPDATE segments SET next_attempt_at = 0")
        uploader = Uploader(spool, api, rate_limit_bps=None)
        for _ in range(20):
            uploader.pump(max_uploads=10)
            if spool.pending_upload_count() == 0 and spool.recording(start["recordingKey"]).state == "FINALIZED":
                break
            time.sleep(0.3)
        assert spool.pending_upload_count() == 0, uploader.last_error
        time.sleep(1.0)
        verified = sql(f"SELECT 1 FROM evidence.video_segments s JOIN evidence.camera_recordings r ON r.id = s.recording_id "
                       f"WHERE r.edge_recording_key = '{start['recordingKey']}' AND s.upload_status = 'VERIFIED'")
        assert verified == [], "nobody verifies while the Worker is down"
    finally:
        worker.start()

    total = len(spool.segments(start["recordingKey"]))
    wait_for(lambda: len(sql(f"SELECT 1 FROM evidence.video_segments s JOIN evidence.camera_recordings r ON r.id = s.recording_id "
                             f"WHERE r.edge_recording_key = '{start['recordingKey']}' AND s.upload_status = 'VERIFIED'")) == total,
             60, interval=1.0, what="verification after the Worker came back")
    wait_for(lambda: len(sql(f"SELECT 1 FROM evidence.media_processing_jobs j JOIN evidence.media_assets a ON a.id = j.media_asset_id "
                             f"JOIN evidence.camera_recordings r ON r.id = a.recording_id WHERE r.edge_recording_key = '{start['recordingKey']}'")) == 1,
             30, interval=1.0, what="one media job")
