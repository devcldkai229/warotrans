"""What the evidence agent decides, without ROS (the node feeds it; tests drive it directly).

Recording policy: the camera records while the robot executes a backend command (``/fleet/active_command``) or while
the Recorder says the robot holds a run, and stops 15 s after both are gone. This works offline: the command comes
over MQTT from the fleet backend, so recording starts even when the Recorder cannot be reached, and the recording is
bound to its run later. A run change (another trip) closes the recording and opens a new one.

Telemetry (path, events, plans) is only kept while a run is known: outside a trip there is no journey to replay.
"""

from __future__ import annotations

import os
import shutil
import time
import uuid
from dataclasses import dataclass, field
from typing import Callable, Optional

from .sampling import PoseSample, TrajectorySampler, decimate, path_hash
from .spool import Spool
from .uploader import iso


@dataclass
class CameraProfile:
    camera_id: str = "front"
    camera_model: str = "Raspberry Pi Camera Module 3"
    nominal_fov_degrees: Optional[float] = None
    width: int = 1280
    height: int = 720
    fps: int = 15
    software_version: str = "warotrans_camera 0.2.0"

    def as_dict(self) -> dict:
        return {"cameraId": self.camera_id, "cameraModel": self.camera_model, "nominalFovDegrees": self.nominal_fov_degrees,
                "width": self.width, "height": self.height, "fps": self.fps, "captureSoftwareVersion": self.software_version}


@dataclass
class AgentState:
    recording_key: Optional[str] = None
    recording_run_id: Optional[str] = None
    last_command_at: float = -1e9
    command: Optional[dict] = None
    run: Optional[dict] = None
    run_seen_at: float = -1e9
    trajectory: list[dict] = field(default_factory=list)
    events: list[dict] = field(default_factory=list)
    last_flush_at: float = 0.0
    last_plan_hash: Optional[str] = None
    paused_for_quota: bool = False


class AgentCore:
    STOP_AFTER_S = 15.0
    RUN_FRESH_S = 10.0
    FLUSH_EVERY_S = 5.0

    def __init__(self, spool: Spool, spool_dir: str, profile: CameraProfile, boot_session_id: str,
                 send_control: Callable[[dict], None], quota_bytes: int = 20 * 1024 ** 3, map_version: Optional[str] = None,
                 monotonic: Callable[[], float] = time.monotonic, wall_ns: Callable[[], int] = time.time_ns,
                 disk_free: Callable[[str], int] = lambda p: shutil.disk_usage(p).free, logger=None) -> None:
        self.spool, self.spool_dir, self.profile = spool, spool_dir, profile
        self.boot_session_id = boot_session_id
        self.send_control = send_control
        self.quota_bytes = quota_bytes
        self.map_version = map_version
        self._mono, self._wall, self._disk_free = monotonic, wall_ns, disk_free
        self.log = logger
        self.state = AgentState()
        self.sampler = TrajectorySampler()
        os.makedirs(os.path.join(spool_dir, "recordings"), exist_ok=True)

    # ------------------------------------------------------------------ inputs

    def on_active_command(self, command: Optional[dict]) -> None:
        if command:
            self.state.command = command
            self.state.last_command_at = self._mono()
        else:
            self.state.command = None
        self.tick()

    def on_active_run(self, run: Optional[dict]) -> None:
        """The Recorder's view of the robot's open run (polled); None = no open run."""
        self.state.run = run
        self.state.run_seen_at = self._mono()
        self.tick()

    def on_camera_segment(self, info: dict) -> None:
        if self.spool.add_segment(info) and self.log:
            self.log.info(f"evidence: segment {info['sequence_number']} of {info['recording_key']} ({info['byte_size']} B)")

    def on_pose(self, x: float, y: float, yaw: float, v: Optional[float], w: Optional[float], tf_age_s: float,
                localized: bool, force: bool = False) -> None:
        run_id = self.run_id
        if run_id is None:
            return
        sample, event = self.sampler.offer(x, y, yaw, v, w, tf_age_s, localized, self._wall(), force)
        if sample is not None:
            self.state.trajectory.append(self._point(sample))
        if event is not None:
            self.add_event(event, "NAVIGATION", "WARNING" if event != "localization.regained" else "INFO",
                           {"jump": event == "localization.jump"}, x=x if sample and sample.valid else None,
                           y=y if sample and sample.valid else None)

    def on_plan(self, points: list[tuple[float, float]], planner: str = "nav2") -> None:
        run_id = self.run_id
        if run_id is None or not points:
            return
        path = decimate(points)
        digest = path_hash(path)
        if digest == self.state.last_plan_hash:
            return
        version = self.spool.next_value(f"plan:{run_id}")
        reason = "initial" if self.state.last_plan_hash is None else "replan"
        self.state.last_plan_hash = digest
        self.spool.enqueue("plans", run_id, [{
            "planVersion": version, "plannerName": planner, "generatedAtUtc": iso(self._wall()), "frameId": "map",
            "mapVersion": self.map_version, "pathPoints": path, "pathHash": digest, "reason": reason}])

    def add_event(self, event_type: str, category: str, severity: str, payload: Optional[dict] = None,
                  message: Optional[str] = None, x: Optional[float] = None, y: Optional[float] = None) -> None:
        if self.run_id is None:
            return
        self.state.events.append({
            "sourceEventId": str(uuid.uuid4()), "eventType": event_type, "category": category, "severity": severity,
            "occurredAtUtc": iso(self._wall()), "x": x, "y": y, "message": message or event_type, "payload": payload or {}})

    def add_clock_anchor(self, monotonic_ns: int, quality) -> None:
        self.spool.enqueue("anchors", None, [{
            "bootSessionId": self.boot_session_id, "monotonicNs": monotonic_ns, "utcEstimate": iso(self._wall()),
            "syncSource": quality.source, "offsetMs": quality.offset_ms, "errorBoundMs": quality.error_bound_ms,
            "synchronized": quality.synchronized}])

    # ------------------------------------------------------------------ decisions

    @property
    def run_id(self) -> Optional[str]:
        s = self.state
        fresh = self._mono() - s.run_seen_at <= self.RUN_FRESH_S
        return s.run["runId"] if s.run and fresh else None

    def wants_recording(self) -> bool:
        s = self.state
        now = self._mono()
        commanded = s.command is not None or now - s.last_command_at < self.STOP_AFTER_S
        return commanded or self.run_id is not None

    def tick(self) -> None:
        s = self.state
        self._check_quota()
        want = self.wants_recording() and not s.paused_for_quota
        run_changed = s.recording_key is not None and self.run_id is not None and s.recording_run_id not in (None, self.run_id)
        if s.recording_key is not None and (not want or run_changed):
            self._stop_recording()
        if want and s.recording_key is None:
            self._start_recording()
        elif s.recording_key is not None and s.recording_run_id is None and self.run_id is not None:
            s.recording_run_id = self.run_id
        if self._mono() - s.last_flush_at >= self.FLUSH_EVERY_S:
            self.flush()

    def flush(self) -> None:
        s = self.state
        s.last_flush_at = self._mono()
        run_id = self.run_id or s.recording_run_id
        if run_id is None:
            s.trajectory.clear()
            s.events.clear()
            return
        if s.trajectory:
            self.spool.enqueue("trajectory", run_id, s.trajectory[:])
            s.trajectory.clear()
        if s.events:
            self.spool.enqueue("events", run_id, s.events[:])
            s.events.clear()

    def shutdown(self) -> None:
        if self.state.recording_key is not None:
            self._stop_recording()
        self.flush()

    # ------------------------------------------------------------------ internals

    def _point(self, p: PoseSample) -> dict:
        return {"sequenceNumber": self.spool.next_value(f"traj:{self.run_id}"), "capturedAtUtc": iso(p.wall_ns),
                "x": p.x, "y": p.y, "yawRad": p.yaw, "linearVelocityMps": p.v, "angularVelocityRadps": p.w,
                "poseSource": "tf:map->base_link", "frameId": "map", "localizationQuality": None, "isValid": p.valid,
                "mapVersion": self.map_version}

    def _start_recording(self) -> None:
        s = self.state
        key = uuid.uuid4().hex
        directory = os.path.join(self.spool_dir, "recordings", key)
        job_step = (s.command or {}).get("jobStepId")
        self.spool.add_recording(key, self._wall(), self.boot_session_id, job_step, directory, self.profile.as_dict())
        s.recording_key, s.recording_run_id = key, self.run_id
        s.last_plan_hash = None
        self.send_control({"action": "start", "recordingKey": key, "directory": directory, "firstSequence": 0})
        if self.log:
            self.log.info(f"evidence: recording {key} started (step {job_step}, run {s.recording_run_id})")

    def _stop_recording(self) -> None:
        s = self.state
        key = s.recording_key
        self.send_control({"action": "stop", "recordingKey": key})
        self.spool.end_recording(key, self._wall())
        s.recording_key = s.recording_run_id = None
        if self.log:
            self.log.info(f"evidence: recording {key} stopped")

    def _check_quota(self) -> None:
        s = self.state
        used = _dir_size(os.path.join(self.spool_dir, "recordings"))
        low_disk = self._disk_free(self.spool_dir) < 1024 ** 3  # keep 1 GiB for the system
        full = used >= self.quota_bytes or low_disk
        if full and not s.paused_for_quota:
            s.paused_for_quota = True
            if self.log:
                self.log.error(f"evidence: spool full ({used} B used, quota {self.quota_bytes} B): recording paused, "
                               "nothing is deleted before it is uploaded")
            self.add_event("recording.spool_full", "RECORDING", "ERROR", {"usedBytes": used, "quotaBytes": self.quota_bytes})
        elif s.paused_for_quota and used < self.quota_bytes * 0.9 and not low_disk:
            s.paused_for_quota = False


def _dir_size(path: str) -> int:
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += os.path.getsize(os.path.join(root, name))
            except OSError:
                pass
    return total
