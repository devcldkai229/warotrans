"""Moves what is in the spool to the Evidence Recorder, in order, at its own pace, surviving any interruption.

One ``pump()`` does a bounded amount of work and returns; the agent calls it from a background thread. Every step can
be repeated (the Recorder is idempotent), so a crash between "sent" and "marked" only causes a harmless resend.

Per recording: bind it to the robot's run (``409 no_run_yet`` → ask again later), register closed segments in sequence
order, upload each through its presigned grant, finalize once the recording ended and every segment is registered
(uploads may still follow: the Recorder waits for them before transcoding). Telemetry batches go independently.
"""

from __future__ import annotations

import os
import time
from typing import Callable, Optional

from .recorder_api import ApiError, RecorderApi, put_file
from .spool import Spool

#: The camera closes its last segment a moment after the agent stopped the recording: finalize only after this.
FINALIZE_SETTLE_S = 10.0
#: The Recorder does not know the segment (404/410): asking again never helps. It is parked this long, not retried
#: every few seconds (lab 2026-11-24: ~70 such recordings from earlier days took every upload slot, 2,475 attempts
#: each, and the Recorder answered 429; no segment of a new trip was ever uploaded).
ABANDON_S = 30 * 24 * 3600.0

SEGMENT_FIELDS = ("segment_id", "sequence_number", "start_monotonic_ns", "end_monotonic_ns", "duration_ms",
                  "frame_count", "dropped_frame_count", "duplicated_frame_count", "max_slot_error_ms",
                  "container_format", "codec", "byte_size", "sha256")


def iso(ns: int) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(ns // 1_000_000_000)) + f".{(ns % 1_000_000_000) // 1000:06d}Z"


def camel(name: str) -> str:
    head, *rest = name.split("_")
    return head + "".join(p.title() for p in rest)


def segment_intent(info: dict) -> dict:
    body = {camel(k): info[k] for k in SEGMENT_FIELDS}
    body["capturedFromUtc"] = iso(int(info["captured_from_utc_ns"]))
    body["capturedToUtc"] = iso(int(info["captured_to_utc_ns"]))
    body["bootSessionId"] = info["boot_session_id"]
    return body


def backoff(attempts: int, base: float = 5.0, cap: float = 300.0) -> float:
    return min(cap, base * (2 ** min(attempts, 8)))


class Uploader:
    def __init__(self, spool: Spool, api: RecorderApi, logger=None, rate_limit_bps: Optional[float] = 2_000_000,  # bytes/s
                 delete_uploaded: bool = True, put: Callable = put_file, wall_ns: Callable[[], int] = time.time_ns,
                 settle_s: float = FINALIZE_SETTLE_S) -> None:
        self.spool, self.api, self.log = spool, api, logger
        self.rate_limit_bps = rate_limit_bps
        self.delete_uploaded = delete_uploaded
        self._put = put
        self._wall_ns = wall_ns
        self.settle_s = settle_s
        self.last_error = ""

    def _warn(self, text: str) -> None:
        self.last_error = text
        if self.log:
            self.log.warning(text)

    def pump(self, max_uploads: int = 3) -> dict:
        stats = {"bound": 0, "registered": 0, "uploaded": 0, "finalized": 0, "batches": 0}
        for rec in self.spool.recordings(("RECORDING", "ENDED"), due_only=True):
            if rec.recording_id is None and not self._bind(rec):
                continue
            stats["bound"] += 1 if rec.recording_id is None else 0
            rec = self.spool.recording(rec.key)
            stats["registered"] += self._register(rec)
            settled = rec.ended_at_ns is not None and (self._wall_ns() - rec.ended_at_ns) / 1e9 >= self.settle_s
            if rec.state == "ENDED" and settled and all(s["registered"] for s in self.spool.segments(rec.key)):
                if self._finalize(rec):
                    stats["finalized"] += 1
        # A segment closed after the finalize still goes up (the Recorder seals it as its own group).
        for rec in self.spool.recordings(("FINALIZED",), due_only=True):
            if rec.recording_id is not None:
                stats["registered"] += self._register(rec)
        # Newest first: the trip that just ran is the one someone will replay; an old backlog waits behind it.
        for rec in sorted(self.spool.recordings(("RECORDING", "ENDED", "FINALIZED")), key=lambda r: r.started_at_ns,
                          reverse=True):
            if rec.recording_id is None or stats["uploaded"] >= max_uploads:
                continue
            for info in self.spool.due_uploads(rec.key)[: max_uploads - stats["uploaded"]]:
                stats["uploaded"] += self._upload(info)
        stats["batches"] = self._send_batches()
        if self.delete_uploaded:
            self._delete_confirmed()
        return stats

    # ------------------------------------------------------------------ steps

    def _bind(self, rec) -> bool:
        p = rec.profile
        body = {
            "edgeRecordingKey": rec.key, "jobStepId": rec.job_step_id, "startedAtUtc": iso(rec.started_at_ns),
            "bootSessionId": rec.boot_session_id, "cameraId": p.get("cameraId", "front"),
            "cameraModel": p.get("cameraModel", ""), "nominalFovDegrees": p.get("nominalFovDegrees"),
            "cameraCalibrationVersion": p.get("cameraCalibrationVersion"), "mountPose": p.get("mountPose"),
            "width": p["width"], "height": p["height"], "targetFps": p["fps"], "sourceCodec": "MJPEG",
            "captureSoftwareVersion": p.get("captureSoftwareVersion", "warotrans_camera"),
        }
        try:
            res = self.api.start_recording(body)
        except ApiError as exc:
            self.spool.recording_failed(rec.key, str(exc), backoff(rec.attempts, 5.0, 60.0) if exc.transient else 3600.0)
            if exc.code != "no_run_yet":
                self._warn(f"evidence: cannot bind recording {rec.key}: {exc}")
            return False
        self.spool.bind_recording(rec.key, res["recordingId"], res["runId"])
        return True

    def _register(self, rec) -> int:
        todo = [s for s in self.spool.segments(rec.key) if not s["registered"]]
        if not todo:
            return 0
        try:
            res = self.api.register_segments(rec.recording_id, [segment_intent(s) for s in todo[:100]])
        except ApiError as exc:
            self.spool.recording_failed(rec.key, str(exc), backoff(rec.attempts))
            self._warn(f"evidence: register {rec.key}: {exc}")
            return 0
        ids = [s["segmentId"] for s in res.get("segments", [])]
        self.spool.mark_registered(ids)
        return len(ids)

    def _finalize(self, rec) -> bool:
        segments = self.spool.segments(rec.key)
        body = {"endedAtUtc": iso(rec.ended_at_ns or rec.started_at_ns), "expectedSegmentCount": len(segments),
                "gaps": self._gaps(segments), "reason": "run ended"}
        try:
            self.api.finalize(rec.recording_id, body)
        except ApiError as exc:
            self.spool.recording_failed(rec.key, str(exc), backoff(rec.attempts))
            self._warn(f"evidence: finalize {rec.key}: {exc}")
            return False
        self.spool.finalize_recording(rec.key)
        return True

    @staticmethod
    def _gaps(segments: list[dict]) -> list[dict]:
        gaps = []
        for prev, cur in zip(segments, segments[1:]):
            if cur.get("gap_before") or int(cur["captured_from_utc_ns"]) - int(prev["captured_to_utc_ns"]) > 2_000_000_000:
                gaps.append({"fromUtc": iso(int(prev["captured_to_utc_ns"])), "toUtc": iso(int(cur["captured_from_utc_ns"])),
                             "reason": cur.get("gap_before") or "FRAME_GAP"})
        return gaps

    def _upload(self, info: dict) -> int:
        sid = info["segment_id"]
        if not os.path.exists(info["path"]):
            self.spool.segment_failed(sid, "file missing", 3600.0)
            self._warn(f"evidence: segment file {info['path']} is gone")
            return 0
        try:
            grant = self.api.upload_grant(sid)
        except ApiError as exc:
            if exc.code in ("already_uploaded", "quarantined"):
                self.spool.mark_uploaded(sid)  # nothing more this robot can do for it
                return 0
            if exc.status in (404, 410):
                self.spool.segment_failed(sid, f"abandoned: {exc}", ABANDON_S)
                self._warn(f"evidence: the Recorder does not know segment {sid}; parked ({exc.status})")
                return 0
            # Growing backoff (was a fixed 5 s whatever the number of failures).
            self.spool.segment_failed(sid, str(exc), backoff(self.spool.segment_attempts(sid)))
            return 0
        try:
            self._put(grant, info["path"], self.rate_limit_bps)
        except ApiError as exc:
            if exc.status == 412:  # already stored (an earlier attempt got through)
                self.spool.mark_uploaded(sid)
                return 1
            self.spool.segment_failed(sid, str(exc), backoff(1))
            self._warn(f"evidence: upload {sid}: {exc}")
            return 0
        self.spool.mark_uploaded(sid)
        return 1

    def _send_batches(self) -> int:
        sent = 0
        for batch_id, kind, run_id, payload in self.spool.due_batches():
            try:
                if kind == "trajectory":
                    self.api.trajectory(run_id, payload)
                elif kind == "events":
                    self.api.events(run_id, payload)
                elif kind == "plans":
                    self.api.plans(run_id, payload)
                elif kind == "anchors":
                    self.api.clock_anchors(payload)
            except ApiError as exc:
                if exc.transient:
                    self.spool.batch_failed(batch_id, backoff(1))
                    continue
                self._warn(f"evidence: {kind} batch refused: {exc}")  # bad batch: dropped, never blocks the rest
            self.spool.batch_done(batch_id)
            sent += 1
        return sent

    def _delete_confirmed(self) -> None:
        for path in self.spool.uploaded_files():
            try:
                os.remove(path)
            except FileNotFoundError:
                pass
            except OSError as exc:
                self._warn(f"evidence: cannot delete {path}: {exc}")
