"""Records the camera's frames into constant-frame-rate MJPEG AVI segments on the robot's disk.

Single camera owner: the sink runs inside the camera process and reads the same latest-frame buffer the live stream
uses (it registers as a viewer, so rpicam-vid keeps running while it records). No second process ever opens the
camera.

Timing (spec §6): every frame is stamped with CLOCK_MONOTONIC and the wall clock the moment it is cut from the
rpicam-vid stream. The sink then fills fixed slots t0 + k/fps (monotonic): each slot takes the newest frame, a slot with
no new frame repeats the last one (duplicated), frames that arrive between two slots are skipped (dropped). Media time
k/fps therefore stays within one frame of capture time, and the timing sidecar records, for each slot, the capture
stamp of the frame in it, so the backend can state the real error bound instead of assuming one.

Durability: a segment is written as ``<id>.avi.partial``; on rotation it is closed, fsynced and renamed to
``<id>.avi``, then hashed. A power cut leaves at most one ``.partial`` per segment being finished (recovered or
discarded by the agent, never uploaded as complete). Segment rotation never blocks the slot loop: the closed segment is
handed to a finisher thread (close, fsync, rename, SHA-256, sidecar, callback, in segment order) and the loop opens the
next one on the very next slot. Doing that work in the loop cost ~1 s on the Pi's SD card while uploads were reading
it, enough to trip STALL_S and cut a 1 s hole into the video at every rotation (lab 2026-11-21).
"""

from __future__ import annotations

import hashlib
import json
import os
import queue
import threading
import time
import uuid
from dataclasses import asdict, dataclass, field
from typing import Callable, Optional, Protocol

from .avi_writer import AviMjpegWriter


class FrameSource(Protocol):
    """What the camera offers: the newest frame with its sequence number and capture stamps."""

    def add_viewer(self) -> None: ...

    def remove_viewer(self) -> None: ...

    def latest(self) -> tuple[Optional[bytes], int, int, int]:
        """(jpeg, seq, capture_monotonic_ns, capture_wall_ns)."""
        ...


@dataclass
class SegmentInfo:
    segment_id: str
    recording_key: str
    sequence_number: int
    boot_session_id: str
    path: str
    sidecar_path: str
    captured_from_utc_ns: int
    captured_to_utc_ns: int
    start_monotonic_ns: int
    end_monotonic_ns: int
    duration_ms: int
    frame_count: int
    dropped_frame_count: int
    duplicated_frame_count: int
    max_slot_error_ms: float
    byte_size: int
    sha256: str
    width: int
    height: int
    fps: int
    container_format: str = "AVI"
    codec: str = "MJPEG"
    gap_before: Optional[str] = None


@dataclass
class _Open:
    segment_id: str
    sequence_number: int
    writer: AviMjpegWriter
    partial_path: str
    final_path: str
    first_slot_mono: int
    first_slot_wall: int
    last_slot_mono: int = 0
    last_slot_wall: int = 0
    dropped: int = 0
    duplicated: int = 0
    max_err_ns: int = 0
    gap_before: Optional[str] = None
    slots: list[tuple[int, int]] = field(default_factory=list)  # (slot wall ns, frame capture wall ns)


def boot_session_id() -> str:
    """Linux boot id: monotonic stamps only compare within one boot."""
    try:
        with open("/proc/sys/kernel/random/boot_id", encoding="ascii") as f:
            return str(uuid.UUID(f.read().strip()))
    except (OSError, ValueError):
        return str(uuid.uuid5(uuid.NAMESPACE_OID, f"boot-{time.time_ns() - time.monotonic_ns()}"))


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


class RecorderSink:
    """One recording at a time. ``start`` → segments are produced via ``on_segment`` → ``stop`` closes the last one."""

    #: No new frame for this long: the camera is lost, the open segment is closed and a gap starts.
    CAMERA_LOST_S = 1.0

    #: The slot loop is this far behind: it restarts instead of writing a burst of repeated frames.
    STALL_S = 1.0

    def __init__(
        self,
        source: FrameSource,
        on_segment: Callable[[SegmentInfo], None],
        width: int,
        height: int,
        fps: int,
        segment_seconds: float = 30.0,
        logger=None,
        clock_monotonic_ns: Callable[[], int] = time.monotonic_ns,
        clock_wall_ns: Callable[[], int] = time.time_ns,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self.source = source
        self.on_segment = on_segment
        self.width, self.height, self.fps = width, height, int(fps)
        self.segment_seconds = segment_seconds
        self.log = logger
        self._mono, self._wall, self._sleep = clock_monotonic_ns, clock_wall_ns, sleep
        self._thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._finishing: "queue.Queue[Optional[_Open]]" = queue.Queue()
        self._finisher: Optional[threading.Thread] = None
        self.recording_key: Optional[str] = None
        self.directory: Optional[str] = None
        self.boot_id = boot_session_id()
        self.next_sequence = 0
        self.last_error = ""

    @property
    def active(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def start(self, recording_key: str, directory: str, first_sequence: int = 0) -> None:
        if self.active:
            if recording_key == self.recording_key:
                return
            self.stop()
        os.makedirs(directory, exist_ok=True)
        self.recording_key, self.directory, self.next_sequence = recording_key, directory, first_sequence
        self._stop.clear()
        self.source.add_viewer()
        if self._finisher is None or not self._finisher.is_alive():
            self._finisher = threading.Thread(target=self._finish_loop, name="recorder_finisher", daemon=True)
            self._finisher.start()
        self._thread = threading.Thread(target=self._run, name="recorder_sink", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 10.0) -> None:
        if self._thread is None:
            return
        self._stop.set()
        self._thread.join(timeout)
        self._thread = None
        self.source.remove_viewer()
        # Every segment handed over is on disk, hashed and reported before stop() returns.
        if self._finisher is not None:
            self._finishing.put(None)
            self._finisher.join(timeout + 30.0)
            self._finisher = None

    def _finish_loop(self) -> None:
        while True:
            seg = self._finishing.get()
            if seg is None:
                return
            try:
                self._finish(seg)
            except Exception as exc:  # never lose the thread: the next segments still need finishing
                self.last_error = f"recorder: finishing segment {seg.sequence_number} failed: {exc}"
                if self.log:
                    self.log.error(self.last_error)

    # ------------------------------------------------------------------ loop

    def _run(self) -> None:
        period_ns = 1_000_000_000 // self.fps
        seg_slots = max(1, int(round(self.segment_seconds * self.fps)))
        current: Optional[_Open] = None
        last_seq = -1
        last_frame: Optional[bytes] = None
        last_new_frame_mono = self._mono()
        t0 = self._mono()
        t0_wall = self._wall()
        k = 0
        pending_gap: Optional[str] = None
        try:
            while not self._stop.is_set():
                slot_mono = t0 + k * period_ns
                delay = (slot_mono - self._mono()) / 1e9
                if delay > 0:
                    self._sleep(delay)
                elif delay < -self.STALL_S:
                    # The process stalled (CPU starvation, swap): do not fill the lost second with copies of one
                    # frame; close the segment, restart the slot clock, and let the timeline show the hole.
                    if current is not None:
                        self._close(current)
                        current = None
                    pending_gap = "STALL"
                    t0, t0_wall, k = self._mono(), self._wall(), 0
                    continue
                k += 1
                frame, seq, cap_mono, cap_wall = self.source.latest()
                now = self._mono()
                if frame is None or (seq == last_seq and (now - last_new_frame_mono) / 1e9 > self.CAMERA_LOST_S):
                    # Camera not delivering: close what we have, the timeline will show a gap.
                    if current is not None:
                        self._close(current)
                        current = None
                        pending_gap = "CAMERA_LOST"
                    continue
                if seq != last_seq:
                    dropped = max(0, seq - last_seq - 1) if last_seq >= 0 and current is not None else 0
                    last_seq, last_frame, last_new_frame_mono = seq, frame, now
                    duplicated = 0
                else:
                    dropped, duplicated = 0, 1
                    frame = last_frame
                slot_wall = t0_wall + (slot_mono - t0)
                if current is None:
                    current = self._open(slot_mono, slot_wall, pending_gap)
                    pending_gap = None
                    dropped = 0
                current.writer.write_frame(frame)
                current.last_slot_mono, current.last_slot_wall = slot_mono, slot_wall
                current.dropped += dropped
                current.duplicated += duplicated
                err = abs(slot_wall - cap_wall)
                current.max_err_ns = max(current.max_err_ns, err)
                current.slots.append((slot_wall, cap_wall))
                if current.writer.frame_count >= seg_slots:
                    self._close(current)
                    current = None
        except Exception as exc:  # never take the camera process down
            self.last_error = f"recorder: {exc}"
            if self.log:
                self.log.error(self.last_error)
        finally:
            if current is not None:
                self._close(current)

    def _open(self, slot_mono: int, slot_wall: int, gap: Optional[str]) -> _Open:
        segment_id = str(uuid.uuid4())
        seq = self.next_sequence
        self.next_sequence += 1
        final = os.path.join(self.directory, f"{seq:06d}-{segment_id}.avi")
        partial = final + ".partial"
        return _Open(segment_id, seq, AviMjpegWriter(partial, self.width, self.height, self.fps), partial, final,
                     slot_mono, slot_wall, gap_before=gap)

    def _close(self, seg: _Open) -> None:
        """Hands the segment to the finisher thread; the slot loop never waits for the disk."""
        self._finishing.put(seg)

    def _finish(self, seg: _Open) -> None:
        if seg.writer.frame_count == 0:
            seg.writer.abort()
            os.remove(seg.partial_path)
            return
        seg.writer.close(fsync=True)
        os.replace(seg.partial_path, seg.final_path)
        period_ns = 1_000_000_000 // self.fps
        frames = seg.writer.frame_count
        info = SegmentInfo(
            segment_id=seg.segment_id,
            recording_key=self.recording_key or "",
            sequence_number=seg.sequence_number,
            boot_session_id=self.boot_id,
            path=seg.final_path,
            sidecar_path=seg.final_path[:-4] + ".timing.json",
            captured_from_utc_ns=seg.first_slot_wall,
            captured_to_utc_ns=seg.last_slot_wall + period_ns,
            start_monotonic_ns=seg.first_slot_mono,
            end_monotonic_ns=seg.last_slot_mono + period_ns,
            duration_ms=frames * 1000 // self.fps,
            frame_count=frames,
            dropped_frame_count=seg.dropped,
            duplicated_frame_count=seg.duplicated,
            max_slot_error_ms=round(seg.max_err_ns / 1e6, 3),
            byte_size=os.path.getsize(seg.final_path),
            sha256=sha256_file(seg.final_path),
            width=self.width,
            height=self.height,
            fps=self.fps,
            gap_before=seg.gap_before,
        )
        sidecar = dict(asdict(info))
        # Per slot: offset of the slot from the segment start and the capture time of the frame in it (wall, ms).
        sidecar["slots"] = [[round((s - seg.first_slot_wall) / 1e6, 3), round((c - seg.first_slot_wall) / 1e6, 3)]
                            for s, c in seg.slots]
        tmp = info.sidecar_path + ".partial"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(sidecar, f)
            f.flush()
            os.fsync(f.fileno())
        os.replace(tmp, info.sidecar_path)
        try:
            self.on_segment(info)
        except Exception as exc:  # the file is safe on disk; the agent also scans the folder
            if self.log:
                self.log.warning(f"recorder: on_segment failed: {exc}")
