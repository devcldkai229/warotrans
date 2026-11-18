"""AVI writer and RecorderSink without a camera: fake frames, a fake clock."""

from __future__ import annotations

import hashlib
import json
import os
import threading

import pytest

from warotrans_camera.avi_writer import AviMjpegWriter, read_avi_index
from warotrans_camera.recorder_sink import RecorderSink


def jpeg(n: int, size: int = 40) -> bytes:
    body = bytes([n % 251]) * (size + n % 3)  # odd and even sizes (word alignment)
    return b"\xff\xd8" + body + b"\xff\xd9"


def test_avi_round_trip_keeps_every_frame_rate_and_size(tmp_path):
    path = str(tmp_path / "a.avi")
    w = AviMjpegWriter(path, 1280, 720, 15)
    frames = [jpeg(i) for i in range(31)]
    for f in frames:
        w.write_frame(f)
    w.close()
    avi = read_avi_index(path)
    assert avi["fps"] == 15
    assert (avi["width"], avi["height"]) == (1280, 720)
    assert avi["total_frames"] == avi["length"] == 31
    assert avi["has_index"]
    assert avi["frames"] == frames
    assert avi["micro_sec_per_frame"] == 66667


def test_avi_refuses_what_is_not_a_jpeg(tmp_path):
    w = AviMjpegWriter(str(tmp_path / "b.avi"), 640, 480, 10)
    with pytest.raises(ValueError):
        w.write_frame(b"not a jpeg")
    w.close()


class FakeCamera:
    """Frames on demand; the test decides when a new one 'arrives'."""

    def __init__(self) -> None:
        self.frame = None
        self.seq = 0
        self.mono = 0
        self.wall = 0
        self.viewers = 0

    def add_viewer(self):
        self.viewers += 1

    def remove_viewer(self):
        self.viewers -= 1

    def latest(self):
        return self.frame, self.seq, self.mono, self.wall


class FakeClock:
    """Monotonic and wall clocks driven by sleep(); every slot the camera may get a new frame via on_tick."""

    WALL0 = 1_763_348_400_000_000_000  # 2025-11-17T03:00:00Z in ns

    def __init__(self, cam: FakeCamera, on_tick, stop_after_s: float, sink_ref) -> None:
        self.t = 1_000_000_000
        self.cam, self.on_tick, self.stop_after = cam, on_tick, stop_after_s
        self.sink_ref = sink_ref

    def mono(self) -> int:
        return self.t

    def wall(self) -> int:
        return self.WALL0 + self.t

    def sleep(self, seconds: float) -> None:
        self.t += int(seconds * 1e9)
        self.on_tick(self)
        if self.t > self.stop_after * 1e9 + 1_000_000_000:
            self.sink_ref[0]._stop.set()


def run_sink(tmp_path, on_tick, seconds: float, segment_seconds: float = 2.0, fps: int = 10):
    cam = FakeCamera()
    segments = []
    ref = [None]
    clock = FakeClock(cam, on_tick, seconds, ref)
    sink = RecorderSink(cam, segments.append, 640, 480, fps, segment_seconds,
                        clock_monotonic_ns=clock.mono, clock_wall_ns=clock.wall, sleep=clock.sleep)
    ref[0] = sink
    on_tick(clock)
    sink.start("rec-key", str(tmp_path / "rec"))
    sink._thread.join(10)
    sink.stop()
    assert cam.viewers == 0, "the sink gives its viewer back"
    return segments


def steady(fps: int):
    """A camera delivering exactly at the slot rate, each frame stamped 3 ms before the slot."""
    def tick(clock: FakeClock):
        clock.cam.seq += 1
        clock.cam.frame = jpeg(clock.cam.seq)
        clock.cam.mono = clock.t - 3_000_000
        clock.cam.wall = clock.wall() - 3_000_000
    return tick


def test_steady_camera_makes_full_cfr_segments_with_measured_slot_error(tmp_path):
    segments = run_sink(tmp_path, steady(10), seconds=6.0)
    assert len(segments) >= 3
    first = segments[0]
    assert first.frame_count == 20 and first.duration_ms == 2000
    assert first.dropped_frame_count == 0 and first.duplicated_frame_count == 0
    assert first.max_slot_error_ms == pytest.approx(3.0, abs=0.01)
    assert [s.sequence_number for s in segments] == list(range(len(segments)))
    # back to back on the capture clock
    assert segments[1].captured_from_utc_ns == first.captured_to_utc_ns
    for s in segments:
        assert os.path.exists(s.path) and not os.path.exists(s.path + ".partial")
        assert hashlib.sha256(open(s.path, "rb").read()).hexdigest() == s.sha256
        assert os.path.getsize(s.path) == s.byte_size
        assert read_avi_index(s.path)["total_frames"] == s.frame_count
        side = json.load(open(s.sidecar_path, encoding="utf-8"))
        assert side["segment_id"] == s.segment_id and len(side["slots"]) == s.frame_count


def test_a_slow_camera_is_padded_with_repeats_and_a_fast_one_is_thinned(tmp_path):
    def half_rate(clock: FakeClock):
        if (clock.t // 100_000_000) % 2 == 0:  # a new frame every other slot
            steady(10)(clock)

    slow = run_sink(tmp_path / "slow", half_rate, seconds=4.0)
    assert slow[0].duplicated_frame_count >= 9

    def double_rate(clock: FakeClock):
        steady(10)(clock)
        clock.cam.seq += 1  # one more frame between two slots: it is skipped
    fast = run_sink(tmp_path / "fast", double_rate, seconds=4.0)
    assert fast[1].dropped_frame_count >= 19


def test_a_camera_that_stops_closes_the_segment_and_marks_a_gap(tmp_path):
    def stops_at_1s(clock: FakeClock):
        if clock.t < 2_000_000_000:  # frames for the first second only, then again after 4 s
            steady(10)(clock)
        elif clock.t > 5_000_000_000:
            steady(10)(clock)

    segments = run_sink(tmp_path, stops_at_1s, seconds=7.0, segment_seconds=10.0)
    assert len(segments) >= 2
    assert segments[0].gap_before is None
    assert segments[1].gap_before == "CAMERA_LOST"
    # The frozen last frame is repeated only until the loss is noticed (1 s); after that the time is a hole.
    assert segments[0].captured_to_utc_ns - FakeClock.WALL0 <= 3_200_000_000
    assert segments[1].captured_from_utc_ns - segments[0].captured_to_utc_ns > 1_500_000_000
    assert segments[0].duplicated_frame_count >= 9


def test_start_with_the_same_key_twice_keeps_one_recording(tmp_path):
    cam = FakeCamera()
    cam.frame, cam.seq = jpeg(1), 1
    sink = RecorderSink(cam, lambda s: None, 64, 48, 5, 1.0)
    sink.start("k", str(tmp_path))
    thread = sink._thread
    sink.start("k", str(tmp_path))
    assert sink._thread is thread and cam.viewers == 1
    sink.stop()
    assert cam.viewers == 0
