"""Map building from the web: localization off / slam on, live view, map sent in chunks, everything put back."""

import base64
import random
import hashlib
import logging
import zlib

from fleet_fakes import ROBOT, Recorder

from warotrans_fleet.command_router import CommandRouter
from warotrans_fleet.mapping_session import (
    STATE_IDLE,
    STATE_MAPPING,
    Grid,
    MappingController,
    StartMappingHandler,
    StopMappingHandler,
    preview_body,
    to_pgm,
    to_yaml,
    upload_chunks,
)

LOG = logging.getLogger("t")


def _grid(stamp=10.0, w=4, h=3):
    # bottom row: occupied, free, unknown, 50 (between thresholds → unknown)
    data = [100, 0, -1, 50] + [0] * w * (h - 1)
    return Grid(w, h, 0.05, -1.0, -2.0, 0.0, data[: w * h], stamp)


class Clock:
    def __init__(self):
        self.t = 0.0

    def now(self):
        return self.t

    def sleep(self, dt):
        self.t += dt


class Robot:
    def __init__(self):
        self.calls: list[str] = []
        self.grid = None
        self.map_after_start = _grid(stamp=5.0)
        self.pause_fails = None
        self.slam_fails = None
        self.upload_ok = True
        self.previews: list[dict] = []
        self.uploads: list[dict] = []
        self.finished = 0
        self.busy = False
        self.stuck_inactive = None

    def pause(self):
        self.calls.append("pause")
        return self.pause_fails

    def resume(self):
        self.calls.append("resume")
        return None

    def start_slam(self, session_id):
        self.calls.append(f"slam:{session_id}")
        if self.slam_fails:
            return self.slam_fails
        self.grid = self.map_after_start
        return None

    def stop_slam(self):
        self.calls.append("stop_slam")

    def ensure_active(self):
        # slam_toolbox stuck inactive (launch missed its lifecycle event) until the gateway activates it.
        if self.stuck_inactive is None:
            return None
        self.calls.append("activate")
        self.grid = self.stuck_inactive
        self.stuck_inactive = None
        return "slam_toolbox was left inactive: activate done by the gateway"

    def upload(self, body):
        self.uploads.append(body)
        return self.upload_ok

    def done(self):
        self.finished += 1


def _setup(robot: Robot):
    rec = Recorder()
    router = CommandRouter(robot_code=ROBOT, publisher=rec, log=LOG)
    clock = Clock()
    pending: list = []
    controller = MappingController(
        pause_localization=robot.pause,
        resume_localization=robot.resume,
        start_slam=robot.start_slam,
        stop_slam=robot.stop_slam,
        latest_map=lambda: robot.grid,
        current_pose=lambda: (1.0, 2.0, 0.5),
        publish_preview=robot.previews.append,
        publish_upload=robot.upload,
        on_finished=robot.done,
        log=LOG,
        save_local=lambda sid, pgm, yaml_text: f"/maps/{sid}",
        run_async=pending.append,
        first_map_timeout_sec=3.0,
        ensure_slam_active=robot.ensure_active,
        slam_nudge_after_sec=1.0,
        slam_nudge_every_sec=0.5,
        monotonic=clock.now,
        sleep=clock.sleep,
    )
    router.register(StartMappingHandler(controller, busy=lambda: robot.busy))
    router.register(StopMappingHandler(controller))

    def run():
        # The work runs on a thread in the gateway; here: one queued job at a time (the preview loop is left queued).
        while pending:
            job = pending.pop(0)
            if getattr(job, "__name__", "") == "<lambda>" and "preview" in repr(job.__code__.co_names):
                continue
            job()

    return rec, router, controller, run


def _cmd(command_id, type_, payload):
    return {"schemaVersion": 1, "messageId": "m", "commandId": command_id, "robotCode": ROBOT, "type": type_, "payload": payload}


def test_start_pauses_localization_starts_slam_and_waits_for_the_map():
    robot = Robot()
    rec, router, controller, run = _setup(robot)

    router.handle_message(_cmd("c1", "START_MAPPING", {"sessionId": "s-1"}))
    run()

    assert rec.acks[0]["accepted"] is True
    assert rec.results[0]["outcome"] == "SUCCEEDED"
    assert robot.calls == ["pause", "slam:s-1"]
    assert controller.state == STATE_MAPPING and controller.drivable and controller.active


def test_start_rolls_back_when_slam_never_publishes_a_map():
    robot = Robot()
    robot.map_after_start = None
    rec, router, controller, run = _setup(robot)

    router.handle_message(_cmd("c1", "START_MAPPING", {"sessionId": "s-1"}))
    run()

    assert rec.results[0]["outcome"] == "FAILED"
    assert rec.results[0]["errorCode"] == "MAPPING_NO_MAP"
    assert robot.calls == ["pause", "slam:s-1", "stop_slam", "resume"]
    assert controller.state == STATE_IDLE and robot.finished == 1


def test_start_rolls_back_when_slam_cannot_start():
    robot = Robot()
    robot.slam_fails = "no ros2"
    rec, router, controller, run = _setup(robot)

    router.handle_message(_cmd("c1", "START_MAPPING", {"sessionId": "s-1"}))
    run()

    assert rec.results[0]["errorCode"] == "MAPPING_SLAM_START_FAILED"
    assert robot.calls[-2:] == ["stop_slam", "resume"]
    assert controller.state == STATE_IDLE


def test_start_is_refused_while_moving_or_already_mapping():
    robot = Robot()
    robot.busy = True
    rec, router, controller, run = _setup(robot)
    router.handle_message(_cmd("c1", "START_MAPPING", {"sessionId": "s-1"}))
    assert rec.acks[0]["accepted"] is False
    assert robot.calls == []

    robot.busy = False
    router.handle_message(_cmd("c2", "START_MAPPING", {"sessionId": "s-1"}))
    run()
    router.handle_message(_cmd("c3", "START_MAPPING", {"sessionId": "s-2"}))
    assert rec.acks[-1]["accepted"] is False

    # The same session asked again (backend reconnect): answered at once, nothing restarted.
    router.handle_message(_cmd("c4", "START_MAPPING", {"sessionId": "s-1"}))
    run()
    assert rec.results[-1]["outcome"] == "SUCCEEDED"
    assert robot.calls.count("slam:s-1") == 1


def test_stop_with_save_sends_the_map_then_puts_localization_back():
    robot = Robot()
    rec, router, controller, run = _setup(robot)
    router.handle_message(_cmd("c1", "START_MAPPING", {"sessionId": "s-1"}))
    run()

    router.handle_message(_cmd("c2", "STOP_MAPPING", {"sessionId": "s-1", "save": True}))
    run()

    result = rec.results[-1]
    assert result["outcome"] == "SUCCEEDED"
    body = result["result"]
    assert body["saved"] is True and body["chunks"] == len(robot.uploads) >= 1
    assert body["width"] == 4 and body["height"] == 3 and body["robotPath"] == "/maps/s-1"
    assert robot.calls[-2:] == ["stop_slam", "resume"]
    assert controller.state == STATE_IDLE and robot.finished == 1

    blob = b"".join(base64.b64decode(c["data"]) for c in sorted(robot.uploads, key=lambda c: c["index"]))
    assert hashlib.sha256(blob).hexdigest() == body["sha256"]
    assert zlib.decompress(blob) == to_pgm(robot.grid)
    assert robot.uploads[0]["yaml"] == to_yaml(robot.grid)
    assert robot.uploads[0]["robotPath"] == "/maps/s-1" and robot.uploads[0]["knownAreaM2"] == body["knownAreaM2"]


def test_a_failed_upload_keeps_the_session_open():
    robot = Robot()
    rec, router, controller, run = _setup(robot)
    router.handle_message(_cmd("c1", "START_MAPPING", {"sessionId": "s-1"}))
    run()
    robot.upload_ok = False

    router.handle_message(_cmd("c2", "STOP_MAPPING", {"sessionId": "s-1", "save": True}))
    run()

    assert rec.results[-1]["errorCode"] == "MAPPING_UPLOAD_FAILED"
    assert controller.state == STATE_MAPPING
    assert "stop_slam" not in robot.calls

    # Cancel still works.
    router.handle_message(_cmd("c3", "STOP_MAPPING", {"sessionId": "s-1", "save": False}))
    run()
    assert rec.results[-1]["outcome"] == "SUCCEEDED"
    assert rec.results[-1]["result"]["saved"] is False
    assert controller.state == STATE_IDLE


def test_stop_checks_the_session_and_the_save_flag():
    robot = Robot()
    rec, router, controller, run = _setup(robot)

    router.handle_message(_cmd("c1", "STOP_MAPPING", {"sessionId": "s-1", "save": True}))
    assert rec.acks[-1]["accepted"] is False  # nothing to save

    router.handle_message(_cmd("c2", "STOP_MAPPING", {"sessionId": "s-1", "save": False}))
    run()
    assert rec.results[-1]["outcome"] == "SUCCEEDED"  # cancelling nothing is fine

    router.handle_message(_cmd("c3", "START_MAPPING", {"sessionId": "s-1"}))
    run()
    router.handle_message(_cmd("c4", "STOP_MAPPING", {"sessionId": "other", "save": False}))
    assert rec.acks[-1]["accepted"] is False
    router.handle_message(_cmd("c5", "STOP_MAPPING", {"sessionId": "s-1"}))
    assert rec.acks[-1]["accepted"] is False  # save missing


def test_pgm_is_top_row_first_and_trinary():
    grid = _grid()
    pgm = to_pgm(grid)
    header = b"P5\n4 3\n255\n"
    assert pgm.startswith(header)
    pixels = pgm[len(header):]
    assert len(pixels) == 12
    # bottom grid row is the last image row
    assert list(pixels[8:12]) == [0, 254, 205, 205]
    assert list(pixels[0:4]) == [254, 254, 254, 254]
    yaml_text = to_yaml(grid)
    assert "origin: [-1.000000, -2.000000, 0.000000]" in yaml_text and "resolution: 0.050000" in yaml_text


def test_chunks_fit_a_message_and_reassemble():
    pgm = random.Random(1).randbytes(300_000)  # incompressible: needs several chunks
    chunks = upload_chunks("s", pgm, "y", _grid(), chunk_bytes=50_000)
    assert len(chunks) > 1
    assert all(len(c["data"]) < 128_000 for c in chunks)
    assert [c["index"] for c in chunks] == list(range(len(chunks)))
    blob = b"".join(base64.b64decode(c["data"]) for c in chunks)
    assert zlib.decompress(blob) == pgm


def test_preview_carries_the_pose_and_the_grid_only_when_new():
    robot = Robot()
    rec, router, controller, run = _setup(robot)
    router.handle_message(_cmd("c1", "START_MAPPING", {"sessionId": "s-1"}))
    run()

    first = controller.preview_tick("s-1")
    second = controller.preview_tick("s-1")
    assert first["pose"] == {"x": 1.0, "y": 2.0, "yaw": 0.5}
    assert "map" in first and "map" not in second
    cells = zlib.decompress(base64.b64decode(first["map"]["cells"]))
    assert len(cells) == first["map"]["width"] * first["map"]["height"]

    body = preview_body("s", 1, None, None)
    assert body["pose"] is None and "map" not in body


def test_start_activates_slam_left_inactive_by_its_launch():
    """Lab 2026-11-26 (RBT-002, second session after a save): slam_toolbox configured but never activated."""
    robot = Robot()
    robot.stuck_inactive = robot.map_after_start
    robot.map_after_start = None
    rec, router, controller, run = _setup(robot)

    router.handle_message(_cmd("c1", "START_MAPPING", {"sessionId": "s-2"}))
    run()

    assert rec.results[0]["outcome"] == "SUCCEEDED"
    assert robot.calls == ["pause", "slam:s-2", "activate"]
    assert controller.state == STATE_MAPPING


def test_an_active_slam_is_never_touched():
    robot = Robot()
    rec, router, controller, run = _setup(robot)

    router.handle_message(_cmd("c1", "START_MAPPING", {"sessionId": "s-3"}))
    run()

    assert rec.results[0]["outcome"] == "SUCCEEDED"
    assert "activate" not in robot.calls
