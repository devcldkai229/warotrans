"""Map building from the web: START_MAPPING / STOP_MAPPING (pure Python, no ROS; the ROS side is mapping_ros.py).

A session, from the backend's point of view:

  START_MAPPING {sessionId}
      ack, then off the MQTT thread: pause localization (AMCL + map_server through their lifecycle manager, so only
      slam_toolbox owns map -> odom), start slam_toolbox in mapping mode, wait for its first map.
      SUCCEEDED {sessionId} once the map arrives; on any failure everything is put back and the result is FAILED.
  while MAPPING
      manual drive (teleop) is allowed even when the gateway's teleop is off for traffic, and every
      ``preview_period_sec`` the robot sends ``robots/{code}/mapping``: its pose in the new map and, when slam_toolbox
      published a newer map, a downsampled grid (zlib + base64) for the web to draw.
  STOP_MAPPING {sessionId, save: true}
      the full-resolution map becomes map.pgm (trinary, as map_saver writes it) + map.yaml, kept on the robot and sent
      in chunks on ``robots/{code}/mapping_upload`` (each chunk carries the metadata, the whole blob's sha256 and its
      index). Then slam_toolbox stops and localization resumes on the old map; the robot is NOT localized until someone
      localizes it again (its old pose means nothing after driving around). If sending fails the session stays open,
      so the operator can try again or cancel.
  STOP_MAPPING {sessionId, save: false}
      cancel: slam_toolbox stops, localization resumes, nothing is sent.
"""

from __future__ import annotations

import base64
import hashlib
import math
import threading
import time
import zlib
from dataclasses import dataclass
from typing import Any, Callable, Optional, Sequence

from warotrans_fleet.command_contract import (
    OUTCOME_FAILED,
    OUTCOME_SUCCEEDED,
    REASON_BUSY,
    REASON_INVALID_PAYLOAD,
    CommandEnvelope,
    CommandRejected,
)
from warotrans_fleet.command_router import AckDecision, CommandSession, LogSink
from warotrans_fleet.costmap_uplink import encode_grid

START_MAPPING = "START_MAPPING"
STOP_MAPPING = "STOP_MAPPING"

STATE_IDLE = "IDLE"
STATE_STARTING = "STARTING"
STATE_MAPPING = "MAPPING"
STATE_STOPPING = "STOPPING"

ERROR_PAUSE_LOCALIZATION = "MAPPING_LOCALIZATION_PAUSE_FAILED"
ERROR_SLAM_START = "MAPPING_SLAM_START_FAILED"
ERROR_NO_MAP = "MAPPING_NO_MAP"
ERROR_NOT_ACTIVE = "MAPPING_NOT_ACTIVE"
ERROR_UPLOAD = "MAPPING_UPLOAD_FAILED"
ERROR_RESUME_LOCALIZATION = "MAPPING_LOCALIZATION_RESUME_FAILED"

# map_saver trinary thresholds (nav2 map_saver defaults) and pixel values.
OCCUPIED_THRESH = 0.65
FREE_THRESH = 0.25
PIXEL_OCCUPIED = 0
PIXEL_FREE = 254
PIXEL_UNKNOWN = 205

UPLOAD_ENCODING = "zlib+base64"


@dataclass(frozen=True)
class Grid:
    """nav_msgs/OccupancyGrid, row-major with row 0 at the bottom (ROS order); values -1 or 0..100."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    data: Sequence[int]
    stamp: float  # monotonic time it was received, to tell a new map from the one already sent


# ------------------------------------------------------------------ map files


def to_pgm(grid: Grid) -> bytes:
    """P5 PGM, top row first (image order), map_saver trinary values."""
    w, h = grid.width, grid.height
    occ = OCCUPIED_THRESH * 100.0
    free = FREE_THRESH * 100.0
    out = bytearray(w * h)
    for row in range(h):
        src = (h - 1 - row) * w
        dst = row * w
        for col in range(w):
            v = grid.data[src + col]
            if v < 0:
                out[dst + col] = PIXEL_UNKNOWN
            elif v >= occ:
                out[dst + col] = PIXEL_OCCUPIED
            elif v <= free:
                out[dst + col] = PIXEL_FREE
            else:
                out[dst + col] = PIXEL_UNKNOWN
    return f"P5\n{w} {h}\n255\n".encode("ascii") + bytes(out)


def to_yaml(grid: Grid, image: str = "map.pgm") -> str:
    return (
        f"image: {image}\n"
        "mode: trinary\n"
        f"resolution: {grid.resolution:.6f}\n"
        f"origin: [{grid.origin_x:.6f}, {grid.origin_y:.6f}, {grid.origin_yaw:.6f}]\n"
        "negate: 0\n"
        f"occupied_thresh: {OCCUPIED_THRESH}\n"
        f"free_thresh: {FREE_THRESH}\n"
    )


def known_area_m2(grid: Grid) -> float:
    known = sum(1 for v in grid.data if v >= 0)
    return round(known * grid.resolution * grid.resolution, 2)


def upload_chunks(
    session_id: str,
    pgm: bytes,
    yaml_text: str,
    grid: Grid,
    chunk_bytes: int = 96_000,
    robot_path: Optional[str] = None,
) -> list[dict[str, Any]]:
    """The map as MQTT messages: zlib(pgm) split into base64 chunks, each one self-describing.

    ``chunk_bytes`` is the size of the compressed slice before base64 (x4/3), under AWS IoT Core's 128 KB per message.
    """
    blob = zlib.compress(pgm, 9)
    sha = hashlib.sha256(blob).hexdigest()
    size = max(1024, int(chunk_bytes))
    parts = [blob[i:i + size] for i in range(0, len(blob), size)] or [b""]
    meta = {
        "width": grid.width,
        "height": grid.height,
        "resolution": grid.resolution,
        "originX": grid.origin_x,
        "originY": grid.origin_y,
        "originYaw": grid.origin_yaw,
        "pgmBytes": len(pgm),
        "yaml": yaml_text,
        # With the map so the backend has them when the last chunk completes the session (before the STOP result).
        "knownAreaM2": known_area_m2(grid),
        "robotPath": robot_path,
    }
    return [
        {
            "sessionId": session_id,
            "index": i,
            "total": len(parts),
            "encoding": UPLOAD_ENCODING,
            "sha256": sha,
            "data": base64.b64encode(part).decode("ascii"),
            **meta,
        }
        for i, part in enumerate(parts)
    ]


def preview_body(
    session_id: str,
    seq: int,
    pose: Optional[tuple[float, float, float]],
    grid: Optional[Grid],
    max_cells: int = 400,
) -> dict[str, Any]:
    """The live view: the pose always, the grid only when there is a new one (max-pooled to ``max_cells`` a side)."""
    body: dict[str, Any] = {
        "sessionId": session_id,
        "seq": seq,
        "pose": None if pose is None else {"x": round(pose[0], 3), "y": round(pose[1], 3), "yaw": round(pose[2], 4)},
    }
    if grid is not None:
        cells, w, h, factor = encode_grid(grid.data, grid.width, grid.height, max_cells)
        body["map"] = {
            "width": w,
            "height": h,
            "resolution": grid.resolution * factor,
            "originX": grid.origin_x,
            "originY": grid.origin_y,
            "originYaw": grid.origin_yaw,
            "fullWidth": grid.width,
            "fullHeight": grid.height,
            "knownAreaM2": known_area_m2(grid),
            # bottom-up rows, 0..100 occupancy, 255 unknown
            "encoding": UPLOAD_ENCODING,
            "cells": base64.b64encode(zlib.compress(cells, 6)).decode("ascii"),
        }
    return body


# ------------------------------------------------------------------ the session


class MappingController:
    """One mapping session at a time; the two command handlers below drive it."""

    def __init__(
        self,
        *,
        pause_localization: Callable[[], Optional[str]],
        resume_localization: Callable[[], Optional[str]],
        start_slam: Callable[[str], Optional[str]],
        stop_slam: Callable[[], None],
        latest_map: Callable[[], Optional[Grid]],
        current_pose: Callable[[], Optional[tuple[float, float, float]]],
        publish_preview: Callable[[dict[str, Any]], None],
        publish_upload: Callable[[dict[str, Any]], bool],
        on_finished: Callable[[], None],
        log: LogSink,
        save_local: Optional[Callable[[str, bytes, str], Optional[str]]] = None,
        save_posegraph: Optional[Callable[[str], Optional[str]]] = None,
        run_async: Optional[Callable[[Callable[[], None]], None]] = None,
        first_map_timeout_sec: float = 30.0,
        preview_period_sec: float = 1.0,
        map_every_sec: float = 2.0,
        chunk_bytes: int = 96_000,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._pause = pause_localization
        self._resume = resume_localization
        self._start_slam = start_slam
        self._stop_slam = stop_slam
        self._latest_map = latest_map
        self._current_pose = current_pose
        self._publish_preview = publish_preview
        self._publish_upload = publish_upload
        self._on_finished = on_finished
        self._log = log
        self._save_local = save_local
        self._save_posegraph = save_posegraph
        self._run_async = run_async or (lambda fn: threading.Thread(target=fn, name="mapping", daemon=True).start())
        self._first_map_timeout = first_map_timeout_sec
        self._preview_period = preview_period_sec
        self._map_every = map_every_sec
        self._chunk_bytes = chunk_bytes
        self._now = monotonic
        self._sleep = sleep
        self._lock = threading.Lock()
        self._state = STATE_IDLE
        self._session_id: Optional[str] = None
        self._started_at = 0.0
        self._preview_stop = threading.Event()
        self._seq = 0
        self._sent_stamp = -math.inf
        self._sent_at = -math.inf

    # ------------------------------------------------------------------ read by the gateway

    @property
    def state(self) -> str:
        return self._state

    @property
    def session_id(self) -> Optional[str]:
        return self._session_id

    @property
    def active(self) -> bool:
        """True from START accepted to the end of STOP: no MOVE / LOCALIZE, teleop allowed, the gate open."""
        return self._state != STATE_IDLE

    @property
    def drivable(self) -> bool:
        return self._state == STATE_MAPPING

    # ------------------------------------------------------------------ START

    def start(self, session_id: str, busy: bool, session: CommandSession) -> AckDecision:
        with self._lock:
            if self._state == STATE_MAPPING and self._session_id == session_id:
                # The backend asked again (reconnect): the same session is already running.
                return AckDecision.accept(
                    after_ack=lambda: session.complete(OUTCOME_SUCCEEDED, result=self._result(session_id))
                )
            if self._state != STATE_IDLE:
                raise CommandRejected(REASON_BUSY, f"a mapping session is {self._state.lower()} ({self._session_id})")
            if busy:
                raise CommandRejected(REASON_BUSY, "a MOVE is running; stop or cancel it before mapping")
            self._state = STATE_STARTING
            self._session_id = session_id

        def _work() -> None:
            try:
                outcome, error, detail = self._do_start(session_id)
            except Exception as exc:  # never leave the command open
                self._log.error(f"[Mapping] start failed: {exc!r}")
                self._rollback()
                outcome, error, detail = OUTCOME_FAILED, "INTERNAL_ERROR", repr(exc)
            self._log.info(f"[Mapping] START {session_id} {outcome} error={error} detail={detail}")
            session.complete(outcome, error_code=error, error_message=detail, result=self._result(session_id))

        return AckDecision.accept(after_ack=lambda: self._run_async(_work))

    def _do_start(self, session_id: str) -> tuple[str, Optional[str], Optional[str]]:
        reason = self._pause()
        if reason is not None:
            self._resume()
            self._finish_idle()
            return OUTCOME_FAILED, ERROR_PAUSE_LOCALIZATION, reason
        asked = self._now()
        reason = self._start_slam(session_id)
        if reason is not None:
            self._rollback()
            return OUTCOME_FAILED, ERROR_SLAM_START, reason
        deadline = asked + self._first_map_timeout
        while self._now() < deadline:
            grid = self._latest_map()
            if grid is not None and grid.stamp >= asked:
                with self._lock:
                    self._state = STATE_MAPPING
                    self._started_at = self._now()
                self._start_preview(session_id)
                return OUTCOME_SUCCEEDED, None, None
            self._sleep(0.2)
        self._rollback()
        return OUTCOME_FAILED, ERROR_NO_MAP, f"slam_toolbox published no map within {self._first_map_timeout:.0f}s"

    # ------------------------------------------------------------------ STOP

    def stop(self, session_id: Optional[str], save: bool, session: CommandSession) -> AckDecision:
        with self._lock:
            if self._state == STATE_IDLE:
                if save:
                    raise CommandRejected(ERROR_NOT_ACTIVE, "no mapping session is running")
                # Cancel of a session the robot no longer has (restart): nothing to undo.
                return AckDecision.accept(
                    after_ack=lambda: session.complete(OUTCOME_SUCCEEDED, result={"sessionId": session_id, "saved": False})
                )
            if session_id and self._session_id and session_id != self._session_id:
                raise CommandRejected(REASON_INVALID_PAYLOAD, f"session {session_id} is not the running one ({self._session_id})")
            if self._state != STATE_MAPPING:
                raise CommandRejected(REASON_BUSY, f"mapping session is {self._state.lower()}")
            self._state = STATE_STOPPING
            current = self._session_id or ""

        def _work() -> None:
            try:
                outcome, error, detail, result = self._do_stop(current, save)
            except Exception as exc:
                self._log.error(f"[Mapping] stop failed: {exc!r}")
                with self._lock:
                    self._state = STATE_MAPPING  # still mapping: the operator may retry or cancel
                outcome, error, detail, result = OUTCOME_FAILED, "INTERNAL_ERROR", repr(exc), {"sessionId": current}
            self._log.info(f"[Mapping] STOP {current} save={save} {outcome} error={error} detail={detail}")
            session.complete(outcome, error_code=error, error_message=detail, result=result)

        return AckDecision.accept(after_ack=lambda: self._run_async(_work))

    def _do_stop(self, session_id: str, save: bool) -> tuple[str, Optional[str], Optional[str], dict[str, Any]]:
        result: dict[str, Any] = {"sessionId": session_id, "saved": False}
        if save:
            grid = self._latest_map()
            if grid is None:
                with self._lock:
                    self._state = STATE_MAPPING
                return OUTCOME_FAILED, ERROR_NO_MAP, "no map from slam_toolbox yet", result
            pgm = to_pgm(grid)
            yaml_text = to_yaml(grid)
            if self._save_local is not None:
                where = self._save_local(session_id, pgm, yaml_text)
                if where:
                    result["robotPath"] = where
            if self._save_posegraph is not None:
                reason = self._save_posegraph(session_id)
                result["posegraph"] = reason is None
                if reason is not None:
                    self._log.warning(f"[Mapping] pose graph not saved: {reason}")
            chunks = upload_chunks(session_id, pgm, yaml_text, grid, self._chunk_bytes, result.get("robotPath"))
            for chunk in chunks:
                if not self._publish_upload(chunk):
                    with self._lock:
                        self._state = STATE_MAPPING
                    return OUTCOME_FAILED, ERROR_UPLOAD, f"chunk {chunk['index'] + 1}/{chunk['total']} not queued", result
            result.update(
                saved=True,
                width=grid.width,
                height=grid.height,
                resolution=grid.resolution,
                originX=grid.origin_x,
                originY=grid.origin_y,
                knownAreaM2=known_area_m2(grid),
                sha256=chunks[0]["sha256"],
                chunks=len(chunks),
                pgmBytes=len(pgm),
            )
        reason = self._end_session()
        if reason is not None:
            return OUTCOME_FAILED, ERROR_RESUME_LOCALIZATION, reason, result
        return OUTCOME_SUCCEEDED, None, None, result

    # ------------------------------------------------------------------ helpers

    def shutdown(self) -> None:
        """Gateway exit: do not leave slam_toolbox running."""
        if self._state != STATE_IDLE:
            self._end_session()

    def _end_session(self) -> Optional[str]:
        self._preview_stop.set()
        self._stop_slam()
        reason = self._resume()
        self._finish_idle()
        return reason

    def _rollback(self) -> None:
        self._stop_slam()
        self._resume()
        self._finish_idle()

    def _finish_idle(self) -> None:
        with self._lock:
            self._state = STATE_IDLE
            self._session_id = None
        try:
            self._on_finished()
        except Exception as exc:  # noqa: BLE001
            self._log.warning(f"[Mapping] on_finished failed: {exc!r}")

    def _result(self, session_id: str) -> dict[str, Any]:
        return {"sessionId": session_id, "state": self._state}

    def _start_preview(self, session_id: str) -> None:
        self._preview_stop = threading.Event()
        stop = self._preview_stop
        self._run_async(lambda: self._preview_loop(session_id, stop))

    def _preview_loop(self, session_id: str, stop: threading.Event) -> None:
        self._sent_stamp = -math.inf
        self._sent_at = -math.inf
        while not stop.is_set() and self._state in (STATE_MAPPING, STATE_STOPPING):
            try:
                self.preview_tick(session_id)
            except Exception as exc:  # noqa: BLE001 — keep the live view going
                self._log.warning(f"[Mapping] preview failed: {exc!r}")
            stop.wait(self._preview_period)

    def preview_tick(self, session_id: str) -> dict[str, Any]:
        """One live-view message: the pose, plus the map if slam_toolbox has a newer one and it is time to send it."""
        grid = self._latest_map()
        now = self._now()
        send_map = grid is not None and grid.stamp > self._sent_stamp and now - self._sent_at >= self._map_every
        self._seq += 1
        body = preview_body(session_id, self._seq, self._current_pose(), grid if send_map else None)
        body["state"] = self._state
        body["elapsedSec"] = round(now - self._started_at, 1)
        self._publish_preview(body)
        if send_map and grid is not None:
            self._sent_stamp, self._sent_at = grid.stamp, now
        return body


class StartMappingHandler:
    command_type = START_MAPPING

    def __init__(self, controller: MappingController, busy: Callable[[], bool]) -> None:
        self._controller = controller
        self._busy = busy

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        payload = command.payload if isinstance(command.payload, dict) else {}
        session_id = payload.get("sessionId")
        if not isinstance(session_id, str) or not session_id.strip():
            raise CommandRejected(REASON_INVALID_PAYLOAD, "sessionId is required")
        return self._controller.start(session_id.strip(), self._busy(), session)


class StopMappingHandler:
    command_type = STOP_MAPPING

    def __init__(self, controller: MappingController) -> None:
        self._controller = controller

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        payload = command.payload if isinstance(command.payload, dict) else {}
        session_id = payload.get("sessionId")
        save = payload.get("save")
        if not isinstance(save, bool):
            raise CommandRejected(REASON_INVALID_PAYLOAD, "save must be true or false")
        return self._controller.stop(session_id.strip() if isinstance(session_id, str) else None, save, session)
