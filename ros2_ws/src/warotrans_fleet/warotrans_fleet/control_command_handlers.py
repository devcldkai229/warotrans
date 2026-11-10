"""STOP / RESUME / LOCALIZE commands from the backend (RB-03, operator re-localize).

STOP holds every motion until RESUME: the hold forces authority_ok to false, so cmd_vel_authority_gate drops every
velocity whatever produces it, and the active MOVE is cancelled. A software hold is not an emergency stop.
LOCALIZE publishes /initialpose at a pose the backend took from a map endpoint; refused while a MOVE is running,
because moving the robot's belief under a running goal sends it somewhere else.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Optional

from warotrans_fleet.command_contract import (
    OUTCOME_SUCCEEDED,
    REASON_BUSY,
    CommandEnvelope,
    CommandRejected,
    parse_navigate_target,
)
from warotrans_fleet.command_router import AckDecision, CommandSession, LogSink

STOP = "STOP"
RESUME = "RESUME"
LOCALIZE = "LOCALIZE"

MODE_STOP = "STOP"
MODE_PAUSE = "PAUSE"


@dataclass
class _HoldState:
    held: bool = False
    mode: Optional[str] = None
    reason: Optional[str] = None


class MotionHold:
    """Whether the robot holds all motion. Thread-safe; read by the authority publisher at 20 Hz."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = _HoldState()

    @property
    def held(self) -> bool:
        with self._lock:
            return self._state.held

    @property
    def mode(self) -> Optional[str]:
        with self._lock:
            return self._state.mode if self._state.held else None

    def engage(self, mode: str, reason: Optional[str]) -> None:
        with self._lock:
            self._state = _HoldState(True, mode, reason)

    def release(self) -> None:
        with self._lock:
            self._state = _HoldState()


class StopCommandHandler:
    command_type = STOP

    def __init__(
        self,
        hold: MotionHold,
        halt: Callable[[], Optional[str]],
        on_hold_changed: Callable[[bool], None],
        log: LogSink,
    ) -> None:
        self._hold = hold
        self._halt = halt
        self._on_hold_changed = on_hold_changed
        self._log = log

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        mode = str(command.payload.get("mode") or MODE_STOP).upper()
        if mode not in (MODE_STOP, MODE_PAUSE):
            mode = MODE_STOP
        reason = command.payload.get("reason")
        reason = str(reason)[:200] if reason else None

        def _after_ack() -> None:
            # Hold first: from this instant the gate drops velocities, even before Nav2 has cancelled its goal.
            self._hold.engage(mode, reason)
            self._on_hold_changed(True)
            cancelled = self._halt()
            self._log.warning(
                f"[MotionHold] engaged mode={mode} reason={reason!r} cancelledCommandId={cancelled}"
            )
            session.complete(OUTCOME_SUCCEEDED, result={"mode": mode, "cancelledCommandId": cancelled})

        return AckDecision.accept(after_ack=_after_ack)


class ResumeCommandHandler:
    command_type = RESUME

    def __init__(self, hold: MotionHold, on_hold_changed: Callable[[bool], None], log: LogSink) -> None:
        self._hold = hold
        self._on_hold_changed = on_hold_changed
        self._log = log

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        def _after_ack() -> None:
            was = self._hold.held
            self._hold.release()
            self._on_hold_changed(False)
            self._log.info(f"[MotionHold] released (was held={was})")
            session.complete(OUTCOME_SUCCEEDED, result={"wasHeld": was})

        return AckDecision.accept(after_ack=_after_ack)


class LocalizeCommandHandler:
    command_type = LOCALIZE

    def __init__(
        self,
        publish_initial_pose: Callable[[float, float, float], None],
        busy: Callable[[], bool],
        log: LogSink,
        map_frame: str = "map",
    ) -> None:
        self._publish = publish_initial_pose
        self._busy = busy
        self._log = log
        self._map_frame = map_frame

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        if self._busy():
            raise CommandRejected(REASON_BUSY, "a MOVE is running; stop or cancel it before re-localizing")
        target = parse_navigate_target(command.payload, map_frame=self._map_frame)

        def _after_ack() -> None:
            self._publish(target.x, target.y, target.yaw)
            self._log.info(
                f"[Localize] initial pose published x={target.x:.3f} y={target.y:.3f} yaw={target.yaw:.4f} "
                f"endpoint={command.payload.get('endpointCode')}"
            )
            session.complete(OUTCOME_SUCCEEDED, result={"x": target.x, "y": target.y, "yaw": target.yaw})

        return AckDecision.accept(after_ack=_after_ack)
