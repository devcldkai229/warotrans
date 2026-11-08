"""STOP / PAUSE / RESUME and SYSTEM_ACTION on the robot (RB-03, RB-09).

Pure Python (no rclpy) so it is unit tested like the rest of the command stack.

STOP (payload.mode STOP or PAUSE):
  1. ACK, then cancel the running goal (MoveCommandHandler.stop_active) and set the hold.
  2. While held, the gateway publishes zero velocity, reports ``motionHold: true`` and refuses NAVIGATE and teleop.
  3. RESULT SUCCEEDED (result.stoppedVerified=true) only once odometry has shown the robot standing still for
     ``settle_sec``; FAILED STOP_UNVERIFIED after ``verify_timeout_sec`` (the hold stays: never "stopped" on faith).
RESUME: release the hold; SUCCEEDED at once.
SYSTEM_ACTION: only when enabled on this robot, the hold is active and no goal runs; the action itself is a
callback (systemctl / reboot) owned by the node.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Optional

from warotrans_fleet.command_contract import (
    ERROR_STOP_UNVERIFIED,
    ERROR_SYSTEM_ACTION_FAILED,
    OUTCOME_FAILED,
    OUTCOME_SUCCEEDED,
    REASON_INVALID_PAYLOAD,
    REASON_MOTION_HELD,
    REASON_NOT_STOPPED,
    REASON_SYSTEM_ACTION_DISABLED,
    RESUME,
    STOP,
    STOP_MODE_PAUSE,
    STOP_MODE_STOP,
    SYSTEM_ACTION,
    SYSTEM_ACTIONS,
    CommandEnvelope,
    CommandRejected,
)
from warotrans_fleet.command_router import AckDecision, CommandSession

SpeedProvider = Callable[[], Optional[tuple[float, float]]]


@dataclass
class _PendingStop:
    session: CommandSession
    mode: str
    stopped_command_id: Optional[str]
    started_at: float
    still_since: Optional[float] = None


class MotionHold:
    """The hold itself plus the STOP verification. Thread-safe; ``tick()`` is called by the node's command timer."""

    def __init__(
        self,
        *,
        stop_active: Callable[[str], Optional[str]],
        speed_provider: SpeedProvider,
        clock: Callable[[], float],
        log,
        settle_sec: float = 0.5,
        verify_timeout_sec: float = 5.0,
        max_still_linear: float = 0.02,
        max_still_angular: float = 0.05,
    ) -> None:
        self._stop_active = stop_active
        self._speed = speed_provider
        self._clock = clock
        self._log = log
        self._settle_sec = settle_sec
        self._verify_timeout_sec = verify_timeout_sec
        self._max_linear = max_still_linear
        self._max_angular = max_still_angular
        self._lock = threading.RLock()
        self._held = False
        self._mode: Optional[str] = None
        self._reason: Optional[str] = None
        self._pending: list[_PendingStop] = []

    # ------------------------------------------------------------------ state

    @property
    def held(self) -> bool:
        with self._lock:
            return self._held

    @property
    def mode(self) -> Optional[str]:
        with self._lock:
            return self._mode if self._held else None

    def stop(self, session: CommandSession, mode: str, reason: Optional[str]) -> None:
        with self._lock:
            self._held = True
            self._mode = mode
            self._reason = reason
        stopped = self._stop_active(f"{mode}:{reason or 'operator'}")
        self._log.info(f"[MotionHold] {mode} held; canceled command={stopped} reason={reason}")
        with self._lock:
            self._pending.append(_PendingStop(session, mode, stopped, self._clock()))
        self.tick()

    def release(self) -> None:
        with self._lock:
            was = self._held
            self._held = False
            self._mode = None
            self._reason = None
            pending, self._pending = self._pending, []
        for p in pending:
            if not p.session.is_completed:
                p.session.complete(OUTCOME_FAILED, error_code=ERROR_STOP_UNVERIFIED, error_message="resumed before the stop was verified")
        if was:
            self._log.info("[MotionHold] released")

    def tick(self) -> None:
        """Completes pending STOPs: verified when still for ``settle_sec``, unverified after the timeout."""
        now = self._clock()
        speed = self._speed()
        finished: list[tuple[_PendingStop, bool]] = []
        with self._lock:
            for p in list(self._pending):
                still = speed is not None and abs(speed[0]) <= self._max_linear and abs(speed[1]) <= self._max_angular
                if still:
                    p.still_since = p.still_since if p.still_since is not None else now
                else:
                    p.still_since = None
                if p.still_since is not None and now - p.still_since >= self._settle_sec:
                    finished.append((p, True))
                elif now - p.started_at >= self._verify_timeout_sec:
                    finished.append((p, False))
            for p, _ in finished:
                self._pending.remove(p)
        for p, verified in finished:
            result = {"mode": p.mode, "stoppedVerified": verified, "stoppedCommandId": p.stopped_command_id}
            if verified:
                p.session.complete(OUTCOME_SUCCEEDED, result=result)
            else:
                p.session.complete(
                    OUTCOME_FAILED,
                    error_code=ERROR_STOP_UNVERIFIED,
                    error_message=(
                        "holding motion, but odometry never showed the robot still"
                        if speed is not None
                        else "holding motion, but no odometry to verify it"
                    ),
                    result=result,
                )

    def ensure_not_held(self) -> None:
        if self.held:
            raise CommandRejected(REASON_MOTION_HELD, "motion is held after STOP; send RESUME first")


class StopCommandHandler:
    command_type = STOP

    def __init__(self, hold: MotionHold) -> None:
        self._hold = hold

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        mode = str(command.payload.get("mode") or STOP_MODE_STOP).upper()
        if mode not in (STOP_MODE_STOP, STOP_MODE_PAUSE):
            raise CommandRejected(REASON_INVALID_PAYLOAD, f"mode must be STOP or PAUSE, got {mode}")
        reason = command.payload.get("reason")

        # A stop is never refused for being "busy": it is exactly what a busy robot must accept.
        return AckDecision.accept(after_ack=lambda: self._hold.stop(session, mode, reason if isinstance(reason, str) else None))


    def tick(self) -> None:
        self._hold.tick()


class ResumeCommandHandler:
    command_type = RESUME

    def __init__(self, hold: MotionHold) -> None:
        self._hold = hold

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        def _resume() -> None:
            self._hold.release()
            session.complete(OUTCOME_SUCCEEDED, result={"motionHold": False})

        return AckDecision.accept(after_ack=_resume)


class HoldGuard:
    """Wraps the NAVIGATE handler: a held robot accepts no new goal."""

    def __init__(self, inner, hold: MotionHold) -> None:
        self._inner = inner
        self._hold = hold
        self.command_type = inner.command_type

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        self._hold.ensure_not_held()
        return self._inner.handle(command, session)

    def tick(self) -> None:
        # The router ticks registered handlers: keep the wrapped move handler's timeouts running.
        inner_tick = getattr(self._inner, "tick", None)
        if inner_tick is not None:
            inner_tick()


class SystemActionHandler:
    """RESTART_STACK / REBOOT / SHUTDOWN. Off unless the node enables it (real robot with a sudoers rule)."""

    command_type = SYSTEM_ACTION

    def __init__(
        self,
        hold: MotionHold,
        *,
        enabled_actions: tuple[str, ...],
        goal_active: Callable[[], bool],
        run_action: Callable[[str], None],
    ) -> None:
        self._hold = hold
        self._enabled = tuple(a for a in enabled_actions if a in SYSTEM_ACTIONS)
        self._goal_active = goal_active
        self._run = run_action

    @property
    def capabilities(self) -> list[str]:
        return list(self._enabled)

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        action = str(command.payload.get("action") or "").upper()
        if action not in SYSTEM_ACTIONS:
            raise CommandRejected(REASON_INVALID_PAYLOAD, f"unknown action {action!r}")
        if action not in self._enabled:
            raise CommandRejected(REASON_SYSTEM_ACTION_DISABLED, f"{action} is not enabled on this robot")
        if not self._hold.held or self._goal_active():
            raise CommandRejected(REASON_NOT_STOPPED, "stop the robot first (motion must be held and no goal active)")

        def _run() -> None:
            # RESULT first: after REBOOT / SHUTDOWN the gateway is gone and could not report it.
            session.complete(OUTCOME_SUCCEEDED, result={"action": action})
            try:
                self._run(action)
            except Exception as exc:  # noqa: BLE001 - reported in the log; the RESULT is already out
                session_error = f"{type(exc).__name__}: {exc}"
                self._hold._log.error(f"[SystemAction] {action} failed: {session_error} ({ERROR_SYSTEM_ACTION_FAILED})")

        return AckDecision.accept(after_ack=_run)
