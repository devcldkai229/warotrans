"""MOVE step: NAVIGATE_TO_POSE → Nav2 NavigateToPose, theo dõi goal tới khi có kết quả thật.

V1: tối đa MỘT move active. MOVE mới khi đang chạy → reject BUSY (không tự cancel goal cũ).
RESULT chỉ publish từ kết quả thật của Nav2 (hoặc khi Nav2 không phản hồi trong timeout bảo vệ);
cancel/timeout luôn đi qua cancel_goal_async và chờ Nav2 xác nhận.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

from warotrans_fleet import navigation_state as nav_state
from warotrans_fleet.command_contract import (
    CANCEL,
    ERROR_CANCEL_INEFFECTIVE,
    ERROR_CANCEL_REJECTED,
    ERROR_GOAL_REJECTED,
    ERROR_INTERNAL_ERROR,
    ERROR_NAV2_CANCEL_UNCONFIRMED,
    ERROR_NAV2_UNAVAILABLE,
    ERROR_NAVIGATION_CANCELED,
    ERROR_NAVIGATION_FAILED,
    ERROR_NAVIGATION_TIMEOUT,
    NAVIGATE_TO_POSE,
    OUTCOME_CANCELED,
    OUTCOME_FAILED,
    OUTCOME_SUCCEEDED,
    REASON_BUSY,
    REASON_MAP_MISMATCH,
    REASON_NAV2_UNAVAILABLE,
    REASON_NOT_LOCALIZED,
    REASON_UNKNOWN_TARGET,
    CommandEnvelope,
    CommandRejected,
    NavigateTarget,
    parse_cancel_target,
    parse_navigate_target,
)
from warotrans_fleet.command_router import AckDecision, CommandSession, LogSink
from warotrans_fleet.navigator import (
    STATUS_ABORTED,
    STATUS_CANCELED,
    STATUS_SUCCEEDED,
    GoalCallbacks,
    GoalTicket,
    NavFeedback,
    NavGoal,
    NavigatorPort,
    NavResult,
)

# Phase của goal Nav2
_SENDING = "SENDING"  # đã gửi goal, chờ Nav2 accept/reject
_ACTIVE = "ACTIVE"  # Nav2 đã accept, robot đang chạy
_CANCELING = "CANCELING"  # đã gửi cancel_goal_async, chờ Nav2 kết thúc goal

# Lý do cancel
_CANCEL_BY_BACKEND = "BACKEND"
_CANCEL_BY_TIMEOUT = "TIMEOUT"


@dataclass
class _ActiveMove:
    command: CommandEnvelope
    session: CommandSession
    target: NavigateTarget
    timeout_sec: Optional[float]
    ticket: GoalTicket = field(default_factory=GoalTicket)
    phase: str = _SENDING
    goal_sent_at: Optional[float] = None
    deadline: Optional[float] = None
    cancel_reason: Optional[str] = None
    cancel_sent_at: Optional[float] = None
    cancel_sessions: list[CommandSession] = field(default_factory=list)
    started: bool = False
    last_feedback: Optional[NavFeedback] = None
    last_progress_log_at: Optional[float] = None

    @property
    def command_id(self) -> str:
        return self.command.command_id


class MoveCommandHandler:
    command_type = NAVIGATE_TO_POSE

    def __init__(
        self,
        *,
        navigator: NavigatorPort,
        localization_provider: Callable[[], str],
        log: LogSink,
        lock: Optional[threading.RLock] = None,
        clock: Callable[[], float] = time.monotonic,
        map_frame: str = "map",
        robot_map_version_code: Optional[str] = None,
        default_timeout_sec: Optional[float] = None,
        goal_response_timeout_sec: float = 10.0,
        cancel_confirm_timeout_sec: float = 10.0,
        terminal_status_hold_sec: float = 3.0,
        progress_log_interval_sec: float = 5.0,
        external_goal_active: Callable[[], bool] = lambda: False,
    ) -> None:
        self._navigator = navigator
        self._localization_provider = localization_provider
        self._log = log
        self._lock = lock if lock is not None else threading.RLock()
        self._clock = clock
        self._map_frame = map_frame
        self._robot_map_version_code = robot_map_version_code or None
        self._default_timeout_sec = default_timeout_sec if default_timeout_sec and default_timeout_sec > 0 else None
        self._goal_response_timeout_sec = float(goal_response_timeout_sec)
        self._cancel_confirm_timeout_sec = float(cancel_confirm_timeout_sec)
        self._progress_log_interval_sec = float(progress_log_interval_sec)
        self._external_goal_active = external_goal_active
        self._active: Optional[_ActiveMove] = None
        self._state = nav_state.NavigationStateMachine(
            clock=clock,
            terminal_hold_sec=terminal_status_hold_sec,
            on_change=lambda old, new: self._log.info(f"[NavigationState] {old} -> {new}"),
        )

    # ------------------------------------------------------------------ snapshot (telemetry)

    @property
    def active_command_id(self) -> Optional[str]:
        with self._lock:
            return self._active.command_id if self._active is not None else None

    @property
    def navigation_status(self) -> str:
        with self._lock:
            return self._state.status

    # ------------------------------------------------------------------ CommandHandler

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        with self._lock:
            if self._active is not None:
                raise CommandRejected(
                    REASON_BUSY, f"command {self._active.command_id} is still active; one MOVE at a time"
                )
            if self._external_goal_active():
                raise CommandRejected(
                    REASON_BUSY, "Nav2 is executing a goal not issued by fleet (e.g. RViz); refusing to preempt"
                )

            target = parse_navigate_target(command.payload, map_frame=self._map_frame)

            if (
                target.map_version_code is not None
                and self._robot_map_version_code is not None
                and target.map_version_code != self._robot_map_version_code
            ):
                raise CommandRejected(
                    REASON_MAP_MISMATCH,
                    f"command map '{target.map_version_code}' != robot map '{self._robot_map_version_code}'",
                )
            if target.map_version_code is not None and self._robot_map_version_code is None:
                self._log.warning(
                    f"[MapVersionUnchecked] commandId={command.command_id} map={target.map_version_code}; "
                    "robot map_version_code not configured"
                )

            localization = self._localization_provider()
            if localization != "LOCALIZED":
                raise CommandRejected(REASON_NOT_LOCALIZED, f"localizationStatus={localization}")

            if not self._navigator.is_ready():
                raise CommandRejected(REASON_NAV2_UNAVAILABLE, "NavigateToPose action server not ready")

            move = _ActiveMove(
                command=command,
                session=session,
                target=target,
                timeout_sec=target.timeout_sec or self._default_timeout_sec,
            )
            self._active = move
            self._state.start()
            return AckDecision.accept(after_ack=lambda: self._send_goal(move))

    def tick(self) -> None:
        with self._lock:
            self._state.tick()
            move = self._active
            if move is None:
                return
            now = self._clock()

            if move.phase == _SENDING and move.goal_sent_at is not None:
                if now - move.goal_sent_at >= self._goal_response_timeout_sec:
                    self._log.error(
                        f"[Nav2GoalNoResponse] commandId={move.command_id} no goal response in "
                        f"{self._goal_response_timeout_sec:.1f}s"
                    )
                    self._finish(
                        move,
                        OUTCOME_FAILED,
                        ERROR_NAV2_UNAVAILABLE,
                        f"Nav2 did not answer goal request within {self._goal_response_timeout_sec:.1f}s",
                        nav2_status=None,
                    )
                    return

            if move.phase == _CANCELING and move.cancel_sent_at is not None:
                if now - move.cancel_sent_at >= self._cancel_confirm_timeout_sec:
                    self._log.error(
                        f"[Nav2CancelUnconfirmed] commandId={move.command_id} goal did not finish "
                        f"{self._cancel_confirm_timeout_sec:.1f}s after cancel request"
                    )
                    self._finish(
                        move,
                        OUTCOME_FAILED,
                        ERROR_NAV2_CANCEL_UNCONFIRMED,
                        "Nav2 did not confirm goal termination after cancel; robot motion state unknown",
                        nav2_status=None,
                    )
                    return

            if move.deadline is not None and now >= move.deadline and move.cancel_reason is None:
                self._log.warning(
                    f"[NavigationTimeout] commandId={move.command_id} exceeded timeoutSeconds="
                    f"{move.timeout_sec}; canceling Nav2 goal"
                )
                self._request_cancel(move, _CANCEL_BY_TIMEOUT, None)

    # ------------------------------------------------------------------ CANCEL (từ CancelCommandHandler)

    def request_cancel(self, target_command_id: str, cancel_session: CommandSession) -> AckDecision:
        with self._lock:
            move = self._active
            if move is None or move.command_id != target_command_id:
                active = move.command_id if move is not None else None
                raise CommandRejected(
                    REASON_UNKNOWN_TARGET,
                    f"targetCommandId {target_command_id} is not the active command (active={active})",
                )
            return AckDecision.accept(
                after_ack=lambda: self._request_cancel(move, _CANCEL_BY_BACKEND, cancel_session)
            )

    # ------------------------------------------------------------------ Nav2 lifecycle

    def _send_goal(self, move: _ActiveMove) -> None:
        with self._lock:
            if self._active is not move:
                return
            now = self._clock()
            move.goal_sent_at = now
            if move.timeout_sec is not None:
                move.deadline = now + move.timeout_sec
            t = move.target
            goal = NavGoal(command_id=move.command_id, frame_id=t.frame_id, x=t.x, y=t.y, yaw=t.yaw)
            self._log.info(
                f"[Nav2GoalSent] commandId={move.command_id} frame={t.frame_id} x={t.x:.3f} y={t.y:.3f} "
                f"yaw={t.yaw:.4f} timeoutSec={move.timeout_sec}"
            )
            callbacks = GoalCallbacks(
                on_response=lambda accepted, error: self._on_goal_response(move, accepted, error),
                on_feedback=lambda feedback: self._on_feedback(move, feedback),
                on_result=lambda result: self._on_result(move, result),
            )
            try:
                self._navigator.send_goal(goal, move.ticket, callbacks)
            except Exception as exc:  # noqa: BLE001
                self._log.error(f"[Nav2GoalSendError] commandId={move.command_id} {type(exc).__name__}: {exc}")
                self._finish(
                    move,
                    OUTCOME_FAILED,
                    ERROR_NAV2_UNAVAILABLE,
                    f"send_goal failed: {type(exc).__name__}: {exc}",
                    nav2_status=None,
                )

    def _on_goal_response(self, move: _ActiveMove, accepted: bool, error: Optional[str]) -> None:
        with self._lock:
            if self._active is not move:
                if accepted:
                    # Response trễ sau khi đã báo FAILED — không để robot chạy goal mồ côi.
                    self._log.warning(f"[Nav2GoalOrphan] commandId={move.command_id} late accept; canceling")
                    self._navigator.cancel_goal(
                        move.ticket,
                        lambda ok, detail: self._log.warning(
                            f"[Nav2GoalOrphan] commandId={move.command_id} cancel accepted={ok} detail={detail}"
                        ),
                    )
                return

            if error is not None:
                self._log.error(f"[Nav2GoalRejected] commandId={move.command_id} transport error: {error}")
                self._finish(move, OUTCOME_FAILED, ERROR_NAV2_UNAVAILABLE, f"goal request failed: {error}", None)
                return
            if not accepted:
                t = move.target
                self._log.warning(f"[Nav2GoalRejected] commandId={move.command_id}")
                self._finish(
                    move,
                    OUTCOME_FAILED,
                    ERROR_GOAL_REJECTED,
                    f"Nav2 rejected goal x={t.x:.3f} y={t.y:.3f} yaw={t.yaw:.4f} frame={t.frame_id}",
                    None,
                )
                return

            move.phase = _ACTIVE
            self._log.info(f"[Nav2GoalAccepted] commandId={move.command_id}")
            if move.cancel_reason is not None:
                # Cancel/timeout đến trong lúc chờ accept — gửi cancel ngay bây giờ.
                self._send_nav2_cancel(move)

    def _on_feedback(self, move: _ActiveMove, feedback: NavFeedback) -> None:
        with self._lock:
            if self._active is not move:
                return
            move.last_feedback = feedback
            now = self._clock()
            if not move.started:
                move.started = True
                move.last_progress_log_at = now
                self._log.info(
                    f"[NavigationStarted] commandId={move.command_id} distanceRemaining="
                    f"{_fmt(feedback.distance_remaining)}"
                )
                return
            if (
                move.last_progress_log_at is None
                or now - move.last_progress_log_at >= self._progress_log_interval_sec
            ):
                move.last_progress_log_at = now
                self._log.info(
                    f"[NavigationProgress] commandId={move.command_id} distanceRemaining="
                    f"{_fmt(feedback.distance_remaining)} recoveries={feedback.number_of_recoveries}"
                )

    def _on_result(self, move: _ActiveMove, result: NavResult) -> None:
        with self._lock:
            if self._active is not move:
                return
            status = result.status
            nav2_detail = f"nav2ErrorCode={result.error_code} nav2ErrorMsg={result.error_message!r}"

            if status == STATUS_SUCCEEDED:
                self._log.info(f"[NavigationSucceeded] commandId={move.command_id}")
                self._finish(move, OUTCOME_SUCCEEDED, None, None, status, "SUCCEEDED", result)
            elif status == STATUS_CANCELED and move.cancel_reason == _CANCEL_BY_TIMEOUT:
                self._log.warning(f"[NavigationTimeout] commandId={move.command_id} goal canceled by Nav2")
                self._finish(
                    move,
                    OUTCOME_FAILED,
                    ERROR_NAVIGATION_TIMEOUT,
                    f"exceeded timeoutSeconds={move.timeout_sec}; Nav2 goal canceled",
                    status,
                    "TIMEOUT",
                    result,
                )
            elif status == STATUS_CANCELED:
                by = "backend CANCEL" if move.cancel_reason == _CANCEL_BY_BACKEND else "outside fleet (RViz/preempt)"
                self._log.info(f"[NavigationCanceled] commandId={move.command_id} by {by}")
                self._finish(
                    move,
                    OUTCOME_CANCELED,
                    ERROR_NAVIGATION_CANCELED,
                    f"Nav2 goal canceled by {by}",
                    status,
                    "CANCELED",
                    result,
                )
            elif status == STATUS_ABORTED:
                self._log.warning(f"[NavigationFailed] commandId={move.command_id} aborted {nav2_detail}")
                self._finish(
                    move,
                    OUTCOME_FAILED,
                    ERROR_NAVIGATION_FAILED,
                    f"Nav2 aborted goal ({nav2_detail})",
                    status,
                    "ABORTED",
                    result,
                )
            else:
                self._log.error(f"[NavigationFailed] commandId={move.command_id} status={status} {nav2_detail}")
                self._finish(
                    move,
                    OUTCOME_FAILED,
                    ERROR_NAVIGATION_FAILED if status else ERROR_INTERNAL_ERROR,
                    f"unexpected Nav2 terminal status {status} ({nav2_detail})",
                    status,
                    "UNKNOWN",
                    result,
                )

    def _request_cancel(self, move: _ActiveMove, reason: str, cancel_session: Optional[CommandSession]) -> None:
        with self._lock:
            if self._active is not move:
                if cancel_session is not None and not cancel_session.is_completed:
                    cancel_session.complete(
                        OUTCOME_FAILED,
                        error_code=ERROR_CANCEL_INEFFECTIVE,
                        error_message=f"command {move.command_id} already finished",
                    )
                return
            if cancel_session is not None:
                move.cancel_sessions.append(cancel_session)
            if move.cancel_reason is not None:
                self._log.info(
                    f"[Nav2CancelRequested] commandId={move.command_id} already canceling "
                    f"(reason={move.cancel_reason}); waiting for Nav2 result"
                )
                return
            move.cancel_reason = reason
            self._log.info(f"[Nav2CancelRequested] commandId={move.command_id} reason={reason}")
            if move.phase == _SENDING:
                self._log.info(
                    f"[Nav2CancelDeferred] commandId={move.command_id} goal response pending; "
                    "cancel will be sent once Nav2 accepts"
                )
                return
            self._send_nav2_cancel(move)

    def _send_nav2_cancel(self, move: _ActiveMove) -> None:
        move.phase = _CANCELING
        move.cancel_sent_at = self._clock()
        try:
            self._navigator.cancel_goal(
                move.ticket,
                lambda accepted, detail: self._on_cancel_response(move, accepted, detail),
            )
        except Exception as exc:  # noqa: BLE001
            self._on_cancel_response(move, False, f"cancel_goal failed: {type(exc).__name__}: {exc}")

    def _on_cancel_response(self, move: _ActiveMove, accepted: bool, detail: str) -> None:
        with self._lock:
            if self._active is not move:
                return
            if accepted:
                self._log.info(f"[Nav2CancelAccepted] commandId={move.command_id}; waiting for goal result")
                return
            # Nav2 từ chối cancel (vd. goal vừa kết thúc) — robot có thể vẫn chạy, chờ result thật.
            self._log.error(
                f"[Nav2CancelRejected] commandId={move.command_id} reason={move.cancel_reason} detail={detail}"
            )
            for cancel_session in move.cancel_sessions:
                if not cancel_session.is_completed:
                    cancel_session.complete(
                        OUTCOME_FAILED,
                        error_code=ERROR_CANCEL_REJECTED,
                        error_message=f"Nav2 rejected cancel: {detail}",
                    )
            move.cancel_sessions.clear()
            move.phase = _ACTIVE
            move.cancel_sent_at = None
            if move.cancel_reason == _CANCEL_BY_BACKEND:
                move.cancel_reason = None  # cho phép backend gửi CANCEL lại

    def _finish(
        self,
        move: _ActiveMove,
        outcome: str,
        error_code: Optional[str],
        error_message: Optional[str],
        nav2_status: Optional[int],
        navigation_result: Optional[str] = None,
        nav_result: Optional[NavResult] = None,
    ) -> None:
        if self._active is not move:
            return
        self._active = None
        self._state.finish(
            {
                OUTCOME_SUCCEEDED: nav_state.SUCCEEDED,
                OUTCOME_CANCELED: nav_state.CANCELED,
            }.get(outcome, nav_state.FAILED)
        )

        elapsed = None
        if move.goal_sent_at is not None:
            elapsed = round(self._clock() - move.goal_sent_at, 3)
        result = {
            "navigationResult": navigation_result or ("FAILED" if outcome == OUTCOME_FAILED else outcome),
            "nav2Status": nav2_status,
            "nav2ErrorCode": nav_result.error_code if nav_result is not None else None,
            "elapsedSec": elapsed,
        }
        move.session.complete(outcome, error_code=error_code, error_message=error_message, result=result)

        # CANCEL command chỉ SUCCEEDED khi Nav2 thực sự kết thúc goal ở trạng thái CANCELED.
        for cancel_session in move.cancel_sessions:
            if cancel_session.is_completed:
                continue
            if nav2_status == STATUS_CANCELED:
                cancel_session.complete(OUTCOME_SUCCEEDED, result={"targetCommandId": move.command_id})
            else:
                cancel_session.complete(
                    OUTCOME_FAILED,
                    error_code=ERROR_CANCEL_INEFFECTIVE,
                    error_message=f"target {move.command_id} ended with {outcome}/{error_code} before cancel took effect",
                )


class CancelCommandHandler:
    """CANCEL: chỉ áp dụng cho command MOVE đang active (targetCommandId)."""

    command_type = CANCEL

    def __init__(self, move_handler: MoveCommandHandler) -> None:
        self._move = move_handler

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        return self._move.request_cancel(parse_cancel_target(command.payload), session)


def _fmt(value: Optional[float]) -> str:
    return "n/a" if value is None else f"{value:.2f}"
