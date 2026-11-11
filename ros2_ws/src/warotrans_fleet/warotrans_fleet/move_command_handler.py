"""MOVE step: NAVIGATE_TO_POSE → Nav2 NavigateToPose, theo dõi goal tới khi có kết quả thật.

V1: tối đa MỘT move active. MOVE mới khi đang chạy → reject BUSY (không tự cancel goal cũ).
RESULT chỉ publish từ kết quả thật của Nav2 (hoặc khi Nav2 không phản hồi trong timeout bảo vệ);
cancel/timeout luôn đi qua cancel_goal_async và chờ Nav2 xác nhận.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Optional

from warotrans_fleet import navigation_state as nav_state
from warotrans_fleet.command_contract import (
    CANCEL,
    ERROR_CANCEL_INEFFECTIVE,
    ERROR_CANCEL_REJECTED,
    ERROR_CORRIDOR_VIOLATION,
    ERROR_GOAL_REJECTED,
    ERROR_GRANT_LOSS,
    ERROR_INTERNAL_ERROR,
    ERROR_NAV2_CANCEL_UNCONFIRMED,
    ERROR_NAV2_UNAVAILABLE,
    ERROR_NAVIGATION_BLOCKED,
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
from warotrans_fleet.route_enforcer import RouteEnforcer
from warotrans_fleet.command_router import AckDecision, CommandSession, LogSink

try:
    from warotrans_fleet.authority_store import parse_authority
    from warotrans_fleet.command_contract import REASON_AUTHORITY_REQUIRED
    from warotrans_fleet.traffic_client import TrafficClient
except ImportError:  # pragma: no cover
    parse_authority = None  # type: ignore
    REASON_AUTHORITY_REQUIRED = "AUTHORITY_REQUIRED"
    TrafficClient = None  # type: ignore
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
_SETTLING = "SETTLING"  # Nav2 đã xong; chờ robot đứng yên hẳn rồi mới đo sai số và báo RESULT

# TrafficClient state → navigationStatus cho telemetry (IDLE → dùng state legacy).
_TRAFFIC_NAV_STATUS = {
    "VALIDATING": nav_state.NAVIGATING,
    "EXECUTING": nav_state.NAVIGATING,
    "STOPPING": nav_state.NAVIGATING,
    "STOPPED_AWAITING": nav_state.PAUSED,
    "FAULTED": nav_state.FAILED,
}

# Lý do cancel
# nav2_msgs: controller (100–199) and planner (200–299). Obstacle / blocked path → wait+replan ≤60s, then PAUSED.
# RPP "collision ahead" is NO_VALID_CONTROL (106), or PATIENCE_EXCEEDED (104) once failure_tolerance is used up.
_NAV2_CONTROLLER_BLOCK_CODES = frozenset({104, 105, 106})  # PATIENCE_EXCEEDED, FAILED_TO_MAKE_PROGRESS, NO_VALID_CONTROL
# Moving this far counts as progress even while distance_remaining is above its best (a long detour after a replan).
_OBSTACLE_PROGRESS_MOVE_M = 0.20
_NAV2_PLANNER_ERROR_CODES = range(200, 300)

_CANCEL_BY_BACKEND = "BACKEND"
REASON_MOTION_HELD = "MOTION_HELD"
_CANCEL_BY_TIMEOUT = "TIMEOUT"
_CANCEL_BY_CORRIDOR = "CORRIDOR"
_CANCEL_BY_GRANT_LOSS = "GRANT_LOSS"
_CANCEL_BY_OBSTACLE = "OBSTACLE"


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
    traffic_retries: int = 0
    obstacle_retries: int = 0
    last_feedback: Optional[NavFeedback] = None
    last_progress_log_at: Optional[float] = None
    last_progress_at: Optional[float] = None
    last_distance_remaining: Optional[float] = None
    # Obstacle window: last_progress_at slides forward while the gate holds the robot, so a traffic wait is not counted.
    last_tick_at: Optional[float] = None
    progress_pose: Optional[tuple[float, float]] = None
    # First planner failure since the last progress. A closed gate cannot cause "no path", so this ignores holds.
    planner_failing_since: Optional[float] = None
    settle_until: Optional[float] = None
    succeeded_result: Optional[NavResult] = None
    # Arrival refine: AMCL is asked to update in place while the robot settles; a goal missed by more than the
    # refine thresholds is sent again (same target), at most arrival_max_refines times.
    refine_count: int = 0
    refreshes_left: int = 0
    next_refresh_at: Optional[float] = None

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
        route_enforcer: Optional[RouteEnforcer] = None,
        traffic_mode: str = "legacy",
        traffic_client: Optional["TrafficClient"] = None,
        expected_graph_hash: Optional[str] = None,
        traffic_hold_active: Callable[[], bool] = lambda: False,
        max_traffic_retries: int = 30,
        obstacle_wait_sec: float = 60.0,
        arrival_settle_sec: float = 0.0,
        arrival_pose: Callable[[], Optional[tuple[float, float, float]]] = lambda: None,
        motion_hold_active: Callable[[], bool] = lambda: False,
        localization_refresh: Callable[[], None] = lambda: None,
        arrival_refine_xy_m: float = 0.05,
        arrival_refine_yaw_rad: float = 0.087,
        arrival_max_refines: int = 0,
    ) -> None:
        self._navigator = navigator
        # 2026-11-10 lab: Nav2 passed its goal checker, then the next AMCL update moved the pose 0.16-0.28 m and
        # 26-52 deg. Asking AMCL for an in-place update before measuring shows the real miss; a miss is driven again.
        self._localization_refresh = localization_refresh
        self._arrival_refine_xy_m = float(arrival_refine_xy_m)
        self._arrival_refine_yaw_rad = float(arrival_refine_yaw_rad)
        self._arrival_max_refines = max(0, int(arrival_max_refines))
        self._arrival_pose = arrival_pose
        # semantic_active: the authority gate held the robot for traffic (no grant / outside reservation). Nav2's
        # progress checker then aborts the goal although nothing failed; such an abort re-sends the goal instead.
        self._traffic_hold_active = traffic_hold_active
        self._motion_hold_active = motion_hold_active
        self._max_traffic_retries = int(max_traffic_retries)
        self._obstacle_wait_sec = float(obstacle_wait_sec)
        # The goal checker passes while the robot still coasts: the heading that counts is the one it stops at.
        self._arrival_settle_sec = float(arrival_settle_sec)
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
        self._route_enforcer = route_enforcer
        self._traffic_mode = (traffic_mode or "legacy").strip().lower()
        self._traffic_client = traffic_client
        self._expected_graph_hash = expected_graph_hash
        # Active Mode: NAVIGATE session theo commandId — TrafficClient báo kết quả qua callback.
        self._traffic_sessions: dict[str, CommandSession] = {}
        self._traffic_cancel_sessions: dict[str, list[CommandSession]] = {}
        if traffic_client is not None and hasattr(traffic_client, "set_result_callback"):
            traffic_client.set_result_callback(self._on_traffic_result)
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
            if self._active_traffic_mode():
                return self._traffic_client.command_id  # type: ignore[union-attr]
            return self._active.command_id if self._active is not None else None

    @property
    def navigation_status(self) -> str:
        with self._lock:
            if self._active_traffic_mode():
                traffic_status = _TRAFFIC_NAV_STATUS.get(self._traffic_client.state)  # type: ignore[union-attr]
                if traffic_status is not None:
                    return traffic_status
            return self._state.status

    def _active_traffic_mode(self) -> bool:
        return self._traffic_mode == "active" and self._traffic_client is not None

    # ------------------------------------------------------------------ CommandHandler

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        with self._lock:
            # RB-03: after STOP nothing moves until RESUME, whoever asks.
            if self._motion_hold_active():
                raise CommandRejected(REASON_MOTION_HELD, "robot holds all motion after STOP; send RESUME first")
            if self._traffic_mode == "active" and self._traffic_client is not None and parse_authority is not None:
                return self._handle_active(command, session)

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

            if self._route_enforcer is not None and self._route_enforcer.enabled:
                self._route_enforcer.prepare_navigate(
                    dict(command.payload),
                    goal_x=target.x,
                    goal_y=target.y,
                    goal_yaw=target.yaw,
                    frame_id=target.frame_id,
                )

            move = _ActiveMove(
                command=command,
                session=session,
                target=target,
                timeout_sec=target.timeout_sec or self._default_timeout_sec,
            )
            self._active = move
            self._state.start()
            return AckDecision.accept(after_ack=lambda: self._send_goal(move))

    def _handle_active(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        if "authority" not in command.payload or command.payload.get("authority") is None:
            raise CommandRejected(REASON_AUTHORITY_REQUIRED, "active mode requires payload.authority")
        authority = parse_authority(command.payload, command_id=command.command_id)
        assert self._traffic_client is not None
        decision = self._traffic_client.start_navigate(
            command,
            authority,
            expected_map_version_code=self._robot_map_version_code,
            expected_graph_hash=self._expected_graph_hash,
        )
        if not decision.accepted:
            raise CommandRejected(decision.reason_code or REASON_AUTHORITY_REQUIRED, decision.message or "rejected")
        self._traffic_sessions[command.command_id] = session
        return AckDecision.accept(message=decision.message)

    def _on_traffic_result(self, command_id: str, outcome: str, error_code: Optional[str]) -> None:
        """TrafficClient kết thúc NAVIGATE (tới đích / fault / cancel) → command_result cho backend."""
        with self._lock:
            session = self._traffic_sessions.pop(command_id, None)
            cancel_sessions = self._traffic_cancel_sessions.pop(command_id, [])
        if session is None:
            self._log.warning(f"[TrafficResult] no session for commandId={command_id} outcome={outcome}")
        elif not session.is_completed:
            session.complete(outcome, error_code=error_code, result={"navigationResult": outcome})
        for cancel_session in cancel_sessions:
            if cancel_session.is_completed:
                continue
            if outcome == OUTCOME_CANCELED:
                cancel_session.complete(OUTCOME_SUCCEEDED, result={"targetCommandId": command_id})
            else:
                cancel_session.complete(
                    OUTCOME_FAILED,
                    error_code=ERROR_CANCEL_INEFFECTIVE,
                    error_message=f"target {command_id} ended with {outcome}/{error_code} before cancel took effect",
                )

    def tick(self) -> None:
        with self._lock:
            self._state.tick()
            move = self._active
            if move is None:
                return
            now = self._clock()

            if move.phase == _SETTLING:
                if move.refreshes_left > 0 and move.next_refresh_at is not None and now >= move.next_refresh_at:
                    move.refreshes_left -= 1
                    move.next_refresh_at = now + self._arrival_settle_sec / 3.0
                    self._refresh_localization()
                if move.settle_until is not None and now >= move.settle_until:
                    self._on_settled(move)
                return

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
                return

            if self._obstacle_window_expired(move, now):
                self._log.warning(
                    f"[NavigationBlockedByObstacle] commandId={move.command_id} no progress for "
                    f"{self._obstacle_wait_sec:.0f}s with the gate open; canceling Nav2 goal"
                )
                self._request_cancel(move, _CANCEL_BY_OBSTACLE, None)
                return

            if (
                self._route_enforcer is not None
                and self._route_enforcer.enabled
                and move.cancel_reason is None
                and move.phase in (_SENDING, _ACTIVE)
            ):
                violation = self._route_enforcer.monitor_tick(now)
                if violation == ERROR_CORRIDOR_VIOLATION:
                    self._log.warning(f"[CorridorViolation] commandId={move.command_id}; canceling")
                    self._request_cancel(move, _CANCEL_BY_CORRIDOR, None)
                elif violation == ERROR_GRANT_LOSS:
                    self._log.warning(f"[GrantLoss] commandId={move.command_id}; MQTT disconnect timeout")
                    self._request_cancel(move, _CANCEL_BY_GRANT_LOSS, None)

    # ------------------------------------------------------------------ STOP (MotionHold)

    def halt(self) -> Optional[str]:
        """Cancels the running MOVE without a CANCEL command (STOP). Returns its command id, or None if idle."""
        with self._lock:
            if self._active_traffic_mode():
                command_id = self._traffic_client.command_id  # type: ignore[union-attr]
                if command_id:
                    self._traffic_client.cancel_command(command_id)  # type: ignore[union-attr]
                return command_id
            move = self._active
        if move is None:
            return None
        self._request_cancel(move, _CANCEL_BY_BACKEND, None)
        return move.command_id

    # ------------------------------------------------------------------ CANCEL (từ CancelCommandHandler)

    def request_cancel(self, target_command_id: str, cancel_session: CommandSession) -> AckDecision:
        with self._lock:
            if self._active_traffic_mode():
                return self._request_cancel_active(target_command_id, cancel_session)
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

    def _request_cancel_active(self, target_command_id: str, cancel_session: CommandSession) -> AckDecision:
        client = self._traffic_client
        assert client is not None
        if client.command_id != target_command_id:
            raise CommandRejected(
                REASON_UNKNOWN_TARGET,
                f"targetCommandId {target_command_id} is not the active command (active={client.command_id})",
            )
        self._traffic_cancel_sessions.setdefault(target_command_id, []).append(cancel_session)

        def _cancel() -> None:
            was_faulted = client.state == "FAULTED"
            decision = client.cancel_command(target_command_id)
            if decision.accepted and was_faulted:
                # FAULTED → reset không phát result (FAILED đã báo); robot đã dừng → cancel coi như xong.
                with self._lock:
                    sessions = self._traffic_cancel_sessions.pop(target_command_id, [])
                for s in sessions:
                    if not s.is_completed:
                        s.complete(OUTCOME_SUCCEEDED, result={"targetCommandId": target_command_id})
                return
            if not decision.accepted:
                with self._lock:
                    sessions = self._traffic_cancel_sessions.get(target_command_id, [])
                    if cancel_session in sessions:
                        sessions.remove(cancel_session)
                cancel_session.complete(
                    OUTCOME_FAILED,
                    error_code=ERROR_CANCEL_REJECTED,
                    error_message=decision.message or "traffic client refused cancel",
                )

        return AckDecision.accept(after_ack=_cancel)

    # ------------------------------------------------------------------ Nav2 lifecycle

    def _send_goal(self, move: _ActiveMove) -> None:
        with self._lock:
            if self._active is not move:
                return
            now = self._clock()
            move.goal_sent_at = now
            if move.last_progress_at is None:
                move.last_progress_at = now  # a re-sent goal must not restart the obstacle window
                move.last_tick_at = now
            if move.timeout_sec is not None and move.deadline is None:
                # Once per command: a goal re-sent after a traffic hold keeps the original timeoutSeconds.
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
            dist = feedback.distance_remaining
            if not move.started:
                move.started = True
                move.last_progress_log_at = now
                self._mark_progress(move, now)
                move.last_distance_remaining = dist
                self._log.info(
                    f"[NavigationStarted] commandId={move.command_id} distanceRemaining="
                    f"{_fmt(dist)}"
                )
                return
            # Distance drop resets the obstacle wait window (robot made forward progress).
            if dist is not None and (
                move.last_distance_remaining is None or dist < move.last_distance_remaining - 0.05
            ):
                self._mark_progress(move, now)
                move.last_distance_remaining = dist
            if (
                move.last_progress_log_at is None
                or now - move.last_progress_log_at >= self._progress_log_interval_sec
            ):
                move.last_progress_log_at = now
                self._log.info(
                    f"[NavigationProgress] commandId={move.command_id} distanceRemaining="
                    f"{_fmt(dist)} recoveries={feedback.number_of_recoveries}"
                )

    def _on_result(self, move: _ActiveMove, result: NavResult) -> None:
        with self._lock:
            if self._active is not move:
                return
            status = result.status
            nav2_detail = f"nav2ErrorCode={result.error_code} nav2ErrorMsg={result.error_message!r}"

            if status == STATUS_SUCCEEDED:
                move.succeeded_result = result
                self._refresh_localization()
                if self._arrival_settle_sec > 0.0:
                    now = self._clock()
                    move.phase = _SETTLING
                    move.settle_until = now + self._arrival_settle_sec
                    # Three in-place AMCL updates in all while the robot comes to rest (one now, two during settle).
                    move.refreshes_left = 2
                    move.next_refresh_at = now + self._arrival_settle_sec / 3.0
                else:
                    self._on_settled(move)
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
            elif status == STATUS_CANCELED and move.cancel_reason == _CANCEL_BY_CORRIDOR:
                self._finish(
                    move,
                    OUTCOME_FAILED,
                    ERROR_CORRIDOR_VIOLATION,
                    "robot left authorized corridor buffer",
                    status,
                    "CORRIDOR_VIOLATION",
                    result,
                )
            elif status == STATUS_CANCELED and move.cancel_reason == _CANCEL_BY_GRANT_LOSS:
                self._finish(
                    move,
                    OUTCOME_FAILED,
                    ERROR_GRANT_LOSS,
                    "MQTT disconnected beyond grant_loss_timeout_sec",
                    status,
                    "GRANT_LOSS",
                    result,
                )
            elif status == STATUS_CANCELED and move.cancel_reason == _CANCEL_BY_OBSTACLE:
                self._finish_blocked(move, result, nav2_detail)
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
            elif status == STATUS_ABORTED and move.cancel_reason == _CANCEL_BY_OBSTACLE:
                self._finish_blocked(move, result, nav2_detail)
            elif status == STATUS_ABORTED and result.error_code in _NAV2_PLANNER_ERROR_CODES:
                # No path: the gate cannot cause this, so it is checked before the traffic hold.
                self._on_obstacle_abort(move, result, nav2_detail)
            elif status == STATUS_ABORTED and self._held_for_traffic(move):
                move.traffic_retries += 1
                self._log.warning(
                    f"[NavigationHeldForTraffic] commandId={move.command_id} Nav2 aborted while the authority gate "
                    f"held the robot; re-sending goal (retry {move.traffic_retries}/{self._max_traffic_retries}) {nav2_detail}"
                )
                move.ticket = GoalTicket()
                move.phase = _SENDING
                self._send_goal(move)
            elif (
                status == STATUS_ABORTED
                and self._is_obstacle_block_code(result.error_code)
                and not self._traffic_hold_is_active()
            ):
                self._on_obstacle_abort(move, result, nav2_detail)
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

    @staticmethod
    def _is_obstacle_block_code(error_code: Optional[int]) -> bool:
        if error_code is None:
            return False
        return error_code in _NAV2_CONTROLLER_BLOCK_CODES or error_code in _NAV2_PLANNER_ERROR_CODES

    def _refresh_localization(self) -> None:
        try:
            self._localization_refresh()
        except Exception as exc:  # noqa: BLE001 — a missing AMCL service must not change the result
            self._log.warning(f"[LocalizationRefreshFailed] {type(exc).__name__}: {exc}")

    def _on_settled(self, move: _ActiveMove) -> None:
        """Robot at rest after Nav2 SUCCEEDED: report, or drive the same goal again when it missed."""
        if self._needs_refine(move):
            now = self._clock()
            move.refine_count += 1
            self._log.warning(
                f"[ArrivalRefine] commandId={move.command_id} attempt={move.refine_count}/{self._arrival_max_refines}"
                f"{self._arrival_error_text(move)} limits xy={self._arrival_refine_xy_m:.3f}m "
                f"yaw={self._arrival_refine_yaw_rad:.3f}rad; re-sending the goal"
            )
            move.settle_until = None
            move.next_refresh_at = None
            move.refreshes_left = 0
            move.succeeded_result = None
            self._mark_progress(move, now)
            move.ticket = GoalTicket()
            move.phase = _SENDING
            self._send_goal(move)
            return
        self._finish_succeeded(move)

    def _arrival_out_of_tolerance(self, move: _ActiveMove) -> Optional[bool]:
        fields = self._arrival_error_fields(move)
        if not fields:
            return None
        return (
            fields["arrivalErrorXyM"] > self._arrival_refine_xy_m
            or fields["arrivalErrorYawRad"] > self._arrival_refine_yaw_rad
        )

    def _needs_refine(self, move: _ActiveMove) -> bool:
        if move.refine_count >= self._arrival_max_refines or move.cancel_reason is not None:
            return False
        if move.deadline is not None and self._clock() >= move.deadline:
            return False
        try:
            if self._motion_hold_active():
                return False
        except Exception:  # noqa: BLE001 — unknown hold state: do not drive
            return False
        return bool(self._arrival_out_of_tolerance(move))

    def _finish_succeeded(self, move: _ActiveMove) -> None:
        self._log.info(f"[NavigationSucceeded] commandId={move.command_id}{self._arrival_error_text(move)}")
        self._finish(move, OUTCOME_SUCCEEDED, None, None, STATUS_SUCCEEDED, "SUCCEEDED", move.succeeded_result)

    def _mark_progress(self, move: _ActiveMove, now: float) -> None:
        move.last_progress_at = now
        move.planner_failing_since = None
        move.progress_pose = None

    def _traffic_hold_is_active(self) -> bool:
        if self._traffic_mode != "semantic_active":
            return False
        try:
            return bool(self._traffic_hold_active())
        except Exception:  # noqa: BLE001 — unknown → not a hold
            return False

    def _obstacle_window_expired(self, move: _ActiveMove, now: float) -> bool:
        """Tick-driven, so the limit holds however long Nav2 spends in its own recovery tree before it aborts."""
        since_tick = now - move.last_tick_at if move.last_tick_at is not None else 0.0
        move.last_tick_at = now
        if move.last_progress_at is None:
            return False
        if self._traffic_hold_is_active():
            move.last_progress_at = min(now, move.last_progress_at + since_tick)
            return False
        try:
            pose = self._arrival_pose()
        except Exception:  # noqa: BLE001 — no pose: only distance_remaining counts
            pose = None
        if pose is not None:
            if move.progress_pose is None:
                move.progress_pose = (pose[0], pose[1])
            elif math.hypot(pose[0] - move.progress_pose[0], pose[1] - move.progress_pose[1]) >= _OBSTACLE_PROGRESS_MOVE_M:
                self._mark_progress(move, now)
                return False
        return (
            move.started
            and move.phase == _ACTIVE
            and move.cancel_reason is None
            and now - move.last_progress_at >= self._obstacle_wait_sec
        )

    def _on_obstacle_abort(self, move: _ActiveMove, result: NavResult, nav2_detail: str) -> None:
        now = self._clock()
        anchor = move.last_progress_at if move.last_progress_at is not None else now
        waited = now - anchor
        if result.error_code in _NAV2_PLANNER_ERROR_CODES:
            if move.planner_failing_since is None:
                move.planner_failing_since = now
            waited = max(waited, now - move.planner_failing_since)
        if waited < self._obstacle_wait_sec and (
            move.deadline is None or now < move.deadline
        ):
            move.obstacle_retries += 1
            self._log.warning(
                f"[NavigationBlockedByObstacle] commandId={move.command_id} waited={waited:.1f}s/"
                f"{self._obstacle_wait_sec:.0f}s; replan retry={move.obstacle_retries} {nav2_detail}"
            )
            move.ticket = GoalTicket()
            move.phase = _SENDING
            self._send_goal(move)
            return
        self._finish_blocked(move, result, nav2_detail)

    def _finish_blocked(self, move: _ActiveMove, result: NavResult, nav2_detail: str) -> None:
        self._log.warning(
            f"[NavigationBlockedByObstacle] commandId={move.command_id} reporting NAVIGATION_BLOCKED after "
            f"{self._obstacle_wait_sec:.0f}s without progress {nav2_detail}"
        )
        self._finish(
            move,
            OUTCOME_FAILED,
            ERROR_NAVIGATION_BLOCKED,
            f"blocked by obstacle for ≥{self._obstacle_wait_sec:.0f}s ({nav2_detail})",
            result.status,
            "NAVIGATION_BLOCKED",
            result,
        )

    def _arrival_error_fields(self, move: _ActiveMove) -> dict[str, float]:
        try:
            pose = self._arrival_pose()
        except Exception:  # noqa: BLE001
            return {}
        if pose is None:
            return {}
        t = move.target
        xy = math.hypot(pose[0] - t.x, pose[1] - t.y)
        yaw = abs(math.atan2(math.sin(pose[2] - t.yaw), math.cos(pose[2] - t.yaw)))
        return {"arrivalErrorXyM": round(xy, 4), "arrivalErrorYawRad": round(yaw, 4)}

    def _arrival_error_text(self, move: _ActiveMove) -> str:
        """Measured data for the goal tolerance decision (lab uses RESULT fields too)."""
        fields = self._arrival_error_fields(move)
        if not fields:
            return " arrivalError=unknown"
        return (
            f" arrivalErrorXy={fields['arrivalErrorXyM']:.3f}m "
            f"arrivalErrorYaw={fields['arrivalErrorYawRad']:.3f}rad"
        )

    def _held_for_traffic(self, move: _ActiveMove) -> bool:
        if self._traffic_mode != "semantic_active" or move.cancel_reason is not None:
            return False
        if move.traffic_retries >= self._max_traffic_retries:
            return False
        if move.deadline is not None and self._clock() >= move.deadline:
            return False
        try:
            return bool(self._traffic_hold_active())
        except Exception:  # noqa: BLE001 — unknown → report the abort
            return False

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
            if move.phase == _SETTLING:
                return  # Nav2 already finished: _finish answers the CANCEL as ineffective
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
        if self._route_enforcer is not None:
            self._route_enforcer.clear_active()
        self._state.finish(
            {
                OUTCOME_SUCCEEDED: nav_state.SUCCEEDED,
                OUTCOME_CANCELED: nav_state.CANCELED,
            }.get(outcome, nav_state.FAILED)
        )

        elapsed = None
        if move.goal_sent_at is not None:
            elapsed = round(self._clock() - move.goal_sent_at, 3)
        result: dict[str, Any] = {
            "navigationResult": navigation_result or ("FAILED" if outcome == OUTCOME_FAILED else outcome),
            "nav2Status": nav2_status,
            "nav2ErrorCode": nav_result.error_code if nav_result is not None else None,
            "elapsedSec": elapsed,
        }
        # Pose error after stop — logged for lab tolerance decisions; not a hard reject yet.
        arrival = self._arrival_error_fields(move)
        if arrival:
            result.update(arrival)
        if self._arrival_max_refines > 0 and outcome == OUTCOME_SUCCEEDED:
            result["refineCount"] = move.refine_count
            out = self._arrival_out_of_tolerance(move)
            if out is not None:
                result["arrivalOutOfTolerance"] = out
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
