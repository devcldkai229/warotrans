"""CommandRouter: validate envelope + dedup theo commandId + dispatch tới handler theo command type.

Router không biết Nav2 hay MQTT: handler xử lý nghiệp vụ kỹ thuật, `CommandEventPublisher`
đẩy ACK/RESULT ra ngoài. Thêm step type mới (CHECK/WAIT/...) = đăng ký handler mới,
không sửa MQTT core.

Threading: router được gọi trên ROS executor thread (node chuyển message từ MQTT thread
sang qua inbox). Router và handler dùng chung một RLock để callback Nav2 → session.complete()
không deadlock với router → handler.handle().
"""

from __future__ import annotations

import threading
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Any, Callable, Mapping, Optional, Protocol

from warotrans_fleet.command_contract import (
    ERROR_INTERNAL_ERROR,
    OUTCOME_FAILED,
    REASON_ALREADY_ACCEPTED,
    REASON_INTERNAL_ERROR,
    REASON_INVALID_PAYLOAD,
    REASON_UNSUPPORTED_COMMAND,
    CommandEnvelope,
    CommandRejected,
    extract_command_id,
    parse_envelope,
)
from warotrans_fleet.payload_builder import build_command_ack, build_command_result


class LogSink(Protocol):
    """Tương thích cả rclpy logger lẫn logging.Logger (chỉ truyền string đã format)."""

    def info(self, message: str) -> Any: ...
    def warning(self, message: str) -> Any: ...
    def error(self, message: str) -> Any: ...


class CommandEventPublisher(Protocol):
    def publish_ack(self, payload: dict[str, Any]) -> None: ...
    def publish_result(self, payload: dict[str, Any]) -> None: ...


@dataclass
class AckDecision:
    accepted: bool
    reason_code: Optional[str] = None
    message: Optional[str] = None
    # Chạy SAU khi ACK đã publish (vd. gửi goal Nav2) — đảm bảo ACK luôn đi trước RESULT.
    after_ack: Optional[Callable[[], None]] = None

    @staticmethod
    def accept(*, after_ack: Optional[Callable[[], None]] = None, message: Optional[str] = None) -> "AckDecision":
        return AckDecision(True, None, message, after_ack)

    @staticmethod
    def reject(reason_code: str, message: str) -> "AckDecision":
        return AckDecision(False, reason_code, message)


class CommandHandler(Protocol):
    command_type: str

    def handle(self, command: CommandEnvelope, session: "CommandSession") -> AckDecision:
        """Quyết định accept/reject nhanh, không block. Raise CommandRejected để reject."""
        ...


# Ledger states
_RECEIVED = "RECEIVED"
_REJECTED = "REJECTED"
_RUNNING = "RUNNING"
_COMPLETED = "COMPLETED"


@dataclass
class _LedgerEntry:
    state: str = _RECEIVED
    ack: Optional[dict[str, Any]] = None
    result: Optional[dict[str, Any]] = None


@dataclass
class CommandSession:
    """Lifecycle của một command đã được accept. Handler gọi complete() đúng một lần."""

    command: CommandEnvelope
    _router: "CommandRouter" = field(repr=False)
    _entry: _LedgerEntry = field(repr=False)
    _ack_published: bool = False
    _completed: bool = False
    _pending_result: Optional[dict[str, Any]] = None

    @property
    def is_completed(self) -> bool:
        return self._completed

    def complete(
        self,
        outcome: str,
        *,
        error_code: Optional[str] = None,
        error_message: Optional[str] = None,
        result: Optional[Mapping[str, Any]] = None,
    ) -> bool:
        with self._router.lock:
            if self._completed:
                self._router.log.warning(
                    f"[CommandResultIgnored] commandId={self.command.command_id} already completed; "
                    f"dropping outcome={outcome}"
                )
                return False
            self._completed = True
            payload = build_command_result(
                robot_code=self._router.robot_code,
                command_id=self.command.command_id,
                outcome=outcome,
                error_code=error_code,
                error_message=error_message,
                job_assignment_id=self.command.job_assignment_id,
                job_step_id=self.command.job_step_id,
                result=result,
            )
            self._entry.state = _COMPLETED
            self._entry.result = payload
            if self._ack_published:
                self._router._publish_result(payload)
            else:
                self._pending_result = payload
            return True

    def _mark_ack_published(self) -> None:
        self._ack_published = True
        if self._pending_result is not None:
            payload, self._pending_result = self._pending_result, None
            self._router._publish_result(payload)


class CommandRouter:
    def __init__(
        self,
        *,
        robot_code: str,
        publisher: CommandEventPublisher,
        log: LogSink,
        lock: Optional[threading.RLock] = None,
        ledger_capacity: int = 500,
    ) -> None:
        self.robot_code = robot_code
        self.log = log
        self.lock = lock if lock is not None else threading.RLock()
        self._publisher = publisher
        self._handlers: dict[str, CommandHandler] = {}
        self._ledger: "OrderedDict[str, _LedgerEntry]" = OrderedDict()
        self._ledger_capacity = max(1, int(ledger_capacity))

    def register(self, handler: CommandHandler) -> None:
        if handler.command_type in self._handlers:
            raise ValueError(f"handler already registered for {handler.command_type}")
        self._handlers[handler.command_type] = handler

    @property
    def supported_types(self) -> tuple[str, ...]:
        return tuple(self._handlers)

    def handle_message(self, body: Any) -> None:
        with self.lock:
            self._handle(body)

    def tick(self) -> None:
        """Gọi định kỳ (timer) cho handler cần theo dõi timeout/deadline."""
        with self.lock:
            for handler in self._handlers.values():
                tick = getattr(handler, "tick", None)
                if tick is None:
                    continue
                try:
                    tick()
                except Exception as exc:  # noqa: BLE001 — log đầy đủ, không làm chết timer
                    self.log.error(
                        f"[CommandTickError] handler={handler.command_type} {type(exc).__name__}: {exc}"
                    )

    # ------------------------------------------------------------------ internals

    def _handle(self, body: Any) -> None:
        command_id = extract_command_id(body)
        command_type = body.get("type") if isinstance(body, Mapping) else None
        self.log.info(f"[CommandReceived] commandId={command_id} type={command_type}")

        if command_id is None:
            # Không có commandId thì backend không correlate được ACK — chỉ log.
            self.log.error("[CommandDropped] missing/invalid 'commandId'; cannot ACK")
            return

        existing = self._ledger.get(command_id)
        if existing is not None:
            self._replay_duplicate(command_id, existing)
            return

        entry = _LedgerEntry()
        self._remember(command_id, entry)

        try:
            command = parse_envelope(body, expected_robot_code=self.robot_code)
        except CommandRejected as rej:
            self._reject(command_id, body, entry, rej.reason_code, rej.message)
            return

        handler = self._handlers.get(command.command_type)
        if handler is None:
            self._reject(
                command_id,
                body,
                entry,
                REASON_UNSUPPORTED_COMMAND,
                f"type '{command.command_type}' not supported; supported={list(self._handlers)}",
            )
            return

        session = CommandSession(command=command, _router=self, _entry=entry)
        try:
            decision = handler.handle(command, session)
        except CommandRejected as rej:
            decision = AckDecision.reject(rej.reason_code, rej.message)
        except Exception as exc:  # noqa: BLE001 — trả INTERNAL_ERROR thay vì nuốt lỗi
            self.log.error(
                f"[CommandHandlerError] commandId={command_id} {type(exc).__name__}: {exc}"
            )
            decision = AckDecision.reject(REASON_INTERNAL_ERROR, f"{type(exc).__name__}: {exc}")

        if not decision.accepted:
            self._reject(
                command_id,
                body,
                entry,
                decision.reason_code or REASON_INVALID_PAYLOAD,
                decision.message or "",
            )
            return

        entry.state = _RUNNING if not session.is_completed else _COMPLETED
        ack = build_command_ack(
            robot_code=self.robot_code,
            command_id=command_id,
            accepted=True,
            reason_message=decision.message,
            job_assignment_id=command.job_assignment_id,
            job_step_id=command.job_step_id,
        )
        entry.ack = ack
        self._publisher.publish_ack(ack)
        self.log.info(
            f"[CommandAccepted] commandId={command_id} type={command.command_type} "
            f"jobAssignmentId={command.job_assignment_id} jobStepId={command.job_step_id}"
        )
        session._mark_ack_published()

        if decision.after_ack is not None:
            try:
                decision.after_ack()
            except Exception as exc:  # noqa: BLE001
                self.log.error(
                    f"[CommandHandlerError] commandId={command_id} after_ack "
                    f"{type(exc).__name__}: {exc}"
                )
                if not session.is_completed:
                    session.complete(
                        OUTCOME_FAILED,
                        error_code=ERROR_INTERNAL_ERROR,
                        error_message=f"{type(exc).__name__}: {exc}",
                    )

    def _reject(
        self,
        command_id: str,
        body: Any,
        entry: _LedgerEntry,
        reason_code: str,
        message: str,
    ) -> None:
        raw = body if isinstance(body, Mapping) else {}
        ack = build_command_ack(
            robot_code=self.robot_code,
            command_id=command_id,
            accepted=False,
            reason_code=reason_code,
            reason_message=message,
            job_assignment_id=_str_or_none(raw.get("jobAssignmentId")),
            job_step_id=_str_or_none(raw.get("jobStepId")),
        )
        entry.state = _REJECTED
        entry.ack = ack
        self._publisher.publish_ack(ack)
        self.log.warning(
            f"[CommandRejected] commandId={command_id} reasonCode={reason_code} message={message}"
        )

    def _replay_duplicate(self, command_id: str, entry: _LedgerEntry) -> None:
        self.log.info(f"[CommandDuplicate] commandId={command_id} state={entry.state}; not re-executing")
        if entry.state == _REJECTED and entry.ack is not None:
            # Gửi lại đúng quyết định reject cũ (cùng reasonCode).
            self._publisher.publish_ack(entry.ack)
            return
        if entry.ack is None:
            return
        ack = build_command_ack(
            robot_code=self.robot_code,
            command_id=command_id,
            accepted=True,
            reason_code=REASON_ALREADY_ACCEPTED,
            reason_message=f"command already {entry.state.lower()}",
            job_assignment_id=entry.ack.get("jobAssignmentId"),
            job_step_id=entry.ack.get("jobStepId"),
        )
        self._publisher.publish_ack(ack)
        if entry.state == _COMPLETED and entry.result is not None:
            # Backend bỏ qua result trùng (idempotent) — gửi lại giúp backend bị mất result trước đó.
            self._publish_result(entry.result)

    def _publish_result(self, payload: dict[str, Any]) -> None:
        self._publisher.publish_result(payload)
        self.log.info(
            f"[CommandResultPublished] commandId={payload['commandId']} outcome={payload['outcome']} "
            f"errorCode={payload.get('errorCode')}"
        )

    def _remember(self, command_id: str, entry: _LedgerEntry) -> None:
        self._ledger[command_id] = entry
        if len(self._ledger) <= self._ledger_capacity:
            return
        # Không bao giờ evict command đang chạy — nếu không duplicate sẽ tạo goal thứ hai.
        for key in list(self._ledger):
            if len(self._ledger) <= self._ledger_capacity:
                break
            if self._ledger[key].state in (_REJECTED, _COMPLETED):
                del self._ledger[key]


def _str_or_none(value: Any) -> Optional[str]:
    return value if isinstance(value, str) and value else None
