"""TrafficClient — state machine phía robot cho Active Mode (segment motion authority).

Pure Python, không import ROS: mọi phụ thuộc được inject (AuthorityStore, RouteGraphPort,
SegmentExecutorPort, progress_publisher, pose_provider).

State machine::

    IDLE ──start_navigate──▶ VALIDATING ──ok──▶ EXECUTING ──segment xong──▶ STOPPED_AWAITING
      ▲                          │ reject                │  ▲                     │ extend
      │                          ▼                       │  └─────────────────────┘
      │                        (state cũ)        revoke/cancel
      │                                                  ▼
      └────────── cancel xong ◀──────────────────── STOPPING
    mọi state chạy ──fault()──▶ FAULTED   (KHÔNG tự resume)

Quy tắc an toàn:
- Dừng (SAFE_STOP_REACHED / STOPPED_AWAITING_AUTHORITY / FAULT) KHÔNG BAO GIỜ là "release" hay
  "hoàn thành": client không gửi CommandResult SUCCEEDED; backend quyết định.
- FAULTED: chỉ thoát bằng start_navigate với (epoch, seq) cao hơn *sau khi* gọi mark_synced()
  (gateway gọi sau khi đã publish authority_state). Extend/renew trong FAULTED bị từ chối.
- Robot không đi ngoài ``authority.segment_edge_ids``; mở rộng chỉ chạy tiếp sau khi tới safe stop
  của ticket hiện tại (executor luôn bị giới hạn trong edge đã cấp).
"""

from __future__ import annotations

import time
from dataclasses import dataclass, replace
from typing import Any, Callable, Mapping, Optional, Sequence

from warotrans_fleet.authority_store import (
    Authority,
    AuthorityStore,
    parse_edge_ids,
    parse_expires_at,
    parse_speed_limits,
)
from warotrans_fleet.command_contract import (
    ERROR_CONTROLLER_FAILED,
    ERROR_CORRIDOR_VIOLATION,
    OUTCOME_CANCELED,
    OUTCOME_FAILED,
    PROGRESS_EDGE_ENTERED,
    PROGRESS_EDGE_EXITED,
    PROGRESS_FAULT,
    PROGRESS_SAFE_STOP_REACHED,
    PROGRESS_STARTED,
    PROGRESS_STOPPED_AWAITING_AUTHORITY,
    REASON_AUTHORITY_LOST,
    REASON_AUTHORITY_MISMATCH,
    REASON_AUTHORITY_REQUIRED,
    REASON_BUSY,
    REASON_CONTROLLER_FAILED,
    REASON_INTERNAL_ERROR,
    REASON_INVALID_PAYLOAD,
    REASON_MAP_VERSION_MISMATCH,
    REASON_NOT_LOCALIZED,
    REASON_PATH_HASH_MISMATCH,
    REASON_ROUTE_GRAPH_MISMATCH,
    REASON_ROUTE_NOT_AUTHORIZED,
    REVOKE_MODE_STOP_AT_SAFE_STOP,
    REVOKE_MODE_STOP_NOW,
    CommandEnvelope,
)
from warotrans_fleet.route_enforcer import RouteGraphPort
from warotrans_fleet.route_grant import compute_path_hash, edges_to_close
from warotrans_fleet.segment_executor import (
    RESULT_SUCCEEDED,
    Pose2D,
    SegmentCallbacks,
    SegmentExecutorPort,
    SegmentResult,
)

STATE_IDLE = "IDLE"
STATE_VALIDATING = "VALIDATING"
STATE_EXECUTING = "EXECUTING"
STATE_STOPPING = "STOPPING"
STATE_STOPPED_AWAITING = "STOPPED_AWAITING"
STATE_FAULTED = "FAULTED"

_INTENT_REVOKE = "REVOKE"
_INTENT_CANCEL = "CANCEL"


@dataclass(frozen=True)
class TrafficDecision:
    accepted: bool
    reason_code: Optional[str] = None
    message: str = ""


def _ok(message: str = "") -> TrafficDecision:
    return TrafficDecision(True, None, message)


def _reject(reason: str, message: str) -> TrafficDecision:
    return TrafficDecision(False, reason, message)


class TrafficClient:
    def __init__(
        self,
        *,
        store: AuthorityStore,
        route_port: Optional[RouteGraphPort],
        executor: SegmentExecutorPort,
        progress_publisher: Callable[[dict[str, Any]], None],
        pose_provider: Callable[[], Optional[Pose2D]] = lambda: None,
        all_edge_ids: Sequence[int] = (),
        clock: Callable[[], float] = time.time,
        monotonic: Callable[[], float] = time.monotonic,
        log: Optional[Callable[[str], None]] = None,
        result_callback: Optional[Callable[[str, str, Optional[str]], None]] = None,
        segment_goal_resolver: Optional[Callable[[Authority], Optional[Pose2D]]] = None,
        localization_ok: Callable[[], bool] = lambda: True,
        frame_id: str = "map",
    ) -> None:
        self._store = store
        self._route_port = route_port
        self._executor = executor
        self._publish = progress_publisher
        self._pose_provider = pose_provider
        self._all_edge_ids = sorted({int(e) for e in all_edge_ids})
        self._clock = clock
        self._monotonic = monotonic
        self._log = log or (lambda _m: None)
        self._result_callback = result_callback
        self._segment_goal_resolver = segment_goal_resolver
        self._localization_ok = localization_ok
        self._frame_id = frame_id

        self._state = STATE_IDLE
        self._authority: Optional[Authority] = None
        self._command_id: Optional[str] = None
        self._generation = 0
        self._covered = 0  # số edge đầu của authority đã giao cho executor
        self._intent: Optional[str] = None
        self._stop_at_safe_stop = False
        self._revoked = False
        self._synced = True
        self._progress_seq = 0
        self._current_edge: Optional[int] = None
        self._last_authority_at: Optional[float] = None
        self._last_fault: Optional[str] = None

    # ------------------------------------------------------------------ read-only view

    @property
    def state(self) -> str:
        return self._state

    @property
    def authority(self) -> Optional[Authority]:
        return self._authority

    @property
    def command_id(self) -> Optional[str]:
        return self._command_id

    @property
    def last_fault(self) -> Optional[str]:
        return self._last_fault

    @property
    def last_authority_at(self) -> Optional[float]:
        """Monotonic time của lần cuối nhận start/extend/renew hợp lệ (cho watchdog)."""
        return self._last_authority_at

    @property
    def current_edge(self) -> Optional[int]:
        return self._current_edge

    @property
    def motion_expected(self) -> bool:
        return self._state == STATE_EXECUTING

    @property
    def synced(self) -> bool:
        return self._synced

    def granted_edge_ids(self) -> tuple[int, ...]:
        return self._authority.segment_edge_ids if self._authority is not None else ()

    def next_edge_id(self) -> Optional[int]:
        """Edge kế tiếp robot sẽ đi sau edge hiện tại (None nếu chưa biết/đã hết)."""
        auth = self._authority
        if auth is None:
            return None
        edges = auth.segment_edge_ids
        if self._current_edge is None:
            return edges[0] if edges else None
        try:
            idx = edges.index(self._current_edge)
        except ValueError:
            return None
        return edges[idx + 1] if idx + 1 < len(edges) else None

    def authority_valid(self, now: Optional[float] = None) -> bool:
        """True nếu robot ĐƯỢC PHÉP có motion lúc này (không phải 'đã dừng' hay 'đã release')."""
        if self._state != STATE_EXECUTING or self._authority is None or self._revoked:
            return False
        wall = self._clock() if now is None else now
        return not self._authority.is_expired(wall)

    def mark_synced(self) -> None:
        """Gateway gọi sau khi đã publish authority_state cho backend (cho phép thoát FAULTED bằng seq cao hơn)."""
        self._synced = True

    def authority_state_report(self) -> dict[str, Any]:
        return self._store.report(state=self._state)

    # ------------------------------------------------------------------ start

    def start_navigate(
        self,
        envelope: CommandEnvelope,
        authority: Authority,
        expected_map_version_code: Optional[str],
        expected_graph_hash: Optional[str],
    ) -> TrafficDecision:
        previous = self._state
        if previous not in (STATE_IDLE, STATE_FAULTED):
            return _reject(REASON_BUSY, f"traffic client busy (state={previous})")
        if previous == STATE_FAULTED and not self._synced:
            return _reject(REASON_AUTHORITY_REQUIRED, "FAULTED: authority_state sync required before a new authority")

        self._state = STATE_VALIDATING
        try:
            decision = self._validate_and_start(envelope, authority, expected_map_version_code, expected_graph_hash)
        except Exception as exc:  # noqa: BLE001 - validation không được làm treo state machine
            self._log(f"[TrafficClient] start_navigate internal error: {exc!r}")
            decision = _reject(REASON_INTERNAL_ERROR, str(exc))
        if not decision.accepted:
            self._state = previous
        return decision

    def _validate_and_start(
        self,
        envelope: CommandEnvelope,
        authority: Authority,
        expected_map_version_code: Optional[str],
        expected_graph_hash: Optional[str],
    ) -> TrafficDecision:
        if authority.command_id is not None and authority.command_id != envelope.command_id:
            return _reject(REASON_AUTHORITY_MISMATCH, "authority.commandId != envelope.commandId")
        authority = replace(authority, command_id=envelope.command_id)

        if expected_map_version_code and authority.map_version_code != expected_map_version_code:
            return _reject(
                REASON_MAP_VERSION_MISMATCH,
                f"authority map {authority.map_version_code!r} != robot map {expected_map_version_code!r}",
            )
        if expected_graph_hash and authority.graph_hash.lower() != expected_graph_hash.strip().lower():
            return _reject(REASON_ROUTE_GRAPH_MISMATCH, "authority.graphHash != loaded graph hash")
        if authority.expected_path_hash() != authority.path_hash:
            return _reject(REASON_PATH_HASH_MISMATCH, "authority.pathHash != sha256(graphHash|edgeIds)")
        if not authority.segment_edge_ids:
            return _reject(REASON_ROUTE_NOT_AUTHORIZED, "authority.segmentEdgeIds is empty")

        pose = self._pose_provider()
        if pose is None:
            return _reject(REASON_NOT_LOCALIZED, "pose unavailable; cannot validate segment")

        accepted = self._store.accept(
            authority, command_id=envelope.command_id, message_id=envelope.message_id or None, now=self._clock()
        )
        if not accepted.accepted:
            return _reject(accepted.reason_code or REASON_AUTHORITY_REQUIRED, "authority rejected by store")

        failure = self._open_corridor(authority.segment_edge_ids)
        if failure is None and self._segment_goal_resolver is not None:
            goal = self._segment_goal_resolver(authority)
            if goal is not None:
                computed = self._executor.compute_route_edge_ids(pose, goal)
                if tuple(computed) != authority.segment_edge_ids:
                    failure = _reject(
                        REASON_ROUTE_NOT_AUTHORIZED,
                        f"ComputeRoute edges {list(computed)} != segment {list(authority.segment_edge_ids)}",
                    )
        if failure is not None:
            self._store.clear_active()
            return failure

        self._authority = authority
        self._command_id = envelope.command_id
        self._intent = None
        self._stop_at_safe_stop = False
        self._revoked = False
        self._covered = 0
        self._current_edge = None
        self._last_fault = None
        self._last_authority_at = self._monotonic()

        try:
            self._begin_ticket(authority.segment_edge_ids)
        except Exception as exc:  # noqa: BLE001
            self._log(f"[TrafficClient] follow_path failed: {exc!r}")
            self._authority = None
            self._command_id = None
            self._store.clear_active()
            return _reject(REASON_CONTROLLER_FAILED, f"follow_path failed: {exc}")

        self._state = STATE_EXECUTING
        self._emit(PROGRESS_STARTED)
        return _ok("segment started")

    def _open_corridor(self, granted: Sequence[int]) -> Optional[TrafficDecision]:
        if self._route_port is None:
            return _reject(REASON_ROUTE_NOT_AUTHORIZED, "route graph port unavailable")
        close_ids = edges_to_close(self._all_edge_ids, granted)
        if not self._route_port.adjust_edges(close_ids=close_ids, open_ids=list(granted)):
            return _reject(REASON_ROUTE_NOT_AUTHORIZED, "DynamicEdges adjust_edges failed")
        return None

    def _begin_ticket(self, edge_ids: Sequence[int]) -> None:
        self._generation += 1
        gen = self._generation
        callbacks = SegmentCallbacks(
            on_edge_entered=lambda e: self._on_edge_entered(gen, e),
            on_edge_exited=lambda e: self._on_edge_exited(gen, e),
            on_result=lambda r: self._on_result(gen, r),
        )
        self._executor.follow_path(tuple(edge_ids), callbacks)
        self._covered += len(edge_ids)

    # ------------------------------------------------------------------ extend / renew / revoke

    def extend_authority(
        self,
        envelope: CommandEnvelope,
        *,
        expected_graph_hash: Optional[str] = None,
    ) -> TrafficDecision:
        """EXTEND_AUTHORITY: segmentEdgeIds = TOÀN BỘ segment mới, phải bắt đầu bằng segment hiện tại."""
        check = self._check_target(envelope.payload, states=(STATE_EXECUTING, STATE_STOPPED_AWAITING))
        if isinstance(check, TrafficDecision):
            return check
        current = check
        payload = envelope.payload
        if self._revoked or self._stop_at_safe_stop:
            return _reject(REASON_AUTHORITY_LOST, "authority revoked; extension refused")

        try:
            edge_ids = parse_edge_ids(payload.get("segmentEdgeIds"), label="payload.segmentEdgeIds")
            graph_hash = payload.get("graphHash")
            path_hash = payload.get("pathHash")
            if not isinstance(graph_hash, str) or not graph_hash or not isinstance(path_hash, str) or not path_hash:
                return _reject(REASON_INVALID_PAYLOAD, "'graphHash' and 'pathHash' are required")
            epoch, seq = self._epoch_seq(payload)
            expires_at = parse_expires_at(payload.get("expiresAt")) if payload.get("expiresAt") is not None else None
            limits = parse_speed_limits(payload.get("speedLimits")) if payload.get("speedLimits") is not None else None
        except Exception as exc:  # noqa: BLE001 - CommandRejected / ValueError → INVALID_PAYLOAD
            reason = getattr(exc, "reason_code", REASON_INVALID_PAYLOAD)
            return _reject(reason, str(getattr(exc, "message", exc)))

        if graph_hash != current.graph_hash or (
            expected_graph_hash and graph_hash.lower() != expected_graph_hash.strip().lower()
        ):
            return _reject(REASON_ROUTE_GRAPH_MISMATCH, "extend graphHash != active graph hash")
        if compute_path_hash(graph_hash, edge_ids) != path_hash.lower():
            return _reject(REASON_PATH_HASH_MISMATCH, "extend pathHash != sha256(graphHash|edgeIds)")
        if len(edge_ids) <= len(current.segment_edge_ids) or edge_ids[: len(current.segment_edge_ids)] != current.segment_edge_ids:
            return _reject(REASON_ROUTE_NOT_AUTHORIZED, "extended segment must start with the current segment and be longer")

        new_auth = replace(
            current,
            epoch=epoch,
            seq=seq,
            segment_edge_ids=edge_ids,
            path_hash=path_hash.lower(),
            expires_at=expires_at if expires_at is not None else current.expires_at,
            speed_limits=limits if limits is not None else current.speed_limits,
        )
        accepted = self._store.accept(
            new_auth, command_id=envelope.command_id, message_id=envelope.message_id or None, now=self._clock()
        )
        if not accepted.accepted:
            return _reject(accepted.reason_code or REASON_AUTHORITY_REQUIRED, "extend rejected by store")

        failure = self._open_corridor(edge_ids)
        if failure is not None:
            # Giữ seq mới (monotonic) nhưng nội dung cũ: không mở rộng quyền.
            self._store.replace_active(replace(current, epoch=epoch, seq=seq))
            self._authority = replace(current, epoch=epoch, seq=seq)
            return failure

        self._authority = new_auth
        self._last_authority_at = self._monotonic()
        if self._state == STATE_STOPPED_AWAITING:
            try:
                self._begin_ticket(new_auth.segment_edge_ids[self._covered :])
            except Exception as exc:  # noqa: BLE001
                self.fault(ERROR_CONTROLLER_FAILED, f"follow_path failed: {exc}")
                return _reject(REASON_CONTROLLER_FAILED, f"follow_path failed: {exc}")
            self._state = STATE_EXECUTING
        return _ok("authority extended")

    def renew_authority(self, envelope: CommandEnvelope) -> TrafficDecision:
        check = self._check_target(envelope.payload, states=(STATE_EXECUTING, STATE_STOPPED_AWAITING))
        if isinstance(check, TrafficDecision):
            return check
        current = check
        if self._revoked:
            return _reject(REASON_AUTHORITY_LOST, "authority revoked; renew refused")
        try:
            epoch, seq = self._epoch_seq(envelope.payload)
            if envelope.payload.get("expiresAt") is None:
                return _reject(REASON_INVALID_PAYLOAD, "'expiresAt' is required")
            expires_at = parse_expires_at(envelope.payload.get("expiresAt"))
        except Exception as exc:  # noqa: BLE001
            reason = getattr(exc, "reason_code", REASON_INVALID_PAYLOAD)
            return _reject(reason, str(getattr(exc, "message", exc)))

        renewed = replace(current, epoch=epoch, seq=seq, expires_at=expires_at)
        accepted = self._store.accept(
            renewed, command_id=envelope.command_id, message_id=envelope.message_id or None, now=self._clock()
        )
        if not accepted.accepted:
            return _reject(accepted.reason_code or REASON_AUTHORITY_REQUIRED, "renew rejected by store")
        self._authority = renewed
        self._last_authority_at = self._monotonic()
        return _ok("authority renewed")

    def revoke_authority(self, envelope: CommandEnvelope) -> TrafficDecision:
        check = self._check_target(
            envelope.payload, states=(STATE_EXECUTING, STATE_STOPPING, STATE_STOPPED_AWAITING)
        )
        if isinstance(check, TrafficDecision):
            return check
        current = check
        mode = envelope.payload.get("mode", REVOKE_MODE_STOP_NOW)
        if mode not in (REVOKE_MODE_STOP_NOW, REVOKE_MODE_STOP_AT_SAFE_STOP):
            return _reject(REASON_INVALID_PAYLOAD, f"'mode' must be {REVOKE_MODE_STOP_NOW} or {REVOKE_MODE_STOP_AT_SAFE_STOP}")
        try:
            epoch, seq = self._epoch_seq(envelope.payload)
        except Exception as exc:  # noqa: BLE001
            reason = getattr(exc, "reason_code", REASON_INVALID_PAYLOAD)
            return _reject(reason, str(getattr(exc, "message", exc)))

        revoked = replace(current, epoch=epoch, seq=seq)
        accepted = self._store.accept(
            revoked, command_id=envelope.command_id, message_id=envelope.message_id or None, now=self._clock()
        )
        if not accepted.accepted:
            return _reject(accepted.reason_code or REASON_AUTHORITY_REQUIRED, "revoke rejected by store")
        self._authority = revoked
        self._last_authority_at = self._monotonic()

        if mode == REVOKE_MODE_STOP_AT_SAFE_STOP:
            self._stop_at_safe_stop = True
            return _ok("will stop at next safe stop")

        self._revoked = True
        if self._state == STATE_EXECUTING:
            self._begin_stop(_INTENT_REVOKE)
        return _ok("revoked: stopping")

    # ------------------------------------------------------------------ cancel / fault

    def cancel_command(self, command_id: str) -> TrafficDecision:
        """CANCEL command trong active mode: dừng, giải phóng authority local, báo CANCELED."""
        if self._command_id is None or command_id != self._command_id:
            return _reject(REASON_AUTHORITY_MISMATCH, "no active traffic command with that id")
        if self._state == STATE_EXECUTING:
            self._begin_stop(_INTENT_CANCEL)
        elif self._state == STATE_STOPPING:
            self._intent = _INTENT_CANCEL
        elif self._state == STATE_STOPPED_AWAITING:
            self._finish_idle(OUTCOME_CANCELED, None)
        elif self._state == STATE_FAULTED:
            self._reset_to_idle()  # FAULT đã báo; không phát result thứ hai
        else:
            return _reject(REASON_BUSY, f"cannot cancel in state {self._state}")
        return _ok("cancel accepted")

    def fault(self, code: str, detail: Optional[str] = None) -> None:
        """Fault khẩn: dừng ngay, FAULTED, KHÔNG tự resume. Idempotent."""
        if self._state in (STATE_IDLE, STATE_FAULTED):
            return
        self._generation += 1  # bỏ qua mọi callback cũ của executor
        try:
            self._executor.cancel()
        except Exception as exc:  # noqa: BLE001
            self._log(f"[TrafficClient] executor.cancel failed during fault: {exc!r}")
        self._state = STATE_FAULTED
        self._intent = None
        self._synced = False
        self._last_fault = code
        self._log(f"[TrafficClient] FAULT {code} {detail or ''}".rstrip())
        self._emit(PROGRESS_FAULT, fault_code=code)
        if self._command_id is not None and self._result_callback is not None:
            self._result_callback(self._command_id, OUTCOME_FAILED, code)

    # ------------------------------------------------------------------ internals

    def _check_target(self, payload: Mapping[str, Any], *, states: tuple[str, ...]):
        """Trả về Authority hiện tại nếu hợp lệ, hoặc TrafficDecision reject."""
        if self._state == STATE_FAULTED:
            return _reject(REASON_AUTHORITY_LOST, "FAULTED: only a new higher-seq authority after sync may run")
        if self._state not in states or self._authority is None:
            return _reject(REASON_AUTHORITY_REQUIRED, f"no active authority (state={self._state})")
        target = payload.get("targetCommandId")
        if not isinstance(target, str) or target != self._command_id:
            return _reject(REASON_AUTHORITY_MISMATCH, "targetCommandId does not match active command")
        authority_id = payload.get("authorityId")
        if not isinstance(authority_id, str) or authority_id != self._authority.authority_id:
            return _reject(REASON_AUTHORITY_MISMATCH, "authorityId does not match active authority")
        return self._authority

    @staticmethod
    def _epoch_seq(payload: Mapping[str, Any]) -> tuple[int, int]:
        values = []
        for key in ("epoch", "seq"):
            value = payload.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or int(value) != value:
                raise ValueError(f"'{key}' must be an integer")
            values.append(int(value))
        return values[0], values[1]

    def _begin_stop(self, intent: str) -> None:
        self._state = STATE_STOPPING
        self._intent = intent
        try:
            self._executor.cancel()
        except Exception as exc:  # noqa: BLE001
            self.fault(ERROR_CONTROLLER_FAILED, f"cancel failed: {exc}")

    def _on_edge_entered(self, gen: int, edge_id: int) -> None:
        if gen != self._generation or self._state not in (STATE_EXECUTING, STATE_STOPPING):
            return
        auth = self._authority
        if auth is None or edge_id not in auth.segment_edge_ids:
            self.fault(ERROR_CORRIDOR_VIOLATION, f"entered ungranted edge {edge_id}")
            return
        self._current_edge = edge_id
        self._emit(PROGRESS_EDGE_ENTERED, edge_id=edge_id)

    def _on_edge_exited(self, gen: int, edge_id: int) -> None:
        if gen != self._generation or self._state not in (STATE_EXECUTING, STATE_STOPPING):
            return
        self._emit(PROGRESS_EDGE_EXITED, edge_id=edge_id)

    def _on_result(self, gen: int, result: SegmentResult) -> None:
        if gen != self._generation:
            return
        if self._state == STATE_EXECUTING:
            self._on_result_executing(result)
        elif self._state == STATE_STOPPING:
            self._on_result_stopping(result)

    def _on_result_executing(self, result: SegmentResult) -> None:
        if result.status != RESULT_SUCCEEDED:
            # ABORTED / TIMEOUT / CANCELED không do mình yêu cầu → controller lỗi, không tự retry.
            self.fault(ERROR_CONTROLLER_FAILED, f"segment {result.status}: {result.detail or ''}".strip())
            return
        auth = self._authority
        if (
            auth is not None
            and not self._stop_at_safe_stop
            and not self._revoked
            and not auth.is_expired(self._clock())
            and len(auth.segment_edge_ids) > self._covered
        ):
            try:
                self._begin_ticket(auth.segment_edge_ids[self._covered :])
            except Exception as exc:  # noqa: BLE001
                self.fault(ERROR_CONTROLLER_FAILED, f"follow_path failed: {exc}")
            return
        self._enter_safe_stop()

    def _on_result_stopping(self, result: SegmentResult) -> None:
        if result.status == RESULT_SUCCEEDED:
            self._emit(PROGRESS_SAFE_STOP_REACHED)
        intent = self._intent
        self._intent = None
        if intent == _INTENT_CANCEL:
            self._finish_idle(OUTCOME_CANCELED, None)
        else:  # REVOKE
            self._state = STATE_STOPPED_AWAITING
            self._emit(PROGRESS_STOPPED_AWAITING_AUTHORITY)

    def _enter_safe_stop(self) -> None:
        self._emit(PROGRESS_SAFE_STOP_REACHED)
        self._state = STATE_STOPPED_AWAITING
        self._emit(PROGRESS_STOPPED_AWAITING_AUTHORITY)

    def _finish_idle(self, outcome: Optional[str], error_code: Optional[str]) -> None:
        command_id = self._command_id
        self._reset_to_idle()
        if outcome is not None and command_id is not None and self._result_callback is not None:
            self._result_callback(command_id, outcome, error_code)

    def _reset_to_idle(self) -> None:
        self._generation += 1
        self._state = STATE_IDLE
        self._authority = None
        self._command_id = None
        self._intent = None
        self._covered = 0
        self._current_edge = None
        self._stop_at_safe_stop = False
        self._revoked = False
        self._store.clear_active()

    def _emit(self, event: str, *, edge_id: Optional[int] = None, fault_code: Optional[str] = None) -> None:
        self._progress_seq += 1
        auth = self._authority
        pose = self._pose_provider()
        payload: dict[str, Any] = {
            "event": event,
            "authorityId": auth.authority_id if auth else None,
            "epoch": auth.epoch if auth else self._store.last_epoch,
            "seq": auth.seq if auth else self._store.last_seq,
            "commandId": self._command_id,
            "edgeId": edge_id,
            "poseX": pose[0] if pose else None,
            "poseY": pose[1] if pose else None,
            "poseYaw": pose[2] if pose else None,
            "localizationOk": bool(self._localization_ok()),
            "faultCode": fault_code,
            "progressSeq": self._progress_seq,
            "sentAt": self._clock(),
        }
        try:
            self._publish(payload)
        except Exception as exc:  # noqa: BLE001 - uplink lỗi không được làm hỏng state machine
            self._log(f"[TrafficClient] progress publish failed: {exc!r}")
