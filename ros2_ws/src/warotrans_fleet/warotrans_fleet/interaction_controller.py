"""InteractionController: IDLE → ARMED → CONFIRM_PENDING → IDLE.

GPIO never completes a JobStep. Backend remains source of truth.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Optional
from uuid import uuid4

from warotrans_fleet.interaction_contract import (
    TERMINAL_REJECT_REASONS,
    InteractionRequest,
    InteractionResult,
    build_interaction_confirm,
)

_LOG = logging.getLogger(__name__)


class InteractionState(str, Enum):
    IDLE = "IDLE"
    ARMED = "ARMED"
    CONFIRM_PENDING = "CONFIRM_PENDING"


@dataclass
class ActiveInteraction:
    job_id: str
    job_step_id: str
    interaction_type: str
    container_id: str
    endpoint_id: str


PublishFn = Callable[[dict[str, Any]], None]
LogFn = Callable[[str], None]


class InteractionController:
    def __init__(
        self,
        *,
        robot_code: str,
        boot_id: str,
        next_sequence: Callable[[], int],
        publish_confirm: PublishFn,
        log_info: Optional[LogFn] = None,
        log_warn: Optional[LogFn] = None,
        resend_interval_sec: float = 5.0,
        allow_stale_replace: bool = True,
    ) -> None:
        self._robot_code = robot_code
        self._boot_id = boot_id
        self._next_sequence = next_sequence
        self._publish = publish_confirm
        self._info = log_info or (lambda m: _LOG.info(m))
        self._warn = log_warn or (lambda m: _LOG.warning(m))
        self._resend_interval = resend_interval_sec
        self._allow_stale_replace = allow_stale_replace
        self.state = InteractionState.IDLE
        self.active: Optional[ActiveInteraction] = None
        self._pending_payload: Optional[dict[str, Any]] = None
        self._pending_since_mono: Optional[float] = None
        self._mono = 0.0

    @property
    def interaction_armed(self) -> bool:
        return self.state == InteractionState.ARMED

    def set_monotonic(self, t: float) -> None:
        self._mono = t

    def handle_request(self, req: InteractionRequest) -> None:
        if self.active is not None and self.active.job_step_id == req.job_step_id:
            # Idempotent re-arm of the same step.
            self.active = ActiveInteraction(
                job_id=req.job_id,
                job_step_id=req.job_step_id,
                interaction_type=req.interaction_type,
                container_id=req.container_id,
                endpoint_id=req.endpoint_id,
            )
            if self.state == InteractionState.CONFIRM_PENDING:
                return
            self.state = InteractionState.ARMED
            self._info(
                f"INTERACTION_ARMED (idempotent) type={req.interaction_type} "
                f"jobStepId={req.job_step_id} containerId={req.container_id} endpointId={req.endpoint_id}"
            )
            return

        if self.active is not None and self.active.job_step_id != req.job_step_id:
            if self._allow_stale_replace and self.state in (
                InteractionState.ARMED,
                InteractionState.CONFIRM_PENDING,
            ):
                self._warn(
                    f"INTERACTION_REPLACED_STALE old={self.active.job_step_id} new={req.job_step_id}"
                )
                self._clear_pending()
            else:
                self._warn(
                    f"INTERACTION_REJECTED_BUSY active={self.active.job_step_id} new={req.job_step_id}"
                )
                return

        self.active = ActiveInteraction(
            job_id=req.job_id,
            job_step_id=req.job_step_id,
            interaction_type=req.interaction_type,
            container_id=req.container_id,
            endpoint_id=req.endpoint_id,
        )
        self.state = InteractionState.ARMED
        self._info(
            f"INTERACTION_ARMED type={req.interaction_type} "
            f"jobStepId={req.job_step_id} containerId={req.container_id} endpointId={req.endpoint_id}"
        )

    def on_button_press(self) -> None:
        if self.state != InteractionState.ARMED or self.active is None:
            self._info("BUTTON_IGNORED_NOT_ARMED")
            return

        self._info(
            f"BUTTON_PRESSED type={self.active.interaction_type} jobStepId={self.active.job_step_id}"
        )
        payload = build_interaction_confirm(
            robot_code=self._robot_code,
            boot_id=self._boot_id,
            sequence=self._next_sequence(),
            job_id=self.active.job_id,
            job_step_id=self.active.job_step_id,
            interaction_type=self.active.interaction_type,
            container_id=self.active.container_id,
            endpoint_id=self.active.endpoint_id,
            message_id=str(uuid4()),
        )
        self._pending_payload = payload
        self._pending_since_mono = self._mono
        self.state = InteractionState.CONFIRM_PENDING
        self._publish(payload)
        self._info(f"INTERACTION_CONFIRM_PUBLISHED messageId={payload['messageId']}")

    def tick(self, mono: float) -> None:
        self._mono = mono
        if self.state != InteractionState.CONFIRM_PENDING or self._pending_payload is None:
            return
        if self._pending_since_mono is None:
            self._pending_since_mono = mono
            return
        if (mono - self._pending_since_mono) >= self._resend_interval:
            self._publish(self._pending_payload)
            self._pending_since_mono = mono
            self._info(
                f"INTERACTION_CONFIRM_PUBLISHED (resend) messageId={self._pending_payload['messageId']}"
            )

    def handle_result(self, result: InteractionResult) -> None:
        if self.active is None:
            return
        if result.job_step_id != self.active.job_step_id:
            self._warn(
                f"INTERACTION_RESULT_IGNORED_STEP result={result.job_step_id} active={self.active.job_step_id}"
            )
            return
        if self._pending_payload is not None and result.confirm_message_id != self._pending_payload["messageId"]:
            self._warn(
                f"INTERACTION_RESULT_IGNORED_MSG result={result.confirm_message_id} "
                f"pending={self._pending_payload['messageId']}"
            )
            return

        if result.accepted:
            self._info(
                f"INTERACTION_ACCEPTED jobStepId={result.job_step_id} reason={result.reason_code}"
            )
            self._clear_all()
            return

        reason = result.reason_code or "unknown"
        self._warn(f"INTERACTION_REJECTED jobStepId={result.job_step_id} reason={reason}")
        if reason in TERMINAL_REJECT_REASONS or reason == "ALREADY_CONFIRMED":
            self._clear_all()
        else:
            # Transient: return to ARMED so the operator can press again.
            self._clear_pending()
            self.state = InteractionState.ARMED

    def _clear_pending(self) -> None:
        self._pending_payload = None
        self._pending_since_mono = None

    def _clear_all(self) -> None:
        self._clear_pending()
        self.active = None
        self.state = InteractionState.IDLE
