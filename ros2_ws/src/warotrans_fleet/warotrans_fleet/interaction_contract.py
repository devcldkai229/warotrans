"""Pure-Python MQTT contract for HUMAN_INTERACTION arming / confirm / result.

Not a RobotCommand. V1 physical button supports PICKUP_CONFIRM and DROPOFF_CONFIRM only.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Any, Optional
from uuid import uuid4

SCHEMA_VERSION = 1
SUPPORTED_INTERACTION_TYPES = frozenset({"PICKUP_CONFIRM", "DROPOFF_CONFIRM"})
TERMINAL_REJECT_REASONS = frozenset(
    {
        "interaction_not_current",
        "interaction_wrong_robot",
        "interaction_wrong_job",
        "interaction_step_not_found",
        "interaction_type_unsupported",
        "interaction_type_mismatch",
        "handover_container_mismatch",
        "interaction_endpoint_mismatch",
        "job_not_running",
    }
)


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


@dataclass(frozen=True)
class InteractionRequest:
    message_id: str
    robot_code: str
    job_id: str
    job_step_id: str
    interaction_type: str
    container_id: str
    endpoint_id: str
    sent_at: str


@dataclass(frozen=True)
class InteractionResult:
    message_id: str
    robot_code: str
    job_step_id: str
    confirm_message_id: str
    accepted: bool
    reason_code: Optional[str]
    processed_at: str


class InteractionContractError(ValueError):
    pass


def parse_interaction_request(body: dict[str, Any], *, local_robot_code: str) -> InteractionRequest:
    if not isinstance(body, dict):
        raise InteractionContractError("body must be an object")
    if body.get("schemaVersion") != SCHEMA_VERSION:
        raise InteractionContractError(f"schemaVersion must be {SCHEMA_VERSION}")
    robot_code = str(body.get("robotCode") or "")
    if robot_code != local_robot_code:
        raise InteractionContractError(f"robotCode mismatch: {robot_code!r} != {local_robot_code!r}")
    interaction_type = str(body.get("interactionType") or "")
    if interaction_type not in SUPPORTED_INTERACTION_TYPES:
        raise InteractionContractError(f"unsupported interactionType: {interaction_type!r}")
    job_step_id = str(body.get("jobStepId") or "")
    job_id = str(body.get("jobId") or "")
    container_id = str(body.get("containerId") or "")
    endpoint_id = str(body.get("endpointId") or "")
    message_id = str(body.get("messageId") or "")
    if not job_step_id or not job_id or not container_id or not endpoint_id or not message_id:
        raise InteractionContractError("jobId/jobStepId/containerId/endpointId/messageId required")
    return InteractionRequest(
        message_id=message_id,
        robot_code=robot_code,
        job_id=job_id,
        job_step_id=job_step_id,
        interaction_type=interaction_type,
        container_id=container_id,
        endpoint_id=endpoint_id,
        sent_at=str(body.get("sentAt") or utc_now_iso()),
    )


def parse_interaction_result(body: dict[str, Any], *, local_robot_code: str) -> InteractionResult:
    if not isinstance(body, dict):
        raise InteractionContractError("body must be an object")
    if body.get("schemaVersion") != SCHEMA_VERSION:
        raise InteractionContractError(f"schemaVersion must be {SCHEMA_VERSION}")
    robot_code = str(body.get("robotCode") or "")
    if robot_code and robot_code != local_robot_code:
        raise InteractionContractError(f"robotCode mismatch: {robot_code!r}")
    job_step_id = str(body.get("jobStepId") or "")
    confirm_message_id = str(body.get("confirmMessageId") or "")
    if not job_step_id or not confirm_message_id:
        raise InteractionContractError("jobStepId and confirmMessageId required")
    accepted = bool(body.get("accepted"))
    reason = body.get("reasonCode")
    return InteractionResult(
        message_id=str(body.get("messageId") or ""),
        robot_code=robot_code or local_robot_code,
        job_step_id=job_step_id,
        confirm_message_id=confirm_message_id,
        accepted=accepted,
        reason_code=str(reason) if reason is not None else None,
        processed_at=str(body.get("processedAt") or utc_now_iso()),
    )


def build_interaction_confirm(
    *,
    robot_code: str,
    boot_id: str,
    sequence: int,
    job_id: str,
    job_step_id: str,
    interaction_type: str,
    container_id: str,
    endpoint_id: str,
    message_id: Optional[str] = None,
) -> dict[str, Any]:
    return {
        "schemaVersion": SCHEMA_VERSION,
        "messageId": message_id or str(uuid4()),
        "robotCode": robot_code,
        "bootId": boot_id,
        "sequence": int(sequence),
        "sentAt": utc_now_iso(),
        "jobId": job_id,
        "jobStepId": job_step_id,
        "interactionType": interaction_type,
        "containerId": container_id,
        "endpointId": endpoint_id,
        "source": "ROBOT_BUTTON",
    }
