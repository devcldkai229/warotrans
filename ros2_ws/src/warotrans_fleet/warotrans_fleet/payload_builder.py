"""Build MQTT JSON payloads matching warotrans-system shared contract."""

from __future__ import annotations

import uuid
from datetime import datetime, timezone
from typing import Any, Mapping, Optional


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def new_message_id() -> str:
    return str(uuid.uuid4())


def build_heartbeat(
    *,
    robot_code: str,
    boot_id: str,
    sequence: int,
    sent_at: Optional[str] = None,
    message_id: Optional[str] = None,
) -> dict[str, Any]:
    return {
        "schemaVersion": 1,
        "messageId": message_id or new_message_id(),
        "robotCode": robot_code,
        "bootId": boot_id,
        "sequence": sequence,
        "sentAt": sent_at or utc_now_iso(),
    }


def build_telemetry(
    *,
    robot_code: str,
    boot_id: str,
    sequence: int,
    map_version_code: Optional[str],
    pose: Optional[Mapping[str, float]],
    battery_percent: Optional[float],
    navigation_status: str,
    localization_status: str,
    linear_velocity: float,
    angular_velocity: float,
    current_command_id: Optional[str] = None,
    error_code: Optional[str] = None,
    sent_at: Optional[str] = None,
    message_id: Optional[str] = None,
) -> dict[str, Any]:
    pose_payload: Any
    if pose is None:
        pose_payload = None
    else:
        pose_payload = {
            "x": float(pose["x"]),
            "y": float(pose["y"]),
            "yaw": float(pose["yaw"]),
        }

    return {
        "schemaVersion": 1,
        "messageId": message_id or new_message_id(),
        "robotCode": robot_code,
        "bootId": boot_id,
        "sequence": sequence,
        "sentAt": sent_at or utc_now_iso(),
        "mapVersionCode": map_version_code,
        "pose": pose_payload,
        "batteryPercent": battery_percent,
        "navigationStatus": navigation_status,
        "localizationStatus": localization_status,
        "linearVelocity": float(linear_velocity),
        "angularVelocity": float(angular_velocity),
        "currentCommandId": current_command_id,
        "errorCode": error_code,
    }


def build_command_ack(
    *,
    robot_code: str,
    command_id: str,
    accepted: bool,
    reason_code: Optional[str] = None,
    reason_message: Optional[str] = None,
    job_assignment_id: Optional[str] = None,
    job_step_id: Optional[str] = None,
    sent_at: Optional[str] = None,
    message_id: Optional[str] = None,
) -> dict[str, Any]:
    # jobAssignmentId / jobStepId / reasonMessage: field mở rộng để correlate + debug.
    # Backend (System.Text.Json) bỏ qua field lạ nên không phá contract.
    return {
        "schemaVersion": 1,
        "messageId": message_id or new_message_id(),
        "commandId": command_id,
        "robotCode": robot_code,
        "accepted": bool(accepted),
        "reasonCode": reason_code,
        "reasonMessage": reason_message,
        "jobAssignmentId": job_assignment_id,
        "jobStepId": job_step_id,
        "sentAt": sent_at or utc_now_iso(),
    }


def build_command_result(
    *,
    robot_code: str,
    command_id: str,
    outcome: str,
    error_code: Optional[str] = None,
    error_message: Optional[str] = None,
    job_assignment_id: Optional[str] = None,
    job_step_id: Optional[str] = None,
    result: Optional[Mapping[str, Any]] = None,
    sent_at: Optional[str] = None,
    message_id: Optional[str] = None,
) -> dict[str, Any]:
    return {
        "schemaVersion": 1,
        "messageId": message_id or new_message_id(),
        "commandId": command_id,
        "robotCode": robot_code,
        "outcome": outcome,
        "errorCode": error_code,
        "errorMessage": error_message,
        "jobAssignmentId": job_assignment_id,
        "jobStepId": job_step_id,
        "result": dict(result) if result is not None else None,
        "sentAt": sent_at or utc_now_iso(),
    }
