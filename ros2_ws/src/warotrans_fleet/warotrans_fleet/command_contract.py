"""MQTT command contract (ADR 0005 / warotrans-system docs/mqtt-robot-contracts.md).

Pure Python — không import ROS để unit test chạy không cần rclpy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any, Mapping, Optional

if TYPE_CHECKING:
    from warotrans_fleet.route_grant import RouteGrant

SCHEMA_VERSION = 1

# Command types — contract chỉ cho phép hai loại này.
NAVIGATE_TO_POSE = "NAVIGATE_TO_POSE"
CANCEL = "CANCEL"

# CommandResult.outcome — backend validator chỉ nhận đúng ba giá trị này.
OUTCOME_SUCCEEDED = "SUCCEEDED"
OUTCOME_FAILED = "FAILED"
OUTCOME_CANCELED = "CANCELED"

# ACK reasonCode. Hai mã đầu giữ tên theo contract hiện có (backend có thể đã dùng).
REASON_NOT_LOCALIZED = "NOT_LOCALIZED"  # prompt: LOCALIZATION_LOST
REASON_BUSY = "BUSY"  # prompt: ROBOT_BUSY
REASON_UNKNOWN_TARGET = "UNKNOWN_TARGET"
REASON_INVALID_PAYLOAD = "INVALID_PAYLOAD"
REASON_NAV2_UNAVAILABLE = "NAV2_UNAVAILABLE"
REASON_ALREADY_ACCEPTED = "ALREADY_ACCEPTED"
REASON_WRONG_ROBOT = "WRONG_ROBOT"
REASON_UNSUPPORTED_COMMAND = "UNSUPPORTED_COMMAND"
REASON_MAP_MISMATCH = "MAP_MISMATCH"
REASON_INTERNAL_ERROR = "INTERNAL_ERROR"
REASON_ROUTE_REQUIRED = "ROUTE_REQUIRED"
REASON_ROUTE_NOT_AUTHORIZED = "ROUTE_NOT_AUTHORIZED"
REASON_ROUTE_GRAPH_MISMATCH = "ROUTE_GRAPH_MISMATCH"

# CommandResult.errorCode.
ERROR_GOAL_REJECTED = "GOAL_REJECTED"
ERROR_NAVIGATION_FAILED = "NAVIGATION_FAILED"
ERROR_NAVIGATION_TIMEOUT = "NAVIGATION_TIMEOUT"  # outcome=FAILED (không có outcome TIMEOUT)
ERROR_NAVIGATION_CANCELED = "NAVIGATION_CANCELED"
ERROR_NAV2_UNAVAILABLE = "NAV2_UNAVAILABLE"
ERROR_NAV2_CANCEL_UNCONFIRMED = "NAV2_CANCEL_UNCONFIRMED"
ERROR_CANCEL_REJECTED = "CANCEL_REJECTED"
ERROR_CANCEL_INEFFECTIVE = "CANCEL_INEFFECTIVE"
ERROR_INTERNAL_ERROR = "INTERNAL_ERROR"
ERROR_CORRIDOR_VIOLATION = "CORRIDOR_VIOLATION"
ERROR_GRANT_LOSS = "GRANT_LOSS"


class CommandRejected(Exception):
    """Command không hợp lệ / không thể nhận — router chuyển thành ACK accepted=false."""

    def __init__(self, reason_code: str, message: str) -> None:
        super().__init__(f"{reason_code}: {message}")
        self.reason_code = reason_code
        self.message = message


@dataclass(frozen=True)
class CommandEnvelope:
    schema_version: int
    message_id: str
    command_id: str
    robot_code: str
    command_type: str
    issued_at: Optional[str]
    job_assignment_id: Optional[str]
    job_step_id: Optional[str]
    payload: Mapping[str, Any] = field(default_factory=dict)


@dataclass(frozen=True)
class NavigateTarget:
    frame_id: str
    x: float
    y: float
    yaw: float
    timeout_sec: Optional[float]
    map_version_code: Optional[str]
    route: Optional["RouteGrant"] = None


def extract_command_id(body: Any) -> Optional[str]:
    if not isinstance(body, Mapping):
        return None
    value = body.get("commandId")
    if isinstance(value, str) and value.strip():
        return value.strip()
    return None


def _optional_str(body: Mapping[str, Any], key: str) -> Optional[str]:
    value = body.get(key)
    if value is None:
        return None
    if not isinstance(value, str):
        raise CommandRejected(REASON_INVALID_PAYLOAD, f"'{key}' must be a string or null")
    return value or None


def parse_envelope(body: Any, *, expected_robot_code: str) -> CommandEnvelope:
    """Validate envelope chung cho mọi command type. Payload cụ thể do handler validate."""
    if not isinstance(body, Mapping):
        raise CommandRejected(REASON_INVALID_PAYLOAD, "command body must be a JSON object")

    command_id = extract_command_id(body)
    if command_id is None:
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'commandId' is required")

    schema_version = body.get("schemaVersion")
    if schema_version != SCHEMA_VERSION or isinstance(schema_version, bool):
        raise CommandRejected(
            REASON_INVALID_PAYLOAD,
            f"'schemaVersion' must be {SCHEMA_VERSION}, got {schema_version!r}",
        )

    robot_code = body.get("robotCode")
    if not isinstance(robot_code, str) or not robot_code:
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'robotCode' is required")
    if robot_code != expected_robot_code:
        raise CommandRejected(
            REASON_WRONG_ROBOT,
            f"command addressed to '{robot_code}' but this robot is '{expected_robot_code}'",
        )

    command_type = body.get("type")
    if not isinstance(command_type, str) or not command_type:
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'type' is required")

    payload = body.get("payload")
    if not isinstance(payload, Mapping):
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'payload' must be a JSON object")

    return CommandEnvelope(
        schema_version=SCHEMA_VERSION,
        message_id=str(body.get("messageId") or ""),
        command_id=command_id,
        robot_code=robot_code,
        command_type=command_type,
        issued_at=_optional_str(body, "issuedAt"),
        job_assignment_id=_optional_str(body, "jobAssignmentId"),
        job_step_id=_optional_str(body, "jobStepId"),
        payload=dict(payload),
    )


def _finite_number(payload: Mapping[str, Any], key: str) -> float:
    if key not in payload:
        raise CommandRejected(REASON_INVALID_PAYLOAD, f"'payload.{key}' is required")
    value = payload[key]
    # bool là subclass của int trong Python — không chấp nhận true/false làm tọa độ.
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CommandRejected(REASON_INVALID_PAYLOAD, f"'payload.{key}' must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise CommandRejected(REASON_INVALID_PAYLOAD, f"'payload.{key}' must be finite")
    return number


def parse_navigate_target(payload: Mapping[str, Any], *, map_frame: str) -> NavigateTarget:
    frame_id = payload.get("frameId", map_frame)
    if not isinstance(frame_id, str) or not frame_id:
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'payload.frameId' must be a non-empty string")
    if frame_id != map_frame:
        raise CommandRejected(
            REASON_INVALID_PAYLOAD,
            f"'payload.frameId' must be '{map_frame}' (robot localizes in that frame), got '{frame_id}'",
        )

    x = _finite_number(payload, "x")
    y = _finite_number(payload, "y")
    yaw = _finite_number(payload, "yaw")

    timeout_sec: Optional[float] = None
    if payload.get("timeoutSeconds") is not None:
        timeout_sec = _finite_number(payload, "timeoutSeconds")
        if timeout_sec <= 0:
            raise CommandRejected(REASON_INVALID_PAYLOAD, "'payload.timeoutSeconds' must be > 0")

    map_version_code = payload.get("mapVersionCode")
    if map_version_code is not None and not isinstance(map_version_code, str):
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'payload.mapVersionCode' must be a string")

    from warotrans_fleet.route_grant import parse_route_grant

    route = parse_route_grant(payload)

    return NavigateTarget(
        frame_id=frame_id,
        x=x,
        y=y,
        yaw=yaw,
        timeout_sec=timeout_sec,
        map_version_code=map_version_code or None,
        route=route,
    )


def parse_cancel_target(payload: Mapping[str, Any]) -> str:
    target = payload.get("targetCommandId")
    if not isinstance(target, str) or not target.strip():
        raise CommandRejected(REASON_INVALID_PAYLOAD, "'payload.targetCommandId' is required")
    return target.strip()
