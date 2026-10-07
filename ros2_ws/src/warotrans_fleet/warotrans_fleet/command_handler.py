"""Accept/reject MQTT commands and drive Nav2 NavigateToPose."""

from __future__ import annotations

import logging
import math
import threading
from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol

logger = logging.getLogger(__name__)


class Nav2Bridge(Protocol):
    def send_navigate(self, *, frame_id: str, x: float, y: float, yaw: float, command_id: str) -> None: ...
    def cancel_active(self) -> None: ...


@dataclass
class CommandDecision:
    accepted: bool
    reason_code: Optional[str] = None


class CommandHandler:
    """Thread-safe command state for the robot gateway."""

    def __init__(
        self,
        *,
        localization_provider: Callable[[], str],
        nav2: Optional[Nav2Bridge],
        on_result: Callable[[str, str, Optional[str]], None],
    ) -> None:
        self._localization_provider = localization_provider
        self._nav2 = nav2
        self._on_result = on_result
        self._lock = threading.Lock()
        self._active_command_id: Optional[str] = None
        self._seen: dict[str, CommandDecision] = {}

    @property
    def active_command_id(self) -> Optional[str]:
        with self._lock:
            return self._active_command_id

    def handle_command(self, message: dict[str, Any]) -> CommandDecision:
        command_id = str(message.get("commandId") or "")
        cmd_type = str(message.get("type") or "")
        if not command_id:
            return CommandDecision(False, "INVALID_PAYLOAD")

        with self._lock:
            prior = self._seen.get(command_id)
            if prior is not None:
                return CommandDecision(prior.accepted, "ALREADY_ACCEPTED" if prior.accepted else prior.reason_code)

            if cmd_type == "NAVIGATE_TO_POSE":
                decision = self._decide_navigate(message)
            elif cmd_type == "CANCEL":
                decision = self._decide_cancel(message)
            else:
                decision = CommandDecision(False, "INVALID_PAYLOAD")

            self._seen[command_id] = decision
            if decision.accepted and cmd_type == "NAVIGATE_TO_POSE":
                self._active_command_id = command_id
            return decision

    def _decide_navigate(self, message: dict[str, Any]) -> CommandDecision:
        if self._active_command_id is not None:
            return CommandDecision(False, "BUSY")
        if self._localization_provider() != "LOCALIZED":
            return CommandDecision(False, "NOT_LOCALIZED")
        if self._nav2 is None:
            return CommandDecision(False, "NAV2_UNAVAILABLE")

        payload = message.get("payload") or {}
        try:
            frame_id = str(payload.get("frameId") or "map")
            x = float(payload["x"])
            y = float(payload["y"])
            yaw = float(payload["yaw"])
        except (KeyError, TypeError, ValueError):
            return CommandDecision(False, "INVALID_PAYLOAD")

        command_id = str(message["commandId"])
        try:
            self._nav2.send_navigate(frame_id=frame_id, x=x, y=y, yaw=yaw, command_id=command_id)
        except Exception as exc:  # noqa: BLE001
            logger.warning("Nav2 send failed: %s", exc)
            return CommandDecision(False, "NAV2_UNAVAILABLE")
        return CommandDecision(True, None)

    def _decide_cancel(self, message: dict[str, Any]) -> CommandDecision:
        payload = message.get("payload") or {}
        target = str(payload.get("targetCommandId") or "")
        if not target:
            return CommandDecision(False, "INVALID_PAYLOAD")
        if self._active_command_id != target:
            return CommandDecision(False, "UNKNOWN_TARGET")
        if self._nav2 is None:
            return CommandDecision(False, "NAV2_UNAVAILABLE")
        try:
            self._nav2.cancel_active()
        except Exception as exc:  # noqa: BLE001
            logger.warning("Nav2 cancel failed: %s", exc)
            return CommandDecision(False, "NAV2_UNAVAILABLE")
        return CommandDecision(True, None)

    def notify_nav_result(self, command_id: str, outcome: str, error_code: Optional[str] = None) -> None:
        with self._lock:
            if self._active_command_id == command_id:
                self._active_command_id = None
        self._on_result(command_id, outcome, error_code)


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))
