"""Manual drive from the backend (MQTT `teleop` topic) → /cmd_vel, with a deadman stop.

Not a command: no ACK / RESULT, QoS 0. The web console sends `{linear, angular}` every ~200 ms while a
direction button is held and `{0, 0}` on release. Safety lives here, on the robot:

- speeds are clamped to `max_linear` / `max_angular` (real robot: provisional motor limits),
- no message for `deadman_sec` → one zero Twist (network loss, closed browser tab, crashed backend),
- ignored while a navigate command or any Nav2 goal is active (two writers on /cmd_vel would fight),
- Drive Assist (`assist`, default on): forward/backward speed scaled down then held at 0 near an obstacle in
  the direction of travel (lidar corridor, see drive_assist.py); re-checked every tick between messages,
- messages for another robot or with non-finite numbers are dropped; so is a `sentAt` older than
  `max_message_age_sec` when that check is enabled (> 0; needs robot and backend clocks in sync).

Pure Python (no rclpy) so it is unit-tested with a fake clock and a fake publisher.
"""

from __future__ import annotations

import math
from datetime import datetime, timezone
from typing import Any, Callable, Mapping, Optional

from warotrans_fleet.drive_assist import Gaps, assisted_linear

TwistPublisher = Callable[[float, float], None]


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _parse_iso(value: Any) -> Optional[datetime]:
    if not isinstance(value, str) or not value:
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        return None
    return parsed if parsed.tzinfo else parsed.replace(tzinfo=timezone.utc)


class TeleopController:
    def __init__(
        self,
        *,
        robot_code: str,
        publish_twist: TwistPublisher,
        monotonic: Callable[[], float],
        navigation_busy: Callable[[], bool],
        max_linear: float,
        max_angular: float,
        deadman_sec: float,
        max_message_age_sec: float = 0.0,
        wall_clock: Callable[[], datetime] = lambda: datetime.now(timezone.utc),
        log: Any = None,
        gaps_provider: Optional[Callable[[], Optional[Gaps]]] = None,
        assist_stop_gap: float = 0.15,
        assist_slow_gap: float = 0.6,
    ) -> None:
        if max_linear <= 0 or max_angular <= 0 or deadman_sec <= 0:
            raise ValueError("teleop limits and deadman must be > 0")
        self._robot_code = robot_code
        self._publish = publish_twist
        self._now = monotonic
        self._busy = navigation_busy
        self._max_linear = max_linear
        self._max_angular = max_angular
        self._deadman = deadman_sec
        self._max_age = max_message_age_sec
        self._wall = wall_clock
        self._log = log
        self._gaps = gaps_provider
        self._stop_gap = assist_stop_gap
        self._slow_gap = assist_slow_gap
        self._moving = False
        self._last_at = 0.0
        self._last_reject: Optional[str] = None
        # Requested setpoint (before assist) and what was last sent, so tick() can re-apply assist between messages.
        self._requested = (0.0, 0.0)
        self._assist = False
        self._sent = (0.0, 0.0)

    @property
    def is_moving(self) -> bool:
        return self._moving

    def handle(self, body: Mapping[str, Any]) -> Optional[str]:
        """Apply one teleop message. Returns None when applied, else the reason it was dropped."""
        reason = self._validate(body)
        if reason is not None:
            self._note_reject(reason)
            return reason

        linear = _clamp(float(body["linear"]), self._max_linear)
        angular = _clamp(float(body["angular"]), self._max_angular)
        # Drive Assist is on unless the operator explicitly turns it off (safe default for older senders).
        self._assist = body.get("assist", True) is not False and self._gaps is not None
        self._requested = (linear, angular)
        self._last_reject = None
        if linear == 0.0 and angular == 0.0:
            self._moving = False
            self._send(0.0, 0.0)
        else:
            self._moving = True
            self._last_at = self._now()
            self._send(*self._apply_assist(linear, angular))
        return None

    def tick(self) -> None:
        """Deadman stop, and Drive Assist re-check between messages (obstacles get closer while driving)."""
        if not self._moving:
            return
        if self._now() - self._last_at > self._deadman:
            self._moving = False
            self._send(0.0, 0.0)
            if self._log is not None:
                self._log.warning(f"[TeleopDeadman] no teleop message for > {self._deadman:.2f}s, stopped")
            return
        if self._assist:
            assisted = self._apply_assist(*self._requested)
            if assisted != self._sent:
                self._send(*assisted)

    def stop(self) -> None:
        if self._moving:
            self._moving = False
            self._send(0.0, 0.0)

    def _apply_assist(self, linear: float, angular: float) -> tuple[float, float]:
        if not self._assist or self._gaps is None:
            return linear, angular
        assisted = assisted_linear(linear, self._gaps(), self._stop_gap, self._slow_gap)
        if assisted == 0.0 and linear != 0.0 and self._log is not None and self._sent[0] != 0.0:
            self._log.info("[DriveAssist] obstacle ahead in the direction of travel, linear speed held at 0")
        return assisted, angular

    def _send(self, linear: float, angular: float) -> None:
        self._sent = (linear, angular)
        self._publish(linear, angular)

    def _validate(self, body: Mapping[str, Any]) -> Optional[str]:
        code = body.get("robotCode")
        if code is not None and code != self._robot_code:
            return "WRONG_ROBOT"
        try:
            linear = float(body.get("linear"))  # type: ignore[arg-type]
            angular = float(body.get("angular"))  # type: ignore[arg-type]
        except (TypeError, ValueError):
            return "INVALID_PAYLOAD"
        if not math.isfinite(linear) or not math.isfinite(angular):
            return "INVALID_PAYLOAD"
        sent_at = _parse_iso(body.get("sentAt"))
        if self._max_age > 0 and sent_at is not None and (self._wall() - sent_at).total_seconds() > self._max_age:
            return "STALE"
        if (linear != 0.0 or angular != 0.0) and self._busy():
            return "NAVIGATION_ACTIVE"
        return None

    def _note_reject(self, reason: str) -> None:
        # One log line per reason change, not one per 200 ms message.
        if self._log is not None and reason != self._last_reject:
            self._log.warning(f"[TeleopRejected] reason={reason}")
        self._last_reject = reason
