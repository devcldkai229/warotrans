"""Physical HUMAN_CONFIRM button: edge detector + lgpio adapter (Pi 5 gpiochip 4).

Not an E-stop. Momentary NO button to GND with internal pull-up → idle HIGH, pressed LOW.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional, Protocol

_LOG = logging.getLogger(__name__)


class ButtonInput(Protocol):
    def read_level(self) -> Optional[int]:
        """Return 0/1, or None if unavailable."""


@dataclass
class ButtonEdgeDetector:
    """One logical press per physical press: debounce + require release before next event.

    Samples are (level, t_ms). Returns True once when a press is confirmed.
    """

    active_low: bool = True
    debounce_ms: int = 50
    min_interval_ms: int = 300

    def __post_init__(self) -> None:
        self._stable_level: Optional[int] = None
        self._candidate_level: Optional[int] = None
        self._candidate_since_ms: Optional[int] = None
        self._pressed = False
        self._last_event_ms: Optional[int] = None

    def _is_pressed(self, level: int) -> bool:
        return (level == 0) if self.active_low else (level == 1)

    def sample(self, level: int, t_ms: int) -> bool:
        if self._candidate_level is None or level != self._candidate_level:
            self._candidate_level = level
            self._candidate_since_ms = t_ms
            return False

        assert self._candidate_since_ms is not None
        if (t_ms - self._candidate_since_ms) < self.debounce_ms:
            return False

        # Level is stable.
        if self._stable_level == level:
            return False
        self._stable_level = level
        pressed = self._is_pressed(level)

        if pressed and not self._pressed:
            if self._last_event_ms is not None and (t_ms - self._last_event_ms) < self.min_interval_ms:
                self._pressed = True
                return False
            self._pressed = True
            self._last_event_ms = t_ms
            return True

        if not pressed:
            self._pressed = False
        return False


class LgpioButtonInput:
    """BCM pin on Pi 5 RP1 (gpiochip 4) with internal pull-up."""

    def __init__(self, *, gpiochip: int, pin: int, log: Optional[logging.Logger] = None) -> None:
        self._chip_handle: Optional[int] = None
        self._pin = pin
        self._log = log or _LOG
        try:
            import lgpio  # type: ignore

            self._lgpio = lgpio
            self._chip_handle = lgpio.gpiochip_open(gpiochip)
            lgpio.gpio_claim_input(self._chip_handle, pin, lgpio.SET_PULL_UP)
            # ROS RcutilsLogger only accepts a single message string (no %-format args).
            self._log.info(
                f"HUMAN_CONFIRM_BUTTON claimed gpiochip={gpiochip} BCM={pin} (pull-up, active-low)"
            )
        except Exception as exc:  # pragma: no cover - hardware / missing lib
            self._lgpio = None
            self._chip_handle = None
            self._log.warning(f"HUMAN_CONFIRM_BUTTON disabled: {exc}")

    @property
    def available(self) -> bool:
        return self._chip_handle is not None and self._lgpio is not None

    def read_level(self) -> Optional[int]:
        if not self.available:
            return None
        try:
            return int(self._lgpio.gpio_read(self._chip_handle, self._pin))
        except Exception as exc:  # pragma: no cover
            self._log.warning(f"HUMAN_CONFIRM_BUTTON read failed: {exc}")
            return None

    def close(self) -> None:
        if self._chip_handle is not None and self._lgpio is not None:
            try:
                self._lgpio.gpiochip_close(self._chip_handle)
            except Exception:
                pass
            self._chip_handle = None


class FakeButtonInput:
    """Test double: set .level then sample."""

    def __init__(self, level: int = 1) -> None:
        self.level = level

    def read_level(self) -> Optional[int]:
        return self.level
