"""TF-age localization latch — require N consecutive stale samples before LOST.

Brief Gazebo/sim TF blips must not flip telemetry to LOST (Host would sticky-quarantine).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class LocalizationHysteresis:
    """Tracks consecutive stale TF lookups before publishing LOST."""

    stale_limit: int = 3
    _stale_count: int = 0
    _ever_localized: bool = False
    _last_pose: Optional[dict[str, float]] = None
    _last_status: str = "UNKNOWN"

    def on_good(self, pose: dict[str, float]) -> tuple[dict[str, float], str]:
        self._stale_count = 0
        self._ever_localized = True
        self._last_pose = pose
        self._last_status = "LOCALIZED"
        return pose, "LOCALIZED"

    def on_stale(self) -> tuple[Optional[dict[str, float]], str]:
        """Missing TF, age over limit, or negative age (clock jump back)."""
        self._stale_count += 1
        if self._stale_count < self.stale_limit and self._ever_localized and self._last_pose is not None:
            # Hold last good pose + LOCALIZED through short blips.
            return self._last_pose, "LOCALIZED"
        status = "LOST" if self._ever_localized else "UNKNOWN"
        self._last_status = status
        return None, status

    @property
    def ever_localized(self) -> bool:
        return self._ever_localized

    @property
    def last_status(self) -> str:
        return self._last_status
