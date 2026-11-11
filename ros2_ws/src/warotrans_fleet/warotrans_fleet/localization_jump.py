"""Detects a localization jump: the map pose moving faster than the robot can drive or turn.

2026-11-10 lab (MAP-ONE-V005): Nav2 reported the goal reached, then the next AMCL update moved the pose 0.16-0.28 m
and 26-52 deg. Such a jump put the pose back into cells the gateway had already released, and the traffic gate held
the robot for ~20 s each time. Logging each jump with its size is how the next run measures it.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Callable, Optional


@dataclass(frozen=True)
class Jump:
    dt: float
    dxy: float
    dyaw: float

    def text(self) -> str:
        return f"dxy={self.dxy:.3f}m dyaw={self.dyaw:.3f}rad dt={self.dt:.2f}s"


class LocalizationJumpDetector:
    """A step counts as a jump when it exceeds what the robot could do in the elapsed time plus a fixed margin.

    Samples further apart than ``max_gap_sec`` only reset the reference: a long gap says nothing about a jump.
    """

    def __init__(
        self,
        *,
        xy_margin_m: float = 0.08,
        yaw_margin_rad: float = 0.15,
        max_linear_mps: float = 0.15,
        max_angular_rps: float = 0.7,
        max_gap_sec: float = 0.5,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self._xy_margin = float(xy_margin_m)
        self._yaw_margin = float(yaw_margin_rad)
        self._max_linear = float(max_linear_mps)
        self._max_angular = float(max_angular_rps)
        self._max_gap = float(max_gap_sec)
        self._clock = clock
        self._last: Optional[tuple[float, float, float, float]] = None
        self.count = 0

    def observe(self, x: float, y: float, yaw: float) -> Optional[Jump]:
        now = self._clock()
        last = self._last
        self._last = (now, x, y, yaw)
        if last is None:
            return None
        dt = now - last[0]
        if dt <= 0.0 or dt > self._max_gap:
            return None
        dxy = math.hypot(x - last[1], y - last[2])
        dyaw = abs(math.atan2(math.sin(yaw - last[3]), math.cos(yaw - last[3])))
        if dxy > self._xy_margin + self._max_linear * dt or dyaw > self._yaw_margin + self._max_angular * dt:
            self.count += 1
            return Jump(dt=dt, dxy=dxy, dyaw=dyaw)
        return None

    def reset(self) -> None:
        self._last = None
