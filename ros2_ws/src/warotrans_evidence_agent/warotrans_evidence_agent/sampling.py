"""Pure helpers of the agent: when to keep a pose, how to shrink a plan, what the clock is worth."""

from __future__ import annotations

import hashlib
import json
import math
import subprocess
from dataclasses import dataclass
from typing import Optional


@dataclass
class PoseSample:
    x: float
    y: float
    yaw: float
    v: Optional[float]
    w: Optional[float]
    valid: bool
    wall_ns: int


class TrajectorySampler:
    """5 Hz while moving, 1 Hz while still; always a sample when validity changes or an event asks for one.

    A sample is invalid when the map→base_link transform is stale or localization is not OK: it is kept (it marks the
    gap) but never joined to its neighbours. A jump larger than ``jump_m`` between two valid samples taken close in
    time is a relocalization, not motion: it is reported so the path is broken there.
    """

    def __init__(self, moving_hz: float = 5.0, still_hz: float = 1.0, still_speed: float = 0.02,
                 jump_m: float = 0.5, max_tf_age_s: float = 0.5) -> None:
        self.moving_period = 1.0 / moving_hz
        self.still_period = 1.0 / still_hz
        self.still_speed = still_speed
        self.jump_m = jump_m
        self.max_tf_age_s = max_tf_age_s
        self._last: Optional[PoseSample] = None

    def offer(self, x: float, y: float, yaw: float, v: Optional[float], w: Optional[float], tf_age_s: float,
              localized: bool, wall_ns: int, force: bool = False) -> tuple[Optional[PoseSample], Optional[str]]:
        """Returns (sample to keep or None, event name or None)."""
        finite = all(math.isfinite(a) for a in (x, y, yaw))
        valid = finite and localized and tf_age_s <= self.max_tf_age_s
        if not finite:
            x = y = yaw = 0.0
        last = self._last
        event = None
        if last is not None:
            dt = (wall_ns - last.wall_ns) / 1e9
            moving = abs(v or 0.0) > self.still_speed or abs(w or 0.0) > 0.05
            # 10 % slack: the node offers a pose every 0.2 s by timer, and a tick that lands at 0.199 s must not wait for
            # the next one (that halved the moving rate to ~2.5 Hz on RBT-001, lab 2026-11-21).
            due = dt >= 0.9 * (self.moving_period if moving else self.still_period)
            if not (force or due or valid != last.valid):
                return None, None
            if valid and last.valid and dt < 2.0 and math.hypot(x - last.x, y - last.y) > self.jump_m:
                event = "localization.jump"
            elif valid != last.valid:
                event = "localization.regained" if valid else "localization.lost"
        sample = PoseSample(x, y, yaw, v, w, valid, wall_ns)
        self._last = sample
        return sample, event


def decimate(points: list[tuple[float, float]], max_points: int = 500, min_step: float = 0.05) -> list[list[float]]:
    """Keeps the shape of a Nav2 plan under ``max_points``: drops points closer than ``min_step``, then strides."""
    kept: list[tuple[float, float]] = []
    for p in points:
        if not kept or math.hypot(p[0] - kept[-1][0], p[1] - kept[-1][1]) >= min_step:
            kept.append(p)
    if points and kept[-1] != points[-1]:
        kept.append(points[-1])
    if len(kept) > max_points:
        stride = math.ceil(len(kept) / (max_points - 1))
        kept = kept[::stride] + ([kept[-1]] if (len(kept) - 1) % stride else [])
    return [[round(a, 3), round(b, 3)] for a, b in kept[:max_points]]


def path_hash(points: list[list[float]]) -> str:
    return hashlib.sha256(json.dumps(points, separators=(",", ":")).encode()).hexdigest()[:32]


@dataclass
class ClockQuality:
    source: str
    synchronized: bool
    offset_ms: Optional[float]
    error_bound_ms: Optional[float]


def parse_chrony_tracking(csv_line: str) -> ClockQuality:
    """``chronyc -c tracking``: system offset (5th), root delay (11th), root dispersion (12th), leap status (14th)."""
    f = csv_line.strip().split(",")
    if len(f) < 14:
        return ClockQuality("chrony", False, None, None)
    try:
        offset = float(f[4]) * 1000.0
        root_delay = float(f[10]) * 1000.0
        root_dispersion = float(f[11]) * 1000.0
        stratum = int(f[2])
    except ValueError:
        return ClockQuality("chrony", False, None, None)
    synced = f[13].strip().lower() not in ("not synchronised", "unknown") and 0 < stratum < 16
    # chrony's own bound on the true time: |offset| + root delay / 2 + root dispersion.
    bound = abs(offset) + root_delay / 2.0 + root_dispersion
    return ClockQuality("chrony", synced, round(offset, 3), round(bound, 3))


def read_clock_quality(timeout: float = 2.0) -> ClockQuality:
    try:
        out = subprocess.run(["chronyc", "-c", "tracking"], capture_output=True, text=True, timeout=timeout)
        if out.returncode == 0 and out.stdout.strip():
            return parse_chrony_tracking(out.stdout.splitlines()[0])
    except (OSError, subprocess.SubprocessError):
        pass
    try:
        out = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"], capture_output=True,
                             text=True, timeout=timeout)
        synced = out.stdout.strip() == "yes"
        return ClockQuality("timedatectl", synced, None, 50.0 if synced else None)
    except (OSError, subprocess.SubprocessError):
        return ClockQuality("none", False, None, None)


class SilenceWatch:
    """A sensor that should publish continuously: one ``sensor.stale`` when it falls silent past ``limit_s`` and one
    ``sensor.recovered`` when it speaks again (never one per tick). Before the first message nothing is reported: a
    sensor that never started is a configuration problem, not a stall."""

    def __init__(self, sensor: str, limit_s: float = 5.0) -> None:
        self.sensor = sensor
        self.limit_s = float(limit_s)
        self._last: Optional[float] = None
        self._stale_since: Optional[float] = None

    def heard(self, now: float) -> Optional[dict]:
        """A message arrived. Returns the recovery event payload if it had been reported stale."""
        out = None
        if self._stale_since is not None and self._last is not None:
            out = {"sensor": self.sensor, "silentS": round(now - self._last, 1), "declared": True}
        self._last = now
        self._stale_since = None
        return out

    def check(self, now: float) -> Optional[dict]:
        """Returns the stale event payload once, when the silence passes the limit."""
        if self._last is None or self._stale_since is not None or now - self._last < self.limit_s:
            return None
        self._stale_since = now
        return {"sensor": self.sensor, "silentS": round(now - self._last, 1), "limitS": self.limit_s, "declared": True}


class GateWatch:
    """The robot's authority gate (cmd_vel allowed or not, and why): one ``gate`` event when it changes, and again
    every ``repeat_s`` while it stays closed, so an investigation can measure how long a robot was held."""

    def __init__(self, repeat_s: float = 10.0) -> None:
        self.repeat_s = float(repeat_s)
        self._state: Optional[tuple[bool, str]] = None
        self._sent_at = -1e18

    def sample(self, open_: Optional[bool], reason: Optional[str], now: float) -> Optional[dict]:
        if open_ is None:
            return None
        state = (bool(open_), "OK" if open_ else (reason or "UNKNOWN"))
        changed = state != self._state
        if not changed and (state[0] or now - self._sent_at < self.repeat_s):
            return None
        self._state, self._sent_at = state, now
        return {"open": state[0], "reason": state[1], "changed": changed}
