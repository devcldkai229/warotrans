"""Technical diagnostics snapshot (RB-10), sent on MQTT ``robots/{code}/health``.

Pure Python: the node feeds it message arrival times and probes; it never invents a value. Anything that cannot be
measured is reported with ``value=None`` and level UNKNOWN; a source that stopped is STALE.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Callable, Optional

OK = "OK"
WARN = "WARN"
ERROR = "ERROR"
STALE = "STALE"
UNKNOWN = "UNKNOWN"


@dataclass
class TopicCheck:
    """A sensor stream expected at about ``min_hz``. ``required`` ones turn ERROR when silent, others STALE."""

    name: str
    topic: str
    min_hz: float
    required: bool = True
    _stamps: list[float] = field(default_factory=list)

    def record(self, now: float) -> None:
        self._stamps.append(now)
        # Keep ~5 s of arrivals for the rate.
        cutoff = now - 5.0
        while self._stamps and self._stamps[0] < cutoff:
            self._stamps.pop(0)

    def evaluate(self, now: float) -> dict:
        if not self._stamps:
            return _item(self.name, ERROR if self.required else UNKNOWN, None, "Hz", f"no message on {self.topic}")
        age = now - self._stamps[-1]
        window = [t for t in self._stamps if t >= now - 5.0]
        rate = len(window) / 5.0
        if age > max(2.0, 3.0 / max(self.min_hz, 0.1)):
            return _item(self.name, ERROR if self.required else STALE, round(rate, 2), "Hz",
                         f"{self.topic} silent for {age:.1f} s")
        level = OK if rate >= self.min_hz else WARN
        return _item(self.name, level, round(rate, 2), "Hz",
                     None if level == OK else f"{self.topic} at {rate:.1f} Hz, expected {self.min_hz:g}")


def _item(name: str, level: str, value: Optional[float], unit: Optional[str], message: Optional[str]) -> dict:
    return {
        "name": name,
        "level": level,
        "value": value,
        "unit": unit,
        "message": message,
        "measuredAt": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
    }


def cpu_load_item(read_loadavg: Callable[[], Optional[float]] = None, cpu_count: Optional[int] = None) -> dict:
    load = (read_loadavg or _read_loadavg)()
    cpus = cpu_count or os.cpu_count() or 1
    if load is None:
        return _item("cpu_load", UNKNOWN, None, "%", "load average not available")
    percent = round(100.0 * load / cpus, 1)
    return _item("cpu_load", OK if percent < 80 else WARN if percent < 95 else ERROR, percent, "%", None)


def temperature_item(read_celsius: Callable[[], Optional[float]] = None) -> dict:
    celsius = (read_celsius or _read_cpu_temperature)()
    if celsius is None:
        return _item("cpu_temperature", UNKNOWN, None, "C", "no thermal sensor")
    level = OK if celsius < 70 else WARN if celsius < 80 else ERROR
    return _item("cpu_temperature", level, round(celsius, 1), "C", None)


def memory_item(read_available_percent: Callable[[], Optional[float]] = None) -> dict:
    available = (read_available_percent or _read_memory_available_percent)()
    if available is None:
        return _item("memory_available", UNKNOWN, None, "%", "meminfo not available")
    level = OK if available > 20 else WARN if available > 10 else ERROR
    return _item("memory_available", level, round(available, 1), "%", None)


def tf_item(pose_age_sec: Optional[float], max_age_sec: float) -> dict:
    if pose_age_sec is None:
        return _item("tf_localization", ERROR, None, "s", "map -> base transform missing")
    level = OK if pose_age_sec <= max_age_sec else STALE
    return _item("tf_localization", level, round(pose_age_sec, 3), "s",
                 None if level == OK else f"transform {pose_age_sec:.2f} s old")


def nav2_item(ready: Optional[bool]) -> dict:
    if ready is None:
        return _item("nav2", UNKNOWN, None, None, "navigator not configured")
    return _item("nav2", OK if ready else ERROR, None, None, None if ready else "navigate_to_pose server not available")


def build_health(*, robot_code: str, items: list[dict]) -> dict:
    return {
        "schemaVersion": 1,
        "robotCode": robot_code,
        "sentAt": datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
        "items": items,
    }


def parse_topic_checks(specs: list[str]) -> list[tuple[TopicCheck, str]]:
    """``name:topic:type:min_hz[:optional]`` → (check, message type). Bad specs are skipped."""
    checks: list[tuple[TopicCheck, str]] = []
    for spec in specs:
        parts = [p.strip() for p in spec.split(":")] if spec else []
        if len(parts) < 4:
            continue
        name, topic, msg_type, hz = parts[:4]
        try:
            min_hz = float(hz)
        except ValueError:
            continue
        checks.append((TopicCheck(name, topic, min_hz, required=not (len(parts) > 4 and parts[4] == "optional")), msg_type))
    return checks


def _read_loadavg() -> Optional[float]:
    try:
        with open("/proc/loadavg", encoding="ascii") as f:
            return float(f.read().split()[0])
    except (OSError, ValueError, IndexError):
        return None


def _read_cpu_temperature() -> Optional[float]:
    try:
        with open("/sys/class/thermal/thermal_zone0/temp", encoding="ascii") as f:
            return int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
        return None


def _read_memory_available_percent() -> Optional[float]:
    try:
        values: dict[str, int] = {}
        with open("/proc/meminfo", encoding="ascii") as f:
            for line in f:
                key, _, rest = line.partition(":")
                values[key] = int(rest.split()[0])
        return 100.0 * values["MemAvailable"] / values["MemTotal"]
    except (OSError, ValueError, KeyError, IndexError, ZeroDivisionError):
        return None
