"""Monotonic sequence generators scoped to a bootId."""

from __future__ import annotations

import threading
import uuid
from dataclasses import dataclass, field


@dataclass
class SequenceClock:
    boot_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    _heartbeat: int = 0
    _telemetry: int = 0
    _interaction: int = 0
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False, compare=False)

    def next_heartbeat(self) -> int:
        with self._lock:
            self._heartbeat += 1
            return self._heartbeat

    def next_telemetry(self) -> int:
        with self._lock:
            self._telemetry += 1
            return self._telemetry

    def next_interaction(self) -> int:
        with self._lock:
            self._interaction += 1
            return self._interaction

    @property
    def heartbeat_sequence(self) -> int:
        return self._heartbeat

    @property
    def telemetry_sequence(self) -> int:
        return self._telemetry

    @property
    def interaction_sequence(self) -> int:
        return self._interaction
