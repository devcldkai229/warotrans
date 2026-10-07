"""Monotonic sequence generators scoped to a bootId."""

from __future__ import annotations

import uuid
from dataclasses import dataclass, field


@dataclass
class SequenceClock:
    boot_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    _heartbeat: int = 0
    _telemetry: int = 0

    def next_heartbeat(self) -> int:
        self._heartbeat += 1
        return self._heartbeat

    def next_telemetry(self) -> int:
        self._telemetry += 1
        return self._telemetry

    @property
    def heartbeat_sequence(self) -> int:
        return self._heartbeat

    @property
    def telemetry_sequence(self) -> int:
        return self._telemetry
