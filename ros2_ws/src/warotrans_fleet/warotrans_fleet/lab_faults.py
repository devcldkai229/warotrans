"""Fault Injection Lab, robot side (pure logic, no ROS).

The backend's /api/lab sends {action: inject|clear, injectionId, scenarioId, ttlSeconds, parameters} on the robot's
``lab`` MQTT topic. The gateway obeys only when it runs with lab_enabled (LAB_ENABLED=true on a test robot), and every
fault ends by itself after its TTL even if the clear message never arrives (IOT-01 cuts the link that would carry it).

What each scenario does on the robot (the gateway applies the side effects; this module only keeps the state):
  IOT-01  MQTT link dropped for the TTL (the gateway disconnects and does not reconnect)
  IOT-02  lidar motor stopped for the TTL (/scan goes silent)
  IOT-03  the robot's clock reported +offset_s (evidence agent clock anchors)
  NAV-01  the next ``count`` Nav2 goals are sent to an unreachable point, so Nav2 really aborts and the gateway retries
  NAV-02  the next MOVE is accepted but not driven until the TTL ends
  MAP-01  traffic plans proposed with another map fingerprint
  OPS-04  after the next command result, a contradicting result for the same command is sent again
"""

from __future__ import annotations

import hashlib

from dataclasses import dataclass, field
from typing import Any, Mapping, Optional

ROBOT_SCENARIOS = frozenset({"IOT-01", "IOT-02", "IOT-03", "NAV-01", "NAV-02", "MAP-01", "OPS-04"})
MAX_TTL_SEC = 900.0
# A point no map contains: Nav2's planner refuses it (goal outside the costmap) and the goal aborts for real.
UNREACHABLE_GOAL = (-500.0, -500.0)
# MAP-01 reports another bundle's fingerprint. It must look like a real one (a hex digest of the same length): a name
# such as "lab-wrong-bundle" would hand the answer to the AI being graded (lab 2026-11-25).


class LabMessageError(ValueError):
    pass


@dataclass
class ActiveFault:
    injection_id: str
    scenario_id: str
    expires_at: float
    parameters: dict[str, str] = field(default_factory=dict)
    remaining: int = 0  # NAV-01: goals still to divert; OPS-04: duplicates still to send


@dataclass
class LabFaults:
    enabled: bool
    active: dict[str, ActiveFault] = field(default_factory=dict)  # by scenario id (one at a time per scenario)

    def handle(self, body: Mapping[str, Any], now: float) -> tuple[str, Optional[ActiveFault]]:
        """Apply one lab message. Returns ("inject" | "clear" | "ignored", the fault concerned)."""
        if not self.enabled:
            return "ignored", None
        if not isinstance(body, Mapping) or body.get("schemaVersion") != 1:
            raise LabMessageError("lab message must be an object with schemaVersion 1")
        action = body.get("action")
        injection_id = str(body.get("injectionId") or "")
        scenario = str(body.get("scenarioId") or "")
        if not injection_id or scenario not in ROBOT_SCENARIOS:
            raise LabMessageError(f"unknown lab scenario {scenario!r} or no injectionId")
        if action == "clear":
            fault = self.active.get(scenario)
            if fault is None or fault.injection_id != injection_id:
                return "ignored", None
            del self.active[scenario]
            return "clear", fault
        if action != "inject":
            raise LabMessageError(f"unknown lab action {action!r}")
        try:
            ttl = float(body.get("ttlSeconds"))
        except (TypeError, ValueError) as exc:
            raise LabMessageError("ttlSeconds must be a number") from exc
        if not 0 < ttl <= MAX_TTL_SEC:
            raise LabMessageError(f"ttlSeconds must be 0..{MAX_TTL_SEC:.0f}")
        params = {str(k): str(v) for k, v in (body.get("parameters") or {}).items()}
        fault = ActiveFault(injection_id, scenario, now + ttl, params)
        if scenario == "NAV-01":
            fault.remaining = max(1, min(20, _int(params.get("count"), 4)))
        elif scenario == "OPS-04":
            fault.remaining = 1
        self.active[scenario] = fault
        return "inject", fault

    def expire(self, now: float) -> list[ActiveFault]:
        """Faults whose TTL has passed, removed."""
        ended = [f for f in self.active.values() if f.expires_at <= now]
        for f in ended:
            del self.active[f.scenario_id]
        return ended

    # ------------------------------------------------------------------ queries used by the gateway
    def is_active(self, scenario: str) -> bool:
        return scenario in self.active

    def mqtt_cut_until(self) -> Optional[float]:
        f = self.active.get("IOT-01")
        return f.expires_at if f else None

    def clock_offset_s(self) -> float:
        f = self.active.get("IOT-03")
        return _float(f.parameters.get("offset_s"), 45.0) if f else 0.0

    def fingerprint(self, real: str) -> str:
        return fake_fingerprint(real) if "MAP-01" in self.active else real

    def divert_goal(self) -> Optional[tuple[float, float]]:
        """NAV-01: the unreachable point for this goal, while goals remain to divert (each call uses one)."""
        f = self.active.get("NAV-01")
        if f is None or f.remaining <= 0:
            return None
        f.remaining -= 1
        return UNREACHABLE_GOAL

    def freeze_next_move(self) -> Optional[ActiveFault]:
        """NAV-02: the fault, once (the first MOVE after injection is frozen; later ones run)."""
        f = self.active.get("NAV-02")
        if f is None or f.parameters.get("_used") == "1":
            return None
        f.parameters["_used"] = "1"
        return f

    def contradict_result(self) -> bool:
        """OPS-04: whether to send a contradicting duplicate of the result just published (once)."""
        f = self.active.get("OPS-04")
        if f is None or f.remaining <= 0:
            return False
        f.remaining -= 1
        return True

    def summary(self) -> dict[str, Any]:
        return {sid: {"injectionId": f.injection_id, "expiresAt": f.expires_at, "parameters": dict(f.parameters)}
                for sid, f in self.active.items()}


def contradicting_outcome(outcome: str) -> str:
    return "FAILED" if outcome == "SUCCEEDED" else "SUCCEEDED"


def _int(text: Optional[str], default: int) -> int:
    try:
        return int(str(text))
    except (TypeError, ValueError):
        return default


def _float(text: Optional[str], default: float) -> float:
    try:
        return float(str(text))
    except (TypeError, ValueError):
        return default


def fake_fingerprint(real: str) -> str:
    """A different, plausible bundle fingerprint: hex, as long as the real one, stable for one real fingerprint."""
    digest = hashlib.sha256(f"other-bundle:{real}".encode()).hexdigest()
    return (digest * 2)[: max(len(real), 16)]
