"""Battery fields of the MQTT telemetry payload (pure, ROS-free).

UNKNOWN is null, never 0: batteryPercent is only a number when a fresh VOLTAGE_ONLY estimate (or the
simulation's fixed value) exists. Field contract: warotrans-system docs/mqtt-robot-contracts.md.
"""

from __future__ import annotations

import math
from typing import Any, Optional

MODE_VOLTAGE_ONLY = "VOLTAGE_ONLY"
MODE_SIMULATED = "SIMULATED"
MODE_UNAVAILABLE = "UNAVAILABLE"
CONFIDENCE_NONE = "NONE"
CHARGING_UNKNOWN = "UNKNOWN"


def _finite(value: Any) -> Optional[float]:
    try:
        v = float(value)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def unavailable_fields() -> dict[str, Any]:
    return {
        "batteryPercent": None,
        "batteryVoltage": None,
        "batteryEstimationMode": MODE_UNAVAILABLE,
        "batteryConfidence": CONFIDENCE_NONE,
        "batteryChargingState": CHARGING_UNKNOWN,
        "batteryMeasuredAt": None,
    }


def battery_fields(
    *,
    simulated_percent: Optional[float],
    estimate: Any = None,
    estimate_age_s: Optional[float] = None,
    stale_after_s: float = 3.0,
    measured_at: Optional[str] = None,
) -> dict[str, Any]:
    """``estimate`` is a warotrans_msgs/BatteryEstimate (or anything with the same attributes)."""

    if simulated_percent is not None:
        return {
            "batteryPercent": round(min(100.0, max(0.0, float(simulated_percent))), 1),
            "batteryVoltage": None,
            "batteryEstimationMode": MODE_SIMULATED,
            "batteryConfidence": None,
            "batteryChargingState": CHARGING_UNKNOWN,
            "batteryMeasuredAt": measured_at,
        }
    if estimate is None or estimate_age_s is None or estimate_age_s > stale_after_s:
        return unavailable_fields()
    if str(getattr(estimate, "mode", "")) != MODE_VOLTAGE_ONLY:
        return unavailable_fields()
    fraction = _finite(getattr(estimate, "percentage", None))
    if fraction is None:
        return unavailable_fields()
    voltage = _finite(getattr(estimate, "voltage", None))
    return {
        "batteryPercent": round(min(1.0, max(0.0, fraction)) * 100.0, 1),
        "batteryVoltage": round(voltage, 3) if voltage is not None else None,
        "batteryEstimationMode": MODE_VOLTAGE_ONLY,
        "batteryConfidence": str(getattr(estimate, "confidence", "") or CONFIDENCE_NONE),
        "batteryChargingState": str(getattr(estimate, "charging_state", "") or CHARGING_UNKNOWN),
        "batteryMeasuredAt": measured_at,
    }
