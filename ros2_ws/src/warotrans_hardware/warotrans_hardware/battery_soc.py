"""Voltage-only battery SOC estimation (no ROS imports, deterministic: every call takes the time).

Hardware: a 0-25 V voltage divider module (nominal 30k/7.5k) on an ESP32-S3 ADC1 pin. There is no current
sensor, so this estimator:
  * converts the calibrated pin voltage to pack volts, filters it (median + EMA) and rejects faults;
  * maps a load-compensated open-circuit voltage (OCV) estimate to SOC with a battery-specific OCV table;
  * uses robot motion (cmd_vel / odom) as the only load proxy: SOC may only fall while the robot moves or the
    voltage is still recovering, and is corrected towards OCV only after the robot has rested;
  * never infers charging from the voltage (charging_state stays UNKNOWN);
  * reports UNAVAILABLE (percentage None) instead of guessing when there is no fresh valid reading.

Every number comes from BatteryConfig (loaded from config/battery.yaml). The defaults below are only the
uncalibrated starting point and must be replaced by the measured values (docs/hardware/battery-voltage-sensor.md).
"""

from __future__ import annotations

import json
import math
import os
from bisect import bisect_left
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median
from typing import Any, Optional

MODE_VOLTAGE_ONLY = "VOLTAGE_ONLY"
MODE_UNAVAILABLE = "UNAVAILABLE"
CONFIDENCE_MEDIUM = "MEDIUM"
CONFIDENCE_LOW = "LOW"
CONFIDENCE_NONE = "NONE"
CHARGING_UNKNOWN = "UNKNOWN"

FAULT_NO_SAMPLES = "no_samples"
FAULT_STALE = "stale"
FAULT_WARMING_UP = "warming_up"
FAULT_ADC_DISCONNECTED = "adc_disconnected"
FAULT_ADC_SATURATED = "adc_saturated"
FAULT_OUT_OF_RANGE = "out_of_range"

# Samsung INR21700-50S resting cell voltage -> SOC %, starting point only (CHƯA HIỆU CHUẨN).
DEFAULT_OCV_TABLE: tuple[tuple[float, float], ...] = (
    (3.00, 0.0), (3.20, 2.0), (3.30, 4.0), (3.40, 8.0), (3.45, 11.0), (3.50, 15.0), (3.55, 20.0),
    (3.60, 26.0), (3.65, 33.0), (3.70, 41.0), (3.75, 48.0), (3.80, 55.0), (3.85, 61.0), (3.90, 67.0),
    (3.95, 73.0), (4.00, 79.0), (4.05, 85.0), (4.10, 90.0), (4.15, 95.0), (4.20, 100.0),
)


@dataclass
class BatteryConfig:
    cells_in_series: int = 3
    design_capacity_ah: float = 10.0
    # pack_V = divider_gain * pin_V + offset_v, unless calibration_points (pin_V, pack_V) has >= 2 points
    divider_gain: float = 5.0
    offset_v: float = 0.0
    calibration_points: list[tuple[float, float]] = field(default_factory=list)
    ocv_table: list[tuple[float, float]] = field(default_factory=lambda: list(DEFAULT_OCV_TABLE))
    adc_disconnected_pin_mv: float = 100.0
    adc_saturated_pin_mv: float = 3050.0
    plausible_min_v: float = 8.0
    plausible_max_v: float = 13.5
    stale_after_s: float = 2.0
    median_window: int = 5
    ema_tau_s: float = 5.0
    # Load-induced sag compensation. idle_sag_v: pack drop with the robot powered but still (Pi + boards
    # always draw current); per m/s and per rad/s on top while driving. 0 until measured (hardware step V6).
    idle_sag_v: float = 0.0
    sag_v_per_mps: float = 0.0
    sag_v_per_radps: float = 0.0
    motion_linear_threshold_mps: float = 0.01
    motion_angular_threshold_radps: float = 0.02
    # Motion readings older than this are treated as "not moving" for the sag compensation.
    motion_timeout_s: float = 0.5
    rest_settle_s: float = 120.0
    max_drop_pct_per_min: float = 3.0
    rest_slew_pct_per_min: float = 2.0
    rest_hysteresis_pct: float = 3.0
    restore_max_age_s: float = 12 * 3600.0
    restore_tolerance_pct: float = 15.0

    def __post_init__(self) -> None:
        if self.cells_in_series < 1:
            raise ValueError("cells_in_series must be >= 1")
        if self.median_window < 1:
            raise ValueError("median_window must be >= 1")
        if self.ema_tau_s <= 0 or self.stale_after_s <= 0:
            raise ValueError("ema_tau_s and stale_after_s must be > 0")
        if not self.plausible_min_v < self.plausible_max_v:
            raise ValueError("plausible_min_v must be < plausible_max_v")
        table = [(float(v), float(s)) for v, s in self.ocv_table]
        if len(table) < 2:
            raise ValueError("ocv_table needs at least 2 points")
        for (v0, s0), (v1, s1) in zip(table, table[1:]):
            if not (v1 > v0 and s1 >= s0):
                raise ValueError("ocv_table must be strictly increasing in voltage and non-decreasing in SOC")
        if not (0.0 <= table[0][1] and table[-1][1] <= 100.0):
            raise ValueError("ocv_table SOC must be within 0..100")
        self.ocv_table = table
        points = sorted((float(p), float(v)) for p, v in self.calibration_points)
        if len(points) == 1:
            raise ValueError("calibration_points needs 0 or >= 2 points")
        for (p0, _), (p1, _) in zip(points, points[1:]):
            if p1 <= p0:
                raise ValueError("calibration_points pin voltages must be distinct")
        self.calibration_points = points

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "BatteryConfig":
        known = {f for f in cls.__dataclass_fields__}
        unknown = set(data) - known
        if unknown:
            raise ValueError(f"unknown battery config keys: {sorted(unknown)}")
        values = dict(data)
        for key in ("ocv_table", "calibration_points"):
            if key in values:
                values[key] = [tuple(p) for p in values[key]]
        return cls(**values)


def _interp(points: list[tuple[float, float]], x: float, *, clamp: bool) -> float:
    xs = [p[0] for p in points]
    if x <= xs[0]:
        if clamp:
            return points[0][1]
        (x0, y0), (x1, y1) = points[0], points[1]
    elif x >= xs[-1]:
        if clamp:
            return points[-1][1]
        (x0, y0), (x1, y1) = points[-2], points[-1]
    else:
        i = bisect_left(xs, x)
        (x0, y0), (x1, y1) = points[i - 1], points[i]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def pack_voltage_from_pin(config: BatteryConfig, pin_mv: float) -> float:
    pin_v = pin_mv / 1000.0
    if config.calibration_points:
        return _interp(config.calibration_points, pin_v, clamp=False)
    return config.divider_gain * pin_v + config.offset_v


def soc_from_cell_ocv(config: BatteryConfig, cell_v: float) -> float:
    """SOC % from the battery-specific OCV curve (piecewise between measured points, clamped 0..100)."""
    return min(100.0, max(0.0, _interp(config.ocv_table, cell_v, clamp=True)))


@dataclass(frozen=True)
class BatteryEstimateResult:
    mode: str
    confidence: str
    charging_state: str
    percentage: Optional[float]  # 0..1
    voltage: Optional[float]
    ocv_estimate: Optional[float]
    rested: bool
    fault: str
    sample_age_s: Optional[float]


class VoltageSocEstimator:
    """Times are monotonic seconds supplied by the caller (the ROS node uses its clock, tests a fake one)."""

    def __init__(self, config: BatteryConfig) -> None:
        self._cfg = config
        self._window: deque[float] = deque(maxlen=config.median_window)
        self._filtered_v: Optional[float] = None
        self._ocv_v: Optional[float] = None
        self._last_filter_t: Optional[float] = None
        self._last_sample_t: Optional[float] = None
        self._last_valid_t: Optional[float] = None
        self._sample_fault = ""
        self._soc: Optional[float] = None  # percent
        self._soc_t: Optional[float] = None
        self._from_restore = False
        self._restored_soc: Optional[float] = None
        self._first_sample_t: Optional[float] = None
        self._last_motion_t: Optional[float] = None
        self._speed: tuple[float, float] = (0.0, 0.0)
        self._speed_t: Optional[float] = None

    # ------------------------------------------------------------------ inputs

    def update_motion(self, t: float, linear_mps: float, angular_radps: float) -> None:
        if not (math.isfinite(linear_mps) and math.isfinite(angular_radps)):
            return
        self._speed = (abs(linear_mps), abs(angular_radps))
        self._speed_t = t
        if self._is_motion(self._speed):
            self._last_motion_t = t

    def restore(self, data: dict[str, Any], wall_now: float) -> bool:
        """Offer a persisted SOC; it is only used if recent and consistent with the first live reading."""
        try:
            soc = float(data["soc_pct"])
            saved_at = float(data["wall_time"])
        except (KeyError, TypeError, ValueError):
            return False
        age = wall_now - saved_at
        if not (0.0 <= soc <= 100.0) or not (0.0 <= age <= self._cfg.restore_max_age_s):
            return False
        self._restored_soc = soc
        return True

    def add_sample(self, t: float, pin_mv: float) -> None:
        cfg = self._cfg
        self._last_sample_t = t
        if self._first_sample_t is None:
            self._first_sample_t = t
        if not math.isfinite(pin_mv) or pin_mv < cfg.adc_disconnected_pin_mv:
            self._fault(FAULT_ADC_DISCONNECTED)
            return
        if pin_mv > cfg.adc_saturated_pin_mv:
            self._fault(FAULT_ADC_SATURATED)
            return
        pack_v = pack_voltage_from_pin(cfg, pin_mv)
        if not cfg.plausible_min_v <= pack_v <= cfg.plausible_max_v:
            self._fault(FAULT_OUT_OF_RANGE)
            return

        self._sample_fault = ""
        self._last_valid_t = t
        self._window.append(pack_v)
        spike_free = median(self._window)
        if self._filtered_v is None or self._last_filter_t is None:
            self._filtered_v = spike_free
        else:
            dt = max(0.0, t - self._last_filter_t)
            alpha = 1.0 - math.exp(-dt / cfg.ema_tau_s)
            self._filtered_v += alpha * (spike_free - self._filtered_v)
        self._last_filter_t = t
        if len(self._window) < cfg.median_window:
            return

        lin, ang = self._current_speed(t)
        self._ocv_v = self._filtered_v + cfg.idle_sag_v + cfg.sag_v_per_mps * lin + cfg.sag_v_per_radps * ang
        candidate = soc_from_cell_ocv(cfg, self._ocv_v / cfg.cells_in_series)
        self._update_soc(t, candidate)

    # ------------------------------------------------------------------ output

    def estimate(self, t: float) -> BatteryEstimateResult:
        age = None if self._last_valid_t is None else max(0.0, t - self._last_valid_t)
        fault = ""
        if self._last_sample_t is None:
            fault = FAULT_NO_SAMPLES
        elif t - self._last_sample_t > self._cfg.stale_after_s:
            fault = FAULT_STALE
        elif self._sample_fault:
            fault = self._sample_fault
        elif self._soc is None:
            fault = FAULT_WARMING_UP
        if fault:
            return BatteryEstimateResult(MODE_UNAVAILABLE, CONFIDENCE_NONE, CHARGING_UNKNOWN,
                                         None, None, None, False, fault, age)
        rested = self.is_rested(t)
        confidence = CONFIDENCE_MEDIUM if rested and not self._from_restore else CONFIDENCE_LOW
        return BatteryEstimateResult(MODE_VOLTAGE_ONLY, confidence, CHARGING_UNKNOWN,
                                     self._soc / 100.0, self._filtered_v, self._ocv_v, rested, "", age)

    def snapshot(self, wall_now: float) -> Optional[dict[str, Any]]:
        if self._soc is None:
            return None
        return {"soc_pct": round(self._soc, 3), "wall_time": wall_now}

    def is_rested(self, t: float) -> bool:
        reference = self._last_motion_t if self._last_motion_t is not None else self._first_sample_t
        return reference is not None and t - reference >= self._cfg.rest_settle_s

    # ------------------------------------------------------------------ internals

    def _fault(self, fault: str) -> None:
        # A faulty reading invalidates the filter: a reconnected sensor must refill the window first.
        self._sample_fault = fault
        self._window.clear()
        self._filtered_v = None
        self._last_filter_t = None
        self._ocv_v = None

    def _is_motion(self, speed: tuple[float, float]) -> bool:
        return (speed[0] > self._cfg.motion_linear_threshold_mps
                or speed[1] > self._cfg.motion_angular_threshold_radps)

    def _current_speed(self, t: float) -> tuple[float, float]:
        if self._speed_t is None or t - self._speed_t > self._cfg.motion_timeout_s:
            return 0.0, 0.0
        return self._speed

    def _update_soc(self, t: float, candidate: float) -> None:
        cfg = self._cfg
        if self._soc is None:
            restored = self._restored_soc
            if restored is not None and abs(restored - candidate) <= cfg.restore_tolerance_pct:
                self._soc, self._from_restore = restored, True
            else:
                self._soc, self._from_restore = candidate, False
            self._restored_soc = None
            self._soc_t = t
            return

        dt_min = max(0.0, t - (self._soc_t or t)) / 60.0
        self._soc_t = t
        if self.is_rested(t):
            # Near OCV: follow it, slowly, both ways; rises need more than the hysteresis band.
            step = cfg.rest_slew_pct_per_min * dt_min
            if candidate < self._soc:
                self._soc = max(candidate, self._soc - step)
            elif candidate > self._soc + cfg.rest_hysteresis_pct:
                self._soc = min(candidate, self._soc + step)
            self._from_restore = False
        elif candidate < self._soc:
            # Under load / recovering: the voltage is below OCV, so only let SOC fall, rate-limited.
            # It never rises here, so the voltage recovering after the motors stop does not jump SOC up.
            self._soc = max(candidate, self._soc - cfg.max_drop_pct_per_min * dt_min)
        self._soc = min(100.0, max(0.0, self._soc))


def load_state(path: Path) -> Optional[dict[str, Any]]:
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


def save_state(path: Path, data: dict[str, Any]) -> None:
    """Atomic write: a power cut mid-write leaves the previous file intact."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + ".tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


# sensor_msgs/BatteryState constants (kept here so the mapping is testable without ROS).
POWER_SUPPLY_STATUS_UNKNOWN = 0
POWER_SUPPLY_HEALTH_UNKNOWN = 0
POWER_SUPPLY_TECHNOLOGY_LION = 2


def battery_state_fields(result: BatteryEstimateResult, config: BatteryConfig) -> dict[str, Any]:
    """sensor_msgs/BatteryState fields: NaN = not measured (current, charge, temperature) or unknown."""
    nan = float("nan")
    return {
        "voltage": result.voltage if result.voltage is not None else nan,
        "temperature": nan,
        "current": nan,  # no current sensor
        "charge": nan,
        "capacity": nan,
        "design_capacity": float(config.design_capacity_ah),
        "percentage": result.percentage if result.percentage is not None else nan,
        # Charging cannot be told from voltage alone and nothing else confirms it.
        "power_supply_status": POWER_SUPPLY_STATUS_UNKNOWN,
        "power_supply_health": POWER_SUPPLY_HEALTH_UNKNOWN,
        "power_supply_technology": POWER_SUPPLY_TECHNOLOGY_LION,
        "present": result.mode == MODE_VOLTAGE_ONLY,
        "cell_voltage": [nan] * config.cells_in_series,  # cells are not measured individually
        "cell_temperature": [nan] * config.cells_in_series,
    }
