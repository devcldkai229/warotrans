import math
import random

import pytest

from warotrans_hardware.battery_soc import (
    CHARGING_UNKNOWN,
    CONFIDENCE_LOW,
    CONFIDENCE_MEDIUM,
    CONFIDENCE_NONE,
    FAULT_ADC_DISCONNECTED,
    FAULT_ADC_SATURATED,
    FAULT_NO_SAMPLES,
    FAULT_OUT_OF_RANGE,
    FAULT_STALE,
    FAULT_WARMING_UP,
    MODE_UNAVAILABLE,
    MODE_VOLTAGE_ONLY,
    POWER_SUPPLY_STATUS_UNKNOWN,
    POWER_SUPPLY_TECHNOLOGY_LION,
    BatteryConfig,
    VoltageSocEstimator,
    battery_state_fields,
    load_state,
    pack_voltage_from_pin,
    save_state,
    soc_from_cell_ocv,
)

DT = 0.2  # firmware reports a battery sample every 200 ms


def pin_mv(pack_v: float, gain: float = 5.0) -> float:
    return pack_v / gain * 1000.0


def pack_for_soc(config: BatteryConfig, soc: float) -> float:
    """Inverse of the OCV table (test helper)."""
    table = config.ocv_table
    for (v0, s0), (v1, s1) in zip(table, table[1:]):
        if s0 <= soc <= s1 and s1 > s0:
            return (v0 + (v1 - v0) * (soc - s0) / (s1 - s0)) * config.cells_in_series
    raise ValueError(soc)


class Sim:
    """Feeds samples at the firmware rate with a fake clock."""

    def __init__(self, config: BatteryConfig | None = None, t0: float = 1000.0) -> None:
        self.cfg = config or BatteryConfig()
        self.est = VoltageSocEstimator(self.cfg)
        self.t = t0

    def run(self, seconds: float, pack_v, *, lin: float = 0.0, ang: float = 0.0) -> None:
        steps = int(round(seconds / DT))
        for _ in range(steps):
            self.t += DT
            self.est.update_motion(self.t, lin, ang)
            v = pack_v(self.t) if callable(pack_v) else pack_v
            self.est.add_sample(self.t, pin_mv(v))

    def soc(self) -> float:
        result = self.est.estimate(self.t)
        assert result.percentage is not None, result
        return result.percentage * 100.0

    def result(self):
        return self.est.estimate(self.t)


# ----------------------------------------------------------------------------- config & curve

def test_default_ocv_table_is_monotonic_and_bounded():
    cfg = BatteryConfig()
    volts = [v for v, _ in cfg.ocv_table]
    socs = [s for _, s in cfg.ocv_table]
    assert volts == sorted(volts) and len(set(volts)) == len(volts)
    assert socs == sorted(socs)
    assert socs[0] == 0.0 and socs[-1] == 100.0


@pytest.mark.parametrize(
    "table",
    [[(3.0, 0.0)], [(3.0, 0.0), (3.0, 50.0)], [(3.0, 50.0), (4.2, 10.0)], [(3.0, -1.0), (4.2, 100.0)]],
)
def test_invalid_ocv_table_rejected(table):
    with pytest.raises(ValueError):
        BatteryConfig(ocv_table=table)


def test_unknown_config_key_rejected():
    with pytest.raises(ValueError):
        BatteryConfig.from_dict({"divider_gain": 5.0, "typo_key": 1})


def test_ocv_curve_is_not_a_linear_voltage_interpolation():
    cfg = BatteryConfig()
    # Linear 3.0..4.2 V would give 50% at 3.6 V; the INR21700-50S curve gives far less.
    assert soc_from_cell_ocv(cfg, 3.60) == pytest.approx(26.0)
    assert soc_from_cell_ocv(cfg, 4.20) == 100.0
    assert soc_from_cell_ocv(cfg, 4.35) == 100.0
    assert soc_from_cell_ocv(cfg, 2.50) == 0.0


def test_pin_to_pack_gain_offset_and_multipoint_calibration():
    assert pack_voltage_from_pin(BatteryConfig(), 2520.0) == pytest.approx(12.6)
    assert pack_voltage_from_pin(BatteryConfig(divider_gain=5.1, offset_v=0.05), 2000.0) == pytest.approx(10.25)
    cfg = BatteryConfig(calibration_points=[(2.0, 10.1), (2.5, 12.55)])
    assert pack_voltage_from_pin(cfg, 2250.0) == pytest.approx(11.325)
    assert pack_voltage_from_pin(cfg, 2520.0) == pytest.approx(12.648)  # extrapolates the end segment
    with pytest.raises(ValueError):
        BatteryConfig(calibration_points=[(2.0, 10.0)])


# ----------------------------------------------------------------------------- availability

def test_no_samples_is_unavailable_not_zero():
    est = VoltageSocEstimator(BatteryConfig())
    r = est.estimate(5.0)
    assert r.mode == MODE_UNAVAILABLE and r.fault == FAULT_NO_SAMPLES
    assert r.percentage is None and r.voltage is None and r.confidence == CONFIDENCE_NONE


def test_warming_up_until_the_median_window_is_full():
    sim = Sim()
    sim.run(DT * (sim.cfg.median_window - 1), 11.4)
    assert sim.result().fault == FAULT_WARMING_UP and sim.result().percentage is None
    sim.run(DT, 11.4)
    assert sim.result().mode == MODE_VOLTAGE_ONLY


def test_stale_sensor_becomes_unavailable_and_recovers():
    sim = Sim()
    sim.run(5.0, 11.4)
    assert sim.result().mode == MODE_VOLTAGE_ONLY
    late = sim.t + sim.cfg.stale_after_s + 0.1
    r = sim.est.estimate(late)
    assert r.mode == MODE_UNAVAILABLE and r.fault == FAULT_STALE and r.percentage is None
    assert r.sample_age_s == pytest.approx(sim.cfg.stale_after_s + 0.1)
    sim.t = late
    sim.run(DT, 11.4)
    assert sim.result().mode == MODE_VOLTAGE_ONLY


@pytest.mark.parametrize(
    "pin,fault",
    [(0.0, FAULT_ADC_DISCONNECTED), (float("nan"), FAULT_ADC_DISCONNECTED),
     (3200.0, FAULT_ADC_SATURATED), (pin_mv(7.0), FAULT_OUT_OF_RANGE), (pin_mv(14.5), FAULT_OUT_OF_RANGE)],
)
def test_sensor_faults_are_unavailable(pin, fault):
    sim = Sim()
    sim.run(5.0, 11.4)
    sim.t += DT
    sim.est.add_sample(sim.t, pin)
    r = sim.result()
    assert r.mode == MODE_UNAVAILABLE and r.fault == fault and r.percentage is None


def test_reconnected_sensor_must_refill_the_filter():
    sim = Sim()
    sim.run(5.0, 11.4)
    sim.t += DT
    sim.est.add_sample(sim.t, 0.0)
    sim.run(DT * (sim.cfg.median_window - 1), 11.4)
    assert sim.result().fault == FAULT_WARMING_UP or sim.result().mode == MODE_VOLTAGE_ONLY
    sim.run(DT, 11.4)
    assert sim.result().mode == MODE_VOLTAGE_ONLY


# ----------------------------------------------------------------------------- estimation

def test_rested_reading_matches_the_ocv_table_with_medium_confidence():
    sim = Sim()
    sim.run(sim.cfg.rest_settle_s + 10, pack_for_soc(sim.cfg, 55.0))
    r = sim.result()
    assert r.confidence == CONFIDENCE_MEDIUM and r.rested
    assert sim.soc() == pytest.approx(55.0, abs=0.5)
    assert r.voltage == pytest.approx(pack_for_soc(sim.cfg, 55.0), abs=0.01)


def test_constant_discharge_at_rest_tracks_soc_down():
    sim = Sim()
    start, end = pack_for_soc(sim.cfg, 80.0), pack_for_soc(sim.cfg, 60.0)
    duration = 20 * 60.0
    t0 = sim.t
    sim.run(duration, lambda t: start + (end - start) * min(1.0, (t - t0) / duration))
    sim.run(5 * 60.0, end)
    assert sim.soc() == pytest.approx(60.0, abs=1.0)


def test_variable_discharge_under_load_only_falls_and_is_rate_limited():
    sim = Sim()
    sim.run(sim.cfg.rest_settle_s + 5, pack_for_soc(sim.cfg, 70.0))
    before = sim.soc()
    history = []
    rest = pack_for_soc(sim.cfg, 70.0)
    for k in range(60):  # 60 s of bursts: hard sag and partial recovery while driving
        sag = 0.6 if k % 2 == 0 else 0.15
        sim.run(1.0, rest - sag, lin=0.3)
        history.append(sim.soc())
        assert sim.result().confidence == CONFIDENCE_LOW
    assert all(b <= a + 1e-9 for a, b in zip(history, history[1:])), "SOC rose while loaded"
    max_drop = sim.cfg.max_drop_pct_per_min * 1.05
    assert before - history[-1] <= max_drop


def test_voltage_recovery_after_motors_stop_does_not_jump_soc():
    sim = Sim()
    rest_v = pack_for_soc(sim.cfg, 60.0)
    sim.run(sim.cfg.rest_settle_s + 5, rest_v)
    sim.run(30.0, rest_v - 0.5, lin=0.4)  # driving: sag
    loaded = sim.soc()
    # Motors stop: the voltage springs back within seconds.
    sim.run(sim.cfg.rest_settle_s - 10, rest_v)
    assert sim.soc() <= loaded + 1e-9
    assert not sim.result().rested and sim.result().confidence == CONFIDENCE_LOW
    # Only after resting does SOC move back towards OCV, limited by the slew rate.
    sim.run(20.0, rest_v)
    assert sim.result().rested
    sim.run(10 * 60.0, rest_v)
    assert sim.soc() == pytest.approx(60.0, abs=sim.cfg.rest_hysteresis_pct + 0.5)


def test_sag_compensation_raises_the_ocv_estimate_under_load():
    cfg = BatteryConfig(sag_v_per_mps=1.0)
    sim = Sim(cfg)
    rest_v = pack_for_soc(cfg, 60.0)
    sim.run(30.0, rest_v - 0.3, lin=0.3)
    assert sim.result().ocv_estimate == pytest.approx(rest_v, abs=0.02)
    assert sim.soc() == pytest.approx(60.0, abs=1.0)


def test_idle_sag_offset_is_added_at_rest():
    cfg = BatteryConfig(idle_sag_v=0.07)
    sim = Sim(cfg)
    rest_v = pack_for_soc(cfg, 60.0)
    sim.run(cfg.rest_settle_s + 10, rest_v - 0.07)
    assert sim.soc() == pytest.approx(60.0, abs=0.5)


def test_noise_and_spikes_are_filtered():
    rng = random.Random(42)
    sim = Sim()
    true_v = pack_for_soc(sim.cfg, 50.0)

    def noisy(_t):
        v = true_v + rng.gauss(0.0, 0.03)
        return v - 1.5 if rng.random() < 0.05 else v  # 5 % single-sample spikes

    sim.run(sim.cfg.rest_settle_s + 120, noisy)
    assert sim.result().voltage == pytest.approx(true_v, abs=0.03)
    assert sim.soc() == pytest.approx(50.0, abs=2.0)


def test_long_term_drift_is_corrected_while_resting():
    sim = Sim()
    true_v = pack_for_soc(sim.cfg, 50.0)
    assert sim.est.restore({"soc_pct": 62.0, "wall_time": 0.0}, wall_now=60.0)
    sim.run(DT * sim.cfg.median_window, true_v)
    assert sim.soc() == pytest.approx(62.0)  # within tolerance -> seeded from the restore
    assert sim.result().confidence == CONFIDENCE_LOW
    sim.run(30.0, true_v)
    # Not rested yet: SOC may only fall towards the reading, at most max_drop_pct_per_min.
    assert 62.0 - sim.cfg.max_drop_pct_per_min * 0.5 - 0.01 <= sim.soc() < 62.0
    sim.run(sim.cfg.rest_settle_s + 10 * 60.0, true_v)
    assert sim.soc() == pytest.approx(50.0, abs=0.5)
    assert sim.result().confidence == CONFIDENCE_MEDIUM


def test_soc_stays_within_bounds():
    sim = Sim()
    sim.run(sim.cfg.rest_settle_s + 30, 13.3)  # above full, still plausible
    assert sim.soc() == 100.0
    sim2 = Sim()
    sim2.run(sim2.cfg.rest_settle_s + 30, 8.5)  # deeply discharged, still plausible
    assert sim2.soc() == 0.0
    assert 0.0 <= sim2.result().percentage <= 1.0


def test_charging_is_never_inferred_from_rising_voltage():
    sim = Sim()
    sim.run(sim.cfg.rest_settle_s + 5, pack_for_soc(sim.cfg, 40.0))
    t0 = sim.t
    sim.run(20 * 60.0, lambda t: pack_for_soc(sim.cfg, 40.0) + 0.002 * (t - t0))  # rising like a charger
    assert sim.result().charging_state == CHARGING_UNKNOWN


# ----------------------------------------------------------------------------- reboot / persistence

def test_restore_ignored_when_too_old_or_inconsistent():
    cfg = BatteryConfig()
    old = VoltageSocEstimator(cfg)
    assert not old.restore({"soc_pct": 50.0, "wall_time": 0.0}, wall_now=cfg.restore_max_age_s + 1)
    sim = Sim(cfg)
    assert sim.est.restore({"soc_pct": 90.0, "wall_time": 0.0}, wall_now=10.0)
    sim.run(5.0, pack_for_soc(cfg, 40.0))
    assert sim.soc() == pytest.approx(40.0, abs=0.5)  # 90 vs 40 > tolerance -> live reading wins
    assert not VoltageSocEstimator(cfg).restore({"soc_pct": "x"}, wall_now=0.0)
    assert not VoltageSocEstimator(cfg).restore({"soc_pct": 150.0, "wall_time": 0.0}, wall_now=1.0)


def test_state_file_roundtrip_and_reboot(tmp_path):
    path = tmp_path / "state" / "battery_soc.json"
    sim = Sim()
    sim.run(sim.cfg.rest_settle_s + 5, pack_for_soc(sim.cfg, 66.0))
    snap = sim.est.snapshot(wall_now=1_000.0)
    save_state(path, snap)
    assert not (path.parent / "battery_soc.json.tmp").exists()
    loaded = load_state(path)
    assert loaded == snap

    rebooted = Sim()
    assert rebooted.est.restore(loaded, wall_now=1_100.0)
    rebooted.run(5.0, pack_for_soc(rebooted.cfg, 64.0))
    assert rebooted.soc() == pytest.approx(66.0, abs=0.5)


def test_load_state_handles_missing_or_corrupt_files(tmp_path):
    assert load_state(tmp_path / "missing.json") is None
    bad = tmp_path / "bad.json"
    bad.write_text("{not json", encoding="utf-8")
    assert load_state(bad) is None
    bad.write_text("[1, 2]", encoding="utf-8")
    assert load_state(bad) is None


def test_snapshot_is_none_before_any_estimate():
    assert VoltageSocEstimator(BatteryConfig()).snapshot(0.0) is None


# ----------------------------------------------------------------------------- ROS message conventions

def test_battery_state_fields_follow_ros_conventions():
    cfg = BatteryConfig()
    sim = Sim(cfg)
    sim.run(cfg.rest_settle_s + 5, pack_for_soc(cfg, 75.0))
    fields = battery_state_fields(sim.result(), cfg)
    assert 0.74 <= fields["percentage"] <= 0.76  # 0..1, not percent
    assert math.isnan(fields["current"]) and math.isnan(fields["charge"]) and math.isnan(fields["temperature"])
    assert fields["power_supply_status"] == POWER_SUPPLY_STATUS_UNKNOWN
    assert fields["power_supply_technology"] == POWER_SUPPLY_TECHNOLOGY_LION
    assert fields["design_capacity"] == 10.0 and fields["present"] is True
    assert len(fields["cell_voltage"]) == 3 and all(math.isnan(v) for v in fields["cell_voltage"])


def test_battery_state_fields_when_unavailable_use_nan_not_zero():
    cfg = BatteryConfig()
    fields = battery_state_fields(VoltageSocEstimator(cfg).estimate(0.0), cfg)
    assert math.isnan(fields["percentage"]) and math.isnan(fields["voltage"])
    assert fields["present"] is False
