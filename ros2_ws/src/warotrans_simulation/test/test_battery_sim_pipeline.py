"""battery_sim_node model -> real voltage-only estimator (no Gazebo; needs the built workspace sourced)."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

pytest.importorskip("rclpy")
pytest.importorskip("warotrans_msgs.msg")

SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))
from battery_sim_node import BatterySimModel  # noqa: E402
from warotrans_hardware.battery_soc import BatteryConfig, VoltageSocEstimator  # noqa: E402

DT = 0.2


def _model(**kw):
    args = dict(initial_soc_pct=85.0, capacity_ah=10.0, cells_in_series=3, idle_current_a=1.2,
                current_a_per_mps=8.0, current_a_per_radps=3.0, internal_resistance_ohm=0.06,
                divider_gain=5.0, noise_mv=2.0, seed=7)
    args.update(kw)
    return BatterySimModel(**args)


def _run(model, est, t, seconds, lin=0.0, ang=0.0):
    for _ in range(int(seconds / DT)):
        t += DT
        est.update_motion(t, lin, ang)
        est.add_sample(t, model.step(DT, lin, ang))
    return t


def test_estimator_tracks_the_simulated_pack_after_driving_and_resting():
    # Sag coefficients as V6 would measure them on this model: I * R.
    cfg = BatteryConfig(idle_sag_v=1.2 * 0.06, sag_v_per_mps=8.0 * 0.06, sag_v_per_radps=3.0 * 0.06)
    model, est = _model(), VoltageSocEstimator(cfg)
    t = _run(model, est, 0.0, cfg.rest_settle_s + 10)
    assert est.estimate(t).percentage * 100 == pytest.approx(model.soc_pct, abs=1.5)

    t = _run(model, est, t, 15 * 60.0, lin=0.3, ang=0.2)  # a long delivery run
    during = est.estimate(t)
    assert during.confidence == "LOW" and during.percentage * 100 <= 85.0
    assert during.percentage * 100 == pytest.approx(model.soc_pct, abs=2.0)  # compensated under load too
    t = _run(model, est, t, cfg.rest_settle_s + 15 * 60.0)  # park and rest
    rested = est.estimate(t)
    assert rested.confidence == "MEDIUM"
    assert rested.percentage * 100 == pytest.approx(model.soc_pct, abs=cfg.rest_hysteresis_pct + 1.5)
    assert rested.charging_state == "UNKNOWN"


def test_model_is_deterministic_for_a_seed():
    a, b = _model(), _model()
    assert [a.step(DT, 0.2, 0.0) for _ in range(50)] == [b.step(DT, 0.2, 0.0) for _ in range(50)]
