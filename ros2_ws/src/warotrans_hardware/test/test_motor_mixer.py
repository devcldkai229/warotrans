import math

from warotrans_hardware.motor_mixer import mix_to_pwm

WHEEL_SEP = 0.4535
FULL_SCALE = 0.32
PWM_MAX = 255
PWM_LIMIT = 40.0


def _mix(linear: float, angular: float):
    return mix_to_pwm(
        linear,
        angular,
        wheel_separation=WHEEL_SEP,
        full_scale_linear=FULL_SCALE,
        pwm_raw_max=PWM_MAX,
        pwm_limit_percent=PWM_LIMIT,
    )


def test_forward_both_positive():
    result = _mix(0.16, 0.0)
    assert result is not None
    assert result.left_pwm_raw > 0
    assert result.right_pwm_raw > 0
    assert result.left_pwm_percent > 0
    assert result.right_pwm_percent > 0


def test_backward_both_negative():
    result = _mix(-0.16, 0.0)
    assert result is not None
    assert result.left_pwm_raw > 0
    assert result.right_pwm_raw > 0
    assert result.left_pwm_percent < 0
    assert result.right_pwm_percent < 0


def test_left_rotation_signs():
    result = _mix(0.0, 1.0)
    assert result is not None
    assert result.left_pwm_percent < 0
    assert result.right_pwm_percent > 0


def test_right_rotation_signs():
    result = _mix(0.0, -1.0)
    assert result is not None
    assert result.left_pwm_percent > 0
    assert result.right_pwm_percent < 0


def test_25_percent_level_matches_raw_pwm():
    result = _mix(0.08, 0.0)
    assert result is not None
    assert result.left_pwm_raw == 64
    assert result.right_pwm_raw == 64


def test_40_percent_level_matches_raw_pwm():
    result = _mix(0.128, 0.0)
    assert result is not None
    assert result.left_pwm_raw == 102
    assert result.right_pwm_raw == 102


def test_speed_level_pwm_table():
    cases = {
        0.032: 26,
        0.048: 38,
        0.064: 51,
        0.080: 64,
        0.096: 77,
        0.112: 89,
        0.128: 102,
    }
    for linear, expected_raw in cases.items():
        result = _mix(linear, 0.0)
        assert result is not None
        assert result.left_pwm_raw == expected_raw


def test_proportional_normalization_on_turn():
    result = _mix(0.32, 2.9317)
    assert result is not None
    assert result.left_pwm_raw == 0
    assert result.right_pwm_raw == 102


def test_rejects_non_finite():
    assert _mix(math.nan, 0.0) is None
    assert _mix(0.0, math.inf) is None
