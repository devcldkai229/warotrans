"""AmclReadiness: the boot pose (0, 0, 0) with fresh TF must not count as localized on the real robot."""

import math

from warotrans_fleet.amcl_readiness import AmclReadiness


def _cov(var_xy: float, var_yaw: float) -> list[float]:
    cov = [0.0] * 36
    cov[0] = cov[7] = var_xy
    cov[35] = var_yaw
    return cov


def test_boot_pose_without_operator_is_not_ready():
    r = AmclReadiness()
    r.on_amcl_pose(_cov(0.01, 0.01))     # amcl publishes its provisional set_initial_pose
    assert r.blocking_reason() == "NO_OPERATOR_INITIAL_POSE"


def test_operator_pose_without_amcl_pose_is_not_ready():
    r = AmclReadiness()
    r.on_initial_pose()
    assert r.blocking_reason() == "NO_AMCL_POSE"


def test_operator_pose_and_tight_covariance_is_ready():
    r = AmclReadiness(max_xy_std_m=0.5, max_yaw_std_rad=0.5)
    r.on_initial_pose()
    r.on_amcl_pose(_cov(0.25, 0.0685))   # RViz "2D Pose Estimate" default covariance
    assert r.blocking_reason() is None


def test_diverged_filter_blocks_again():
    r = AmclReadiness(max_xy_std_m=0.5, max_yaw_std_rad=0.5)
    r.on_initial_pose()
    r.on_amcl_pose(_cov(0.04, 0.01))
    assert r.blocking_reason() is None
    r.on_amcl_pose(_cov(1.0, 0.01))       # σxy 1.0 m
    assert r.blocking_reason().startswith("AMCL_COVARIANCE_HIGH")


def test_non_finite_covariance_blocks():
    r = AmclReadiness()
    r.on_initial_pose()
    r.on_amcl_pose(_cov(math.nan, 0.01))
    assert r.blocking_reason().startswith("AMCL_COVARIANCE_HIGH")
