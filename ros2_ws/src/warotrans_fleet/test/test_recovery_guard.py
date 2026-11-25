"""Recovery guard decisions: a manoeuvre starts only with fresh data and room, using the monitor's padded footprint."""

from __future__ import annotations

import math

import pytest

from warotrans_fleet.recovery_guard import (
    GuardConfig,
    backup_blocking_points,
    clearance_block_reason,
    count_in_footprint,
    spin_blocking_points,
)

CFG = GuardConfig()
HL, HW = CFG.half_length + CFG.padding, CFG.half_width + CFG.padding  # 0.18, 0.15


def wall_x(x: float, n: int = 20) -> list[tuple[float, float]]:
    """A wall across the robot at body-frame x."""
    return [(x, -0.5 + i / n) for i in range(n + 1)]


def wall_y(y: float, n: int = 400) -> list[tuple[float, float]]:
    """A wall along the robot at body-frame y."""
    return [(-1.0 + 2.0 * i / n, y) for i in range(n + 1)]


def decide(kind, amount, points, age=0.1, pose=True):
    return clearance_block_reason(CFG, kind, amount, points, age, pose)


@pytest.mark.parametrize(
    "points, age, pose, expected",
    [
        (None, 0.1, True, "NO_SCAN"),
        ([], None, True, "NO_SCAN"),
        ([], float("nan"), True, "NO_SCAN"),
        ([], 0.6, True, "SCAN_STALE"),
        ([], -0.1, True, "SCAN_STALE"),
        ([], 0.1, False, "POSE_UNKNOWN"),
    ],
)
@pytest.mark.parametrize("kind, amount", [("backup", 0.10), ("spin", 0.785)])
def test_missing_or_old_data_refuses(kind, amount, points, age, pose, expected):
    assert decide(kind, amount, points, age, pose) == expected


def test_unknown_kind_and_bad_amount_refuse():
    assert decide("drive", 0.1, []) == "INVALID_REQUEST"
    assert decide("spin", float("inf"), []) == "INVALID_REQUEST"


def test_padding_is_part_of_the_footprint():
    assert CFG.padding == 0.02
    # 1.5 cm from the body: inside the padded polygon though the bare chassis would be clear.
    assert count_in_footprint(CFG, wall_x(CFG.half_length + 0.015)) >= CFG.min_points
    assert decide("spin", 0.785, wall_x(CFG.half_length + 0.015)) == "OBSTACLE_IN_FOOTPRINT"


def test_reversing_away_from_a_wall_inside_the_padding_may_start_the_monitor_still_decides():
    assert decide("backup", 0.10, wall_x(CFG.half_length + 0.015)) is None


def test_reversing_with_a_wall_along_the_side_inside_the_padding_is_refused():
    assert decide("backup", 0.10, wall_y(CFG.half_width + 0.015)) == "REAR_NOT_CLEAR"


def test_reversing_into_something_inside_the_rear_padding_is_refused():
    assert decide("backup", 0.10, wall_x(-(CFG.half_length + 0.015))) == "REAR_NOT_CLEAR"


def test_backup_with_wall_ahead_only_is_allowed():
    assert decide("backup", 0.10, wall_x(CFG.half_length + 0.05)) is None


def test_backup_refused_when_the_rear_sweep_hits_something():
    behind = wall_x(-(CFG.half_length + CFG.padding + 0.05))
    assert backup_blocking_points(CFG, behind, 0.10) >= CFG.min_points
    assert decide("backup", 0.10, behind) == "REAR_NOT_CLEAR"


def test_backup_refused_within_the_margin_past_its_end():
    just_past = wall_x(-(HL + 0.10 + CFG.backup_margin_m - 0.005))
    assert decide("backup", 0.10, just_past) == "REAR_NOT_CLEAR"
    assert decide("backup", 0.10, wall_x(-(HL + 0.10 + CFG.backup_margin_m + 0.02))) is None


@pytest.mark.parametrize("distance", [0.0, -0.05, 0.13, 0.5])
def test_backup_distance_beyond_the_authorized_one_refused(distance):
    assert decide("backup", distance, []) == "BACKUP_DISTANCE_NOT_AUTHORIZED"


def test_few_points_are_noise_like_for_the_monitor():
    three = [(-HL - 0.05, 0.0), (-HL - 0.05, 0.01), (-HL - 0.05, -0.01)]
    assert decide("backup", 0.10, three) is None
    assert decide("backup", 0.10, three + [(-HL - 0.05, 0.02)]) == "REAR_NOT_CLEAR"


def test_spin_refused_when_a_corner_would_sweep_a_wall_ahead():
    # Nose 2.5 cm past the padded polygon, inside the sweep radius (0.234 m).
    wall = wall_x(HL + 0.025)
    assert count_in_footprint(CFG, wall) == 0
    assert spin_blocking_points(CFG, wall, math.pi / 2) >= CFG.min_points
    assert decide("spin", math.pi / 2, wall) == "SPIN_NOT_CLEAR"


def test_spin_allowed_when_outside_the_sweep_radius():
    reach = math.hypot(HL, HW)
    assert decide("spin", math.pi, wall_x(reach + 0.01)) is None
    assert decide("spin", -math.pi, wall_y(reach + 0.01)) is None


def test_spin_refused_with_a_wall_along_the_side():
    assert decide("spin", 0.785, wall_y(HW + 0.03)) == "SPIN_NOT_CLEAR"


def test_only_the_direction_with_room_is_allowed():
    # A post just ahead of the nose, left of centre: a small turn right swings the nose into it, a small turn left
    # swings the nose away. The recovery tree tries both, so the guard must tell them apart.
    post = [(HL + 0.01, 0.05 + i * 0.002) for i in range(5)]
    assert spin_blocking_points(CFG, post, 0.5) == 0
    assert spin_blocking_points(CFG, post, -0.5) >= CFG.min_points
    assert decide("spin", 0.5, post) is None
    assert decide("spin", -0.5, post) == "SPIN_NOT_CLEAR"


def test_a_chassis_ahead_of_the_turning_centre_is_checked_where_it_is():
    """RBT-001: chassis centre 0.05 m ahead of base_footprint. A wall 0.18 m behind the turning centre is clear of the
    rear (0.11 + 0.02 padding) and of a 0.10 m backup with its margin is not; centred box (0.18) refused even a spin."""
    from warotrans_fleet.recovery_guard import GuardConfig, backup_blocking_points, count_in_footprint
    offset = GuardConfig(half_length=0.16, half_width=0.13, center_x=0.05)
    centred = GuardConfig(half_length=0.16, half_width=0.13)
    wall_behind = [(-0.16, y / 100.0) for y in range(-10, 11, 2)]
    assert count_in_footprint(centred, wall_behind) > 0  # the old box put the wall inside the body
    assert count_in_footprint(offset, wall_behind) == 0  # it is behind the real rear (-0.13)
    front_obstacle = [(0.22, y / 100.0) for y in range(-10, 11, 2)]
    assert count_in_footprint(offset, front_obstacle) > 0  # the real front reaches 0.23
    assert backup_blocking_points(offset, [(-0.24, 0.0)] * 5, 0.10) == 5  # within rear 0.13 + 0.10 + margin 0.03
    # A wall 0.28 m behind the turning centre: the old centred box (rear 0.18) refused a 0.10 m backup, the real body does not.
    assert backup_blocking_points(centred, [(-0.28, 0.0)] * 5, 0.10) == 5
    assert backup_blocking_points(offset, [(-0.28, 0.0)] * 5, 0.10) == 0
