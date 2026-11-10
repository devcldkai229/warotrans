"""nav_stuck_probe analysis (no ROS needed)."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from nav_stuck_probe import lateral_error, points_in_padded_footprint, scale_of, summarize, swallowing_layer  # noqa: E402

GO = (0.1, 0.0)
STOP = (0.0, 0.0)


def test_each_layer_is_named_when_it_zeroes_the_command():
    assert swallowing_layer({"cmd_vel_nav": GO, "cmd_vel_smoothed": STOP}) == "velocity_smoother"
    assert swallowing_layer({"cmd_vel_nav": GO, "cmd_vel_smoothed": GO, "cmd_vel_safe": STOP}) == "collision_monitor"
    assert swallowing_layer({"cmd_vel_nav": GO, "cmd_vel_smoothed": GO, "cmd_vel_safe": GO, "cmd_vel_motor": STOP}) == "authority_gate"


def test_a_silent_monitor_is_blocking_not_passing():
    # It stops publishing after stop_pub_timeout: seen in the rig as cmd_vel_safe missing while the smoother commands.
    assert swallowing_layer({"cmd_vel_nav": GO, "cmd_vel_smoothed": GO}) == "collision_monitor"


def test_a_layer_this_stack_does_not_have_is_skipped():
    known = ("cmd_vel_nav", "cmd_vel_smoothed")
    assert swallowing_layer({"cmd_vel_nav": GO, "cmd_vel_smoothed": GO}, known) is None


def test_nothing_commanded_or_all_through_is_no_block():
    assert swallowing_layer({}) is None
    assert swallowing_layer({"cmd_vel_nav": STOP, "cmd_vel_smoothed": STOP}) is None
    assert swallowing_layer({"cmd_vel_nav": GO, "cmd_vel_smoothed": GO, "cmd_vel_safe": GO, "cmd_vel_motor": GO}) is None


def test_scale_reports_how_much_survived():
    assert scale_of((0.0, 0.5), (0.0, 0.1)) == 0.2
    assert scale_of((0.0, 0.0), (0.0, 0.0)) is None


def test_padded_footprint_count_includes_the_padding_only():
    assert points_in_padded_footprint([(0.17, 0.0), (0.19, 0.0), (0.0, 0.14), (0.0, 0.16)]) == 2


def test_lateral_error_to_the_nearest_segment():
    path = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)]
    assert abs(lateral_error(path, 0.5, 0.2) - 0.2) < 1e-9
    assert abs(lateral_error(path, 1.3, 0.5) - 0.3) < 1e-9
    assert lateral_error([(0.0, 0.0)], 0.0, 0.0) is None


def test_summary_reports_blocked_time_and_pose_jumps():
    samples = [
        {"t": "s", "layer": "collision_monitor", "map_pose": [0.0, 0.0, 0.0], "lat_err": 0.01},
        {"t": "s", "layer": "collision_monitor", "map_pose": [0.0, 0.0, 0.0], "lat_err": 0.02},
        {"t": "s", "layer": None, "map_pose": [0.12, 0.0, 0.0], "lat_err": 0.03},
        {"t": "action", "action": "backup", "status": "SUCCEEDED"},
        {"t": "guard", "kind": "spin", "reason": "SPIN_NOT_CLEAR"},
    ]
    out = summarize(samples)
    assert out["blocked_s"] == {"collision_monitor": 0.2}
    assert out["pose_jumps_over_5cm"] == [0.12]
    assert out["actions"] == {"backup:SUCCEEDED": 1}
    assert out["guard"] == {"spin:SPIN_NOT_CLEAR": 1}
