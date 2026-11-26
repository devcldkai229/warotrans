"""The other robots as costmap / collision-monitor points (fleet_obstacles.py)."""

import math

from warotrans_fleet.fleet_obstacles import (
    MAX_POSE_AGE_SEC,
    FleetObstaclesState,
    fleet_points,
    footprint_points,
    parse_fleet_poses,
    pointcloud2_fields_xyz,
)

MSG = {
    "schemaVersion": 1,
    "robots": [
        {"robotCode": "RBT-001", "x": 0.0, "y": 0.0, "yaw": 0.0, "lengthM": 0.32, "widthM": 0.26, "localized": True, "poseAgeMs": 100},
        {"robotCode": "RBT-002", "x": 1.0, "y": 0.5, "yaw": math.pi / 2, "lengthM": 0.30, "widthM": 0.245, "localized": True, "poseAgeMs": 200},
        {"robotCode": "RBT-003", "x": 2.0, "y": 0.0, "yaw": 0.0, "localized": True, "poseAgeMs": 5000},
    ],
}


def test_only_the_other_fresh_robots_are_drawn():
    others = parse_fleet_poses(MSG, "RBT-001")
    assert [r.code for r in others] == ["RBT-002"]  # not itself, not a pose 5 s old


def test_points_fill_the_body_with_its_margin_at_its_heading():
    rbt2 = parse_fleet_poses(MSG, "RBT-001")[0]
    pts = footprint_points(rbt2, margin=0.05)
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    # Turned 90 deg: its length runs along y.
    assert math.isclose(max(ys) - min(ys), 0.30 + 0.10, abs_tol=1e-6)
    assert math.isclose(max(xs) - min(xs), 0.245 + 0.10, abs_tol=1e-6)
    assert all(abs(x - 1.0) <= 0.1726 and abs(y - 0.5) <= 0.2001 for x, y in pts)


def test_a_robot_that_lost_its_localization_is_a_disc_at_its_last_pose():
    body = {"robots": [{"robotCode": "RBT-002", "x": 0.0, "y": 0.0, "yaw": 0.3, "lengthM": 0.32, "widthM": 0.26,
                        "localized": False, "poseAgeMs": 0}]}
    pts = footprint_points(parse_fleet_poses(body, "RBT-001")[0], margin=0.05)
    radius = max(math.hypot(x, y) for x, y in pts)
    assert radius > math.hypot(0.16, 0.13) + 0.05  # any heading, larger margin


def test_points_age_out_when_no_new_message_arrives():
    t = [0.0]
    state = FleetObstaclesState("RBT-001", lambda: t[0])
    state.on_message(MSG)
    assert [r.code for r in state.current()] == ["RBT-002"]
    t[0] += MAX_POSE_AGE_SEC  # the pose was already 0.2 s old
    assert state.current() == []


def test_pointcloud_body_is_float32_xyz():
    pts = fleet_points(parse_fleet_poses(MSG, "RBT-001"))
    data, width = pointcloud2_fields_xyz(pts)
    assert width == len(pts) and len(data) == 12 * width
    assert all(p[2] == 0.10 for p in pts)


def _other(x: float, y: float) -> "OtherRobot":
    from warotrans_fleet.fleet_obstacles import OtherRobot

    return OtherRobot(code="RBT-X", x=x, y=y, yaw=0.0, length=0.32, width=0.26, localized=True, pose_age_sec=0.0)


def test_robot_ahead_only_counts_close_robots_in_front():
    from warotrans_fleet.fleet_obstacles import robot_ahead

    me = (0.0, 0.0, 0.0)  # facing +x
    assert robot_ahead([_other(0.6, 0.0)], me)
    assert robot_ahead([_other(0.3, 0.5)], me)  # ~59 deg off the nose
    assert not robot_ahead([_other(-0.6, 0.0)], me)  # behind
    assert not robot_ahead([_other(0.0, 0.6)], me)  # beside (90 deg)
    assert not robot_ahead([_other(0.9, 0.0)], me)  # too far
    assert not robot_ahead([_other(0.6, 0.0)], None)  # no own pose
    assert robot_ahead([_other(0.0, -0.6)], (0.0, 0.0, -1.5708))  # facing -y
