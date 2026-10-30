"""FollowPath path geometry: dense, de-duplicated, heading along the lane."""

import math

from warotrans_fleet.nav2_segment_executor import densify_path


def test_shared_vertices_emitted_once_and_spacing_bounded():
    poses = densify_path([[(2.959, 0.357), (2.959, 0.09)], [(2.959, 0.09), (2.44, -0.185)]], step=0.05)
    points = [(x, y) for x, y, _ in poses]
    assert all(math.dist(a, b) > 1e-6 for a, b in zip(points, points[1:]))
    assert all(math.dist(a, b) <= 0.05 + 1e-9 for a, b in zip(points, points[1:]))
    assert points[0] == (2.959, 0.357) and points[-1] == (2.44, -0.185)


def test_heading_follows_each_sub_segment_and_holds_at_end():
    poses = densify_path([[(2.44, -0.185), (1.921, -0.185)], [(1.921, -0.185), (1.237, -0.192)]])
    assert math.isclose(poses[0][2], math.pi, abs_tol=1e-9)
    # Last pose keeps the final lane heading (west) instead of a default yaw of 0.
    assert abs(abs(poses[-1][2]) - math.pi) < 0.02


def test_degenerate_inputs():
    assert densify_path([]) == []
    assert densify_path([[(1.0, 2.0)]]) == [(1.0, 2.0, 0.0)]
