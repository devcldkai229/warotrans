"""Auto-localize scan matching: finds the true pose near a candidate, refuses look-alike places and bad candidates."""

import math

from warotrans_fleet.scan_match_localizer import (
    Candidate,
    GridMap,
    LikelihoodField,
    match,
    scan_points,
)

RES = 0.05


def _grid(width_m: float, height_m: float, walls: list[tuple[float, float, float, float]]) -> GridMap:
    """A free room with an outer wall plus the given wall boxes (x0, y0, x1, y1) in metres."""
    w, h = int(round(width_m / RES)), int(round(height_m / RES))
    data = [0] * (w * h)
    for j in range(h):
        for i in range(w):
            if i in (0, w - 1) or j in (0, h - 1):
                data[j * w + i] = 100
    for x0, y0, x1, y1 in walls:
        for j in range(int(y0 / RES), int(math.ceil(y1 / RES))):
            for i in range(int(x0 / RES), int(math.ceil(x1 / RES))):
                if 0 <= i < w and 0 <= j < h:
                    data[j * w + i] = 100
    return GridMap(w, h, RES, 0.0, 0.0, data)


def _scan(grid: GridMap, x: float, y: float, yaw: float, beams: int = 360, max_range: float = 8.0):
    """Ray-cast a 360 degree scan from the robot pose (laser at the base, facing forward)."""
    ranges = []
    inc = 2.0 * math.pi / beams
    for k in range(beams):
        a = yaw - math.pi + k * inc
        r = 0.0
        hit = float("inf")
        while r < max_range:
            r += RES / 4.0
            cell = grid.cell_of(x + r * math.cos(a), y + r * math.sin(a))
            if cell is None or grid.value(*cell) >= 65:
                hit = r
                break
        ranges.append(hit)
    return scan_points(ranges, -math.pi, inc, 0.05, max_range, max_beams=120)


# An L-shaped feature and a pillar make every place in this room look different.
ASYMMETRIC = _grid(4.0, 3.0, [(1.0, 0.0, 1.2, 1.2), (1.0, 1.0, 2.0, 1.2), (3.0, 2.0, 3.3, 2.3)])
# A plain rectangle: a pose and the same pose turned half a turn see the same walls.
SYMMETRIC = _grid(4.0, 2.0, [])


def test_finds_the_true_pose_near_a_candidate_whatever_the_heading():
    field = LikelihoodField(ASYMMETRIC)
    true_x, true_y, true_yaw = 2.6, 0.6, math.radians(70)
    points = _scan(ASYMMETRIC, true_x, true_y, true_yaw)

    result = match(field, points, [Candidate(2.45, 0.7, 0.0, "EP-PARKING-A", "HOME")])

    assert result.confident, result.reason
    assert result.reason == "MATCHED"
    assert math.hypot(result.best.x - true_x, result.best.y - true_y) < 0.06
    assert abs(math.atan2(math.sin(result.best.yaw - true_yaw), math.cos(result.best.yaw - true_yaw))) < math.radians(4)
    assert result.best.code == "EP-PARKING-A"


def test_picks_the_right_place_among_several_endpoints():
    field = LikelihoodField(ASYMMETRIC)
    points = _scan(ASYMMETRIC, 0.5, 2.3, math.radians(-30))
    candidates = [
        Candidate(2.6, 0.6, None, "EP-A"),
        Candidate(0.5, 2.25, None, "EP-B"),
        Candidate(3.6, 1.0, None, "EP-C"),
    ]

    result = match(field, points, candidates)

    assert result.confident, result.reason
    assert result.best.code == "EP-B"
    assert result.suggestions[0].code == "EP-B"


def test_a_look_alike_place_is_ambiguous_and_not_taken():
    field = LikelihoodField(SYMMETRIC)
    points = _scan(SYMMETRIC, 2.0, 1.0, 0.0)

    result = match(field, points, [Candidate(2.0, 1.0, 0.0, "EP-MID")])

    assert not result.confident
    assert result.reason == "AMBIGUOUS"
    assert len(result.suggestions) >= 2


def test_no_candidate_near_the_robot_is_no_match():
    field = LikelihoodField(ASYMMETRIC)
    points = _scan(ASYMMETRIC, 0.5, 2.3, 0.0)

    result = match(field, points, [Candidate(3.6, 0.5, 0.0, "EP-FAR")])

    assert not result.confident
    assert result.reason == "NO_MATCH"
    assert result.suggestions  # still says what it saw, for a person to judge


def test_no_scan_means_no_answer():
    field = LikelihoodField(ASYMMETRIC)
    result = match(field, [], [Candidate(1.0, 1.0, 0.0)])
    assert result.reason == "NO_SCAN"
    assert result.best is None


def test_a_pose_inside_a_wall_scores_nothing():
    field = LikelihoodField(ASYMMETRIC)
    points = _scan(ASYMMETRIC, 2.6, 0.6, 0.0)
    assert field.score_points(1.1, 0.5, 0.0, points) == 0.0
    assert field.score_points(2.6, 0.6, 0.0, points) > 0.9


def test_scan_points_drops_bad_ranges_and_applies_the_mount():
    pts = scan_points([1.0, float("inf"), 0.01, 2.0], 0.0, math.pi / 2, 0.05, 8.0, mount=(0.1, 0.0, math.pi / 2))
    assert len(pts) == 2
    # Beam 0 at 0 rad in the laser frame, laser turned +90 deg: ends 1 m to the left of the laser.
    assert abs(pts[0][0] - 0.1) < 1e-9 and abs(pts[0][1] - 1.0) < 1e-9


def test_the_whole_map_check_suggests_where_the_robot_really_is():
    field = LikelihoodField(ASYMMETRIC)
    points = _scan(ASYMMETRIC, 0.5, 2.3, 0.0)

    result = match(field, points, [Candidate(3.6, 0.5, 0.0, "EP-FAR")])

    assert result.best.source == "GLOBAL"
    assert math.hypot(result.best.x - 0.5, result.best.y - 2.3) < 0.08


def test_still_matches_with_lidar_noise_and_missing_beams():
    import random

    rng = random.Random(7)
    field = LikelihoodField(ASYMMETRIC)
    clean = _scan(ASYMMETRIC, 2.6, 0.6, 1.0)
    noisy = [(x + rng.gauss(0, 0.015), y + rng.gauss(0, 0.015)) for k, (x, y) in enumerate(clean) if k % 5 != 0]

    result = match(field, noisy, [Candidate(2.6, 0.6, None, "EP-PARKING-A", "HOME")])

    assert result.confident, result.reason
    assert math.hypot(result.best.x - 2.6, result.best.y - 0.6) < 0.06
