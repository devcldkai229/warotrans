"""Auto-localize: find where the robot stands by matching its current laser scan against the map.

The backend sends candidate poses (the robot's last pose on this map, its home endpoint, every endpoint of the map).
Around each candidate the scan is scored against a likelihood field of the map (distance to the nearest wall) over a
small window in x / y and a full turn in yaw, then the best poses are refined. The robot takes a pose only when it is
sure: the best score is high enough AND clearly better than the best pose somewhere else (a warehouse has places that
look alike). Otherwise it answers with the best guesses for a person to confirm.

Pure Python (no numpy, no ROS) so it runs on the robot and in unit tests alike. Maps here are small (a few hundred
cells a side), so a multi-source BFS for the field and a few hundred thousand look-ups per run are fast enough.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence

OCCUPIED_THRESHOLD = 65  # OccupancyGrid value (0..100) from which a cell is a wall
_ALONG_BEAM = (0.25, 0.5, 0.75)  # where along each beam it must not be inside a wall
_CROSS_PENALTY = 2.0  # each beam through a wall costs twice what a beam ending on one earns


@dataclass(frozen=True)
class GridMap:
    """nav_msgs/OccupancyGrid: row-major from the origin corner, row 0 at the origin (bottom), -1 unknown."""

    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]

    def cell_of(self, x: float, y: float) -> Optional[tuple[int, int]]:
        i = int(math.floor((x - self.origin_x) / self.resolution))
        j = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= i < self.width and 0 <= j < self.height:
            return i, j
        return None

    def value(self, i: int, j: int) -> int:
        return int(self.data[j * self.width + i])


class LikelihoodField:
    """Distance (metres) from every cell to the nearest wall, capped at ``max_distance``."""

    def __init__(self, grid: GridMap, max_distance: float = 0.5, sigma: float = 0.08) -> None:
        self.grid = grid
        self.sigma = sigma
        self.max_distance = max_distance
        w, h = grid.width, grid.height
        cap = max(1, int(math.ceil(max_distance / grid.resolution)))
        inf = float("inf")
        dist = [inf] * (w * h)
        nearest: list[int] = [-1] * (w * h)
        queue: deque[int] = deque()
        for idx in range(w * h):
            if grid.data[idx] >= OCCUPIED_THRESHOLD:
                dist[idx] = 0.0
                nearest[idx] = idx
                queue.append(idx)
        # Brushfire carrying each cell's nearest wall cell: close to the Euclidean distance transform.
        while queue:
            idx = queue.popleft()
            ci, cj = idx % w, idx // w
            seed = nearest[idx]
            si, sj = seed % w, seed // w
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    if di == 0 and dj == 0:
                        continue
                    ni, nj = ci + di, cj + dj
                    if not (0 <= ni < w and 0 <= nj < h):
                        continue
                    if abs(ni - si) > cap or abs(nj - sj) > cap:
                        continue
                    nidx = nj * w + ni
                    d = math.hypot(ni - si, nj - sj)
                    if d < dist[nidx]:
                        dist[nidx] = d
                        nearest[nidx] = seed
                        queue.append(nidx)
        res = grid.resolution
        self._dist = [min(d * res, max_distance) if d != inf else max_distance for d in dist]
        two_s2 = 2.0 * sigma * sigma
        # Score of a beam end in each cell; unknown cells score nothing (a scan should end on the known map).
        self._score = [
            0.0 if grid.data[k] < 0 else math.exp(-(self._dist[k] ** 2) / two_s2) for k in range(w * h)
        ]

    def distance_at(self, x: float, y: float) -> float:
        cell = self.grid.cell_of(x, y)
        if cell is None:
            return self.max_distance
        return self._dist[cell[1] * self.grid.width + cell[0]]

    def score_points(self, x: float, y: float, yaw: float, points: Sequence[tuple[float, float]]) -> float:
        """How well the scan fits the map at the robot pose (x, y, yaw), 0..1; 0 if the robot sits in a wall.

        Two pieces of evidence: each beam should END near a wall (mean likelihood of the beam ends), and no beam may
        PASS THROUGH a wall on its way (checked at a few points along it). The second one is what tells a corner from
        the corner opposite it: the ends can fit both, but at the wrong one some beams cross walls.
        """
        grid = self.grid
        own = grid.cell_of(x, y)
        if own is None or grid.value(*own) >= OCCUPIED_THRESHOLD or grid.value(*own) < 0:
            return 0.0
        if not points:
            return 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        ox, oy, res, w, hgt = grid.origin_x, grid.origin_y, grid.resolution, grid.width, grid.height
        score = self._score
        dist = self._dist
        clear = 1.5 * res  # a point along the beam this close to a wall may just be the wall the beam ends on
        total = 0.0
        crossed = 0
        for bx, by in points:
            ex, ey = c * bx - s * by, s * bx + c * by
            i = int((x + ex - ox) // res)
            j = int((y + ey - oy) // res)
            if 0 <= i < w and 0 <= j < hgt:
                total += score[j * w + i]
            for f in _ALONG_BEAM:
                ii = int((x + f * ex - ox) // res)
                jj = int((y + f * ey - oy) // res)
                if 0 <= ii < w and 0 <= jj < hgt:
                    if dist[jj * w + ii] < clear and math.hypot(ex, ey) * (1.0 - f) > 2.0 * clear:
                        crossed += 1
                        break
                else:
                    crossed += 1  # a beam leaving the map on its way cannot be this pose
                    break
        n = len(points)
        return max(0.0, total / n - _CROSS_PENALTY * crossed / n)


def scan_points(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    mount: tuple[float, float, float] = (0.0, 0.0, 0.0),
    max_beams: int = 90,
) -> list[tuple[float, float]]:
    """Valid beam ends in the robot base frame, thinned evenly to at most ``max_beams``."""
    mx, my, myaw = mount
    cm, sm = math.cos(myaw), math.sin(myaw)
    valid: list[tuple[float, float]] = []
    for k, r in enumerate(ranges):
        if not (math.isfinite(r) and range_min < r < range_max):
            continue
        a = angle_min + k * angle_increment
        lx, ly = r * math.cos(a), r * math.sin(a)
        valid.append((mx + cm * lx - sm * ly, my + sm * lx + cm * ly))
    if len(valid) <= max_beams:
        return valid
    step = len(valid) / float(max_beams)
    return [valid[int(n * step)] for n in range(max_beams)]


@dataclass(frozen=True)
class Candidate:
    x: float
    y: float
    yaw: Optional[float] = None
    code: Optional[str] = None
    source: str = "ENDPOINT"  # LAST_POSE / HOME / ENDPOINT


@dataclass(frozen=True)
class Hypothesis:
    x: float
    y: float
    yaw: float
    score: float
    code: Optional[str]
    source: str

    def to_dict(self) -> dict:
        return {
            "x": round(self.x, 3),
            "y": round(self.y, 3),
            "yaw": round(self.yaw, 4),
            "score": round(self.score, 3),
            "endpointCode": self.code,
            "source": self.source,
        }


@dataclass
class MatchSettings:
    window_m: float = 0.3  # search ± this around each candidate
    coarse_step_m: float = 0.1
    coarse_yaw_step_rad: float = math.radians(10.0)
    fine_step_m: float = 0.02
    fine_yaw_step_rad: float = math.radians(1.5)
    coarse_beams: int = 40
    min_score: float = 0.55  # best pose must score at least this
    min_margin: float = 0.12  # and beat the best pose elsewhere by this much
    same_place_m: float = 0.25  # two hypotheses closer than this (and yaw) are the same place
    same_place_yaw_rad: float = math.radians(25.0)
    other_yaw_rad: float = math.radians(45.0)  # a second heading at one place counts as another hypothesis
    # Whole-map check: the pose is only taken when no place away from the candidates fits about as well.
    global_search: bool = True
    global_step_m: float = 0.15
    global_yaw_step_rad: float = math.radians(15.0)
    global_beams: int = 30
    global_max_positions: int = 1500  # larger maps are searched on a coarser grid
    global_refine: int = 4
    robot_radius_m: float = 0.12


@dataclass
class MatchResult:
    confident: bool
    best: Optional[Hypothesis]
    runner_up: Optional[Hypothesis]
    reason: str  # MATCHED / NO_SCAN / NO_MATCH / AMBIGUOUS
    suggestions: list[Hypothesis] = field(default_factory=list)

    @property
    def margin(self) -> Optional[float]:
        if self.best is None:
            return None
        return self.best.score - (self.runner_up.score if self.runner_up else 0.0)


def _angle_diff(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def _thin(points: Sequence[tuple[float, float]], n: int) -> list[tuple[float, float]]:
    if len(points) <= n:
        return list(points)
    step = len(points) / float(n)
    return [points[int(k * step)] for k in range(n)]


def _frange(lo: float, hi: float, step: float) -> list[float]:
    count = int(round((hi - lo) / step))
    return [lo + k * step for k in range(count + 1)]


def _refine(
    field_: LikelihoodField, points: Sequence[tuple[float, float]], x: float, y: float, yaw: float, s: MatchSettings
) -> tuple[float, float, float, float]:
    """Hill-climb from (x, y, yaw) with shrinking steps; returns the best pose and its score."""
    best = (x, y, yaw, field_.score_points(x, y, yaw, points))
    step_xy, step_yaw = s.coarse_step_m / 2.0, s.coarse_yaw_step_rad / 2.0
    while step_xy >= s.fine_step_m - 1e-9 or step_yaw >= s.fine_yaw_step_rad - 1e-9:
        improved = True
        while improved:
            improved = False
            bx, by, byaw, bscore = best
            for dx, dy, dyaw in (
                (step_xy, 0, 0), (-step_xy, 0, 0), (0, step_xy, 0), (0, -step_xy, 0), (0, 0, step_yaw), (0, 0, -step_yaw),
            ):
                cand = field_.score_points(bx + dx, by + dy, byaw + dyaw, points)
                if cand > bscore + 1e-9:
                    best = (bx + dx, by + dy, byaw + dyaw, cand)
                    bscore = cand
                    improved = True
        step_xy /= 2.0
        step_yaw /= 2.0
    x, y, yaw, score = best
    return x, y, math.atan2(math.sin(yaw), math.cos(yaw)), score


def _hypotheses_near(
    field_: LikelihoodField, points: Sequence[tuple[float, float]], candidate: Candidate, s: MatchSettings
) -> list[Hypothesis]:
    """Coarse search (full turn of yaw) around one candidate, then refine its best pose and its best other heading."""
    coarse_pts = _thin(points, s.coarse_beams)
    offsets = _frange(-s.window_m, s.window_m, s.coarse_step_m)
    yaws = _frange(-math.pi, math.pi - s.coarse_yaw_step_rad, s.coarse_yaw_step_rad)
    scored: list[tuple[float, float, float, float]] = []
    for dx in offsets:
        for dy in offsets:
            px, py = candidate.x + dx, candidate.y + dy
            for yaw in yaws:
                scored.append((field_.score_points(px, py, yaw, coarse_pts), px, py, yaw))
    if not scored:
        return []
    scored.sort(reverse=True)
    picks = [scored[0]]
    for entry in scored[1:]:
        if _angle_diff(entry[3], picks[0][3]) >= s.other_yaw_rad:
            picks.append(entry)
            break
    out: list[Hypothesis] = []
    for _, px, py, pyaw in picks:
        rx, ry, ryaw, rscore = _refine(field_, points, px, py, pyaw, s)
        out.append(Hypothesis(rx, ry, ryaw, rscore, candidate.code, candidate.source))
    return out


def _same_place(a: Hypothesis, b: Hypothesis, s: MatchSettings) -> bool:
    return math.hypot(a.x - b.x, a.y - b.y) < s.same_place_m and _angle_diff(a.yaw, b.yaw) < s.same_place_yaw_rad


GLOBAL = "GLOBAL"


def _named_after_nearest(h: Hypothesis, candidates: Sequence[Candidate], s: MatchSettings) -> Hypothesis:
    nearest = min(candidates, key=lambda c: math.hypot(c.x - h.x, c.y - h.y))
    if math.hypot(nearest.x - h.x, nearest.y - h.y) > s.window_m + s.coarse_step_m:
        return Hypothesis(h.x, h.y, h.yaw, h.score, None, h.source)
    return Hypothesis(h.x, h.y, h.yaw, h.score, nearest.code, nearest.source)


def _global_hypotheses(
    field_: LikelihoodField, points: Sequence[tuple[float, float]], s: MatchSettings
) -> list[Hypothesis]:
    """Coarse search of the whole free map: where else could this scan fit? Keeps the best few distinct places."""
    grid = field_.grid
    free = [
        (i, j)
        for j in range(grid.height)
        for i in range(grid.width)
        if 0 <= grid.value(i, j) < OCCUPIED_THRESHOLD
        and field_._dist[j * grid.width + i] >= s.robot_radius_m  # the robot body fits there
    ]
    if not free:
        return []
    stride = max(1, int(round(s.global_step_m / grid.resolution)))
    cells = [(i, j) for i, j in free if i % stride == 0 and j % stride == 0]
    while len(cells) > s.global_max_positions:
        stride += 1
        cells = [(i, j) for i, j in free if i % stride == 0 and j % stride == 0]
    pts = _thin(points, s.global_beams)
    yaws = _frange(-math.pi, math.pi - s.global_yaw_step_rad, s.global_yaw_step_rad)
    scored: list[tuple[float, float, float, float]] = []
    for i, j in cells:
        px = grid.origin_x + (i + 0.5) * grid.resolution
        py = grid.origin_y + (j + 0.5) * grid.resolution
        for yaw in yaws:
            scored.append((field_.score_points(px, py, yaw, pts), px, py, yaw))
    scored.sort(reverse=True)
    out: list[Hypothesis] = []
    for _, px, py, pyaw in scored[: s.global_refine * 8]:
        rx, ry, ryaw, rscore = _refine(field_, points, px, py, pyaw, s)
        h = Hypothesis(rx, ry, ryaw, rscore, None, GLOBAL)
        if not any(_same_place(h, o, s) for o in out):
            out.append(h)
        if len(out) >= s.global_refine:
            break
    return out


def match(
    field_: LikelihoodField,
    points: Sequence[tuple[float, float]],
    candidates: Iterable[Candidate],
    settings: Optional[MatchSettings] = None,
    max_suggestions: int = 3,
) -> MatchResult:
    """Best pose near the candidates, taken only when nothing anywhere on the map fits about as well.

    MATCHED: confident. AMBIGUOUS: another place fits nearly as well. NO_MATCH: nothing near a candidate fits well,
    or the best fit on the map is away from every candidate (the robot is not where the backend thinks it could be).
    """
    s = settings or MatchSettings()
    if len(points) < 10:
        return MatchResult(False, None, None, "NO_SCAN")
    candidates = list(candidates)
    near: list[Hypothesis] = []
    for candidate in candidates:
        near.extend(_hypotheses_near(field_, points, candidate, s))
    # The search window of one candidate can reach the next one: name a pose after the candidate it is closest to.
    near = [_named_after_nearest(h, candidates, s) for h in near]
    elsewhere = _global_hypotheses(field_, points, s) if s.global_search else []
    # A place found both ways keeps the candidate's name.
    found = near + [g for g in elsewhere if not any(_same_place(g, h, s) for h in near)]
    found.sort(key=lambda h: h.score, reverse=True)
    distinct: list[Hypothesis] = []
    for h in found:
        if not any(_same_place(h, d, s) for d in distinct):
            distinct.append(h)
    if not distinct:
        return MatchResult(False, None, None, "NO_MATCH")
    best = distinct[0]
    runner_up = distinct[1] if len(distinct) > 1 else None
    suggestions = distinct[:max_suggestions]
    if best.score < s.min_score or best.source == GLOBAL:
        return MatchResult(False, best, runner_up, "NO_MATCH", suggestions)
    if runner_up is not None and best.score - runner_up.score < s.min_margin:
        return MatchResult(False, best, runner_up, "AMBIGUOUS", suggestions)
    return MatchResult(True, best, runner_up, "MATCHED", suggestions)
