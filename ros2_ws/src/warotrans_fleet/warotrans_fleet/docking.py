"""Docking geometry: reach an endpoint heading in a bay too small to turn in (2026-11-11 lab).

The robot is a skid-steer chassis: it can drive straight and turn in place, nothing else. In the small bays of MAP-ONE
the footprint cannot turn at the endpoint (the tail reaches the wall), so "drive there, then turn to the endpoint
heading" fails. The robot has to turn where there is room, on the bay axis, then drive straight in, forwards or
backwards. Leaving such a bay is the mirror image: straight out first, then turn.

Pure Python (no ROS): the static occupancy map of the bundle, the padded footprint around base_footprint.
"""

from __future__ import annotations

import math
import os
import re
from dataclasses import dataclass
from typing import Optional, Sequence

Pose = tuple[float, float, float]
Point = tuple[float, float]


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class OccupancyMap:
    """Blocked cells of a map_server map (trinary PGM): occupied, and unknown."""

    def __init__(self, width: int, height: int, resolution: float, origin: Point, blocked: Sequence[bool]) -> None:
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin = origin
        self._blocked = blocked  # row 0 = lowest y

    @classmethod
    def load(cls, yaml_path: str) -> "OccupancyMap":
        text = open(yaml_path, encoding="utf-8").read()
        resolution = float(re.search(r"resolution:\s*([-\d.eE]+)", text).group(1))
        origin_block = text.split("origin:", 1)[1]
        ox, oy = (float(v) for v in re.findall(r"-?\d+(?:\.\d+)?(?:[eE]-?\d+)?", origin_block)[:2])
        image = re.search(r"image:\s*(\S+)", text).group(1)
        neg = re.search(r"negate:\s*(\d)", text)
        negate = int(neg.group(1)) if neg else 0
        fr = re.search(r"free_thresh:\s*([\d.]+)", text)
        free = float(fr.group(1)) if fr else 0.196
        raw = open(os.path.join(os.path.dirname(yaml_path), image), "rb").read()
        m = re.match(rb"P5\s+(?:#[^\n]*\n\s*)*(\d+)\s+(\d+)\s+(\d+)\s", raw)
        if m is None:
            raise ValueError(f"{image}: not a binary PGM")
        w, h, maxval = int(m.group(1)), int(m.group(2)), int(m.group(3))
        px = raw[m.end():m.end() + w * h]
        blocked = [False] * (w * h)
        for row in range(h):
            y_index = h - 1 - row  # PGM row 0 is the top (highest y)
            for col in range(w):
                v = px[row * w + col] / maxval
                p = v if negate else 1.0 - v  # occupancy probability
                blocked[y_index * w + col] = p > free + 1e-6  # occupied or unknown: the robot cannot be there
        return cls(w, h, resolution, (ox, oy), blocked)

    def blocked(self, x: float, y: float) -> bool:
        i = int(math.floor((x - self.origin[0]) / self.resolution))
        j = int(math.floor((y - self.origin[1]) / self.resolution))
        if i < 0 or j < 0 or i >= self.width or j >= self.height:
            return True
        return self._blocked[j * self.width + i]


@dataclass(frozen=True)
class StraightMove:
    """A straight drive at a fixed heading; reverse = the robot backs along it."""

    start: Point
    end: Point
    yaw: float
    reverse: bool

    @property
    def length(self) -> float:
        return math.hypot(self.end[0] - self.start[0], self.end[1] - self.start[1])

    def poses(self, step: float = 0.05) -> list[Pose]:
        n = max(1, int(math.ceil(self.length / step)))
        return [(self.start[0] + (self.end[0] - self.start[0]) * k / n,
                 self.start[1] + (self.end[1] - self.start[1]) * k / n, self.yaw) for k in range(n + 1)]


@dataclass(frozen=True)
class DockPlan:
    exit: Optional[StraightMove] = None  # leave a bay straight before anything else
    approach: Optional[Pose] = None  # Nav2 goal: the turning point, at the endpoint heading (None: the goal itself)
    run_in: Optional[StraightMove] = None  # straight drive onto the endpoint

    @property
    def plain(self) -> bool:
        return self.exit is None and self.approach is None and self.run_in is None


class DockingGeometry:
    def __init__(
        self,
        grid: OccupancyMap,
        footprint: Sequence[Point],
        *,
        padding: float = 0.03,
        max_dist: float = 0.9,
        step: float = 0.05,
        turn_step_rad: float = math.radians(10.0),
    ) -> None:
        self.grid = grid
        self.max_dist = max_dist
        self.step = step
        self.turn_step = turn_step_rad
        self._outline = self._sample_outline(self._pad(footprint, padding), 0.5 * grid.resolution)

    @staticmethod
    def _pad(footprint: Sequence[Point], padding: float) -> list[Point]:
        out = []
        for x, y in footprint:
            out.append((x + math.copysign(padding, x) if x else x, y + math.copysign(padding, y) if y else y))
        return out

    @staticmethod
    def _sample_outline(poly: Sequence[Point], spacing: float) -> list[Point]:
        pts: list[Point] = []
        for k in range(len(poly)):
            (ax, ay), (bx, by) = poly[k], poly[(k + 1) % len(poly)]
            n = max(1, int(math.ceil(math.hypot(bx - ax, by - ay) / spacing)))
            pts += [(ax + (bx - ax) * i / n, ay + (by - ay) * i / n) for i in range(n)]
        # A coarse interior grid too: a thin wall can sit between two outline samples of a moved footprint.
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        nx = max(1, int((max(xs) - min(xs)) / (2 * spacing)))
        ny = max(1, int((max(ys) - min(ys)) / (2 * spacing)))
        for i in range(nx + 1):
            for j in range(ny + 1):
                pts.append((min(xs) + (max(xs) - min(xs)) * i / nx, min(ys) + (max(ys) - min(ys)) * j / ny))
        return pts

    def free(self, x: float, y: float, yaw: float) -> bool:
        c, s = math.cos(yaw), math.sin(yaw)
        return not any(self.grid.blocked(x + c * px - s * py, y + s * px + c * py) for px, py in self._outline)

    def can_turn(self, x: float, y: float) -> bool:
        n = int(math.ceil(2 * math.pi / self.turn_step))
        return all(self.free(x, y, -math.pi + k * 2 * math.pi / n) for k in range(n))

    def _clear_line(self, a: Point, b: Point, yaw: float) -> bool:
        n = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / (0.5 * self.grid.resolution))))
        return all(self.free(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n, yaw) for k in range(n + 1))

    def _turning_point(self, origin: Pose, direction: float) -> Optional[Point]:
        """Nearest point on the line from origin along direction (yaw + 0 or pi) where the footprint, kept at the
        origin heading on the way, can turn. None if the line is blocked first or none is within max_dist."""
        x0, y0, yaw = origin
        ux, uy = math.cos(direction), math.sin(direction)
        prev = (x0, y0)
        d = self.step
        while d <= self.max_dist + 1e-9:
            p = (x0 + d * ux, y0 + d * uy)
            if not self._clear_line(prev, p, yaw):
                return None
            if self.can_turn(*p):
                return p
            prev = p
            d += self.step
        return None

    def plan(self, start: Pose, goal: Pose) -> DockPlan:
        gx, gy, gyaw = goal
        goal_needs_dock = not self.can_turn(gx, gy)

        # Already on the axis of a bay endpoint (a refine): straight onto it, nothing else.
        if goal_needs_dock:
            ux, uy = math.cos(gyaw), math.sin(gyaw)
            rx, ry = start[0] - gx, start[1] - gy
            along, across = rx * ux + ry * uy, -rx * uy + ry * ux
            if (abs(across) <= 0.08 and abs(_wrap(start[2] - gyaw)) <= 0.35 and abs(along) <= self.max_dist
                    and self._clear_line((start[0], start[1]), (gx, gy), gyaw)):
                return DockPlan(run_in=StraightMove((start[0], start[1]), (gx, gy), gyaw, reverse=along > 0))

        exit_move: Optional[StraightMove] = None
        if not self.can_turn(start[0], start[1]):
            candidates = []
            for direction, reverse in ((start[2], False), (start[2] + math.pi, True)):
                p = self._turning_point(start, direction)
                if p is not None:
                    candidates.append((math.hypot(p[0] - start[0], p[1] - start[1]), reverse, p))
            if candidates:
                _, reverse, p = min(candidates)
                exit_move = StraightMove((start[0], start[1]), p, start[2], reverse)

        approach: Optional[Pose] = None
        run_in: Optional[StraightMove] = None
        if goal_needs_dock:
            candidates = []
            # forward in: turn behind the endpoint; backward in: turn in front of it and back onto it
            for direction, reverse in ((gyaw + math.pi, False), (gyaw, True)):
                p = self._turning_point(goal, direction)
                if p is not None:
                    candidates.append((math.hypot(p[0] - gx, p[1] - gy) + (0.0 if not reverse else 0.01), reverse, p))
            if candidates:
                _, reverse, p = min(candidates)
                approach = (p[0], p[1], gyaw)
                run_in = StraightMove(p, (gx, gy), gyaw, reverse)
        return DockPlan(exit=exit_move, approach=approach, run_in=run_in)
