"""Paths for the simulated robots on a real map bundle, like the lane-aware Nav2 planner would give them.

A* on the bundle's 5 cm grid: blocked = not free in map.pgm or inside keepout_planning.pgm (the inflated keepout the
real planner uses), grown by a hard clearance; cells near obstacles cost more, and driving against a one-way lane
costs more. The backend refuses any plan that touches a keepout zone, so these paths are also what makes a sim
robot's traffic plan acceptable on that map. Nothing here is the backend's or the robot's code.

    paths = MapPaths(bundle_dir)
    sim = SimRobot(..., path_provider=paths.provider())
"""

from __future__ import annotations

import heapq
import json
import math
from pathlib import Path
from typing import Optional

import numpy as np
from PIL import Image


def _read_yaml_origin(path: Path) -> tuple[float, list[float]]:
    res, origin = 0.05, [0.0, 0.0, 0.0]
    lines = path.read_text(encoding="utf-8").replace("\r", "").splitlines()
    for i, line in enumerate(lines):
        if line.startswith("resolution:"):
            res = float(line.split(":", 1)[1])
        if line.startswith("origin:"):
            rest = line.split(":", 1)[1].strip()
            if rest.startswith("["):
                origin = [float(v) for v in rest.strip("[]").split(",")]
            else:  # block list
                origin = [float(lines[i + k].strip().lstrip("-").strip()) for k in (1, 2, 3)]
    return res, origin


class MapPaths:
    def __init__(self, bundle: Path, clearance_m: float = 0.10, soft_m: float = 0.30, wrong_way_cost: float = 4.0) -> None:
        self.res, origin = _read_yaml_origin(bundle / "map.yaml")
        self.ox, self.oy = origin[0], origin[1]
        occ = np.array(Image.open(bundle / "map.pgm").convert("L"))
        free = occ >= 250  # trinary: 254 free, 0 occupied, 205 unknown
        keep_path = bundle / "keepout_planning.pgm"
        if keep_path.is_file():
            keep = np.array(Image.open(keep_path).convert("L"))
            free &= keep >= 250
        self.h, self.w = free.shape
        blocked = ~free
        # Distance (cells) to the nearest blocked cell, by brute dilation rings (the grid is small).
        self.dist = np.full(free.shape, 99.0)
        ys, xs = np.nonzero(blocked)
        reach = int(math.ceil(soft_m / self.res)) + 1
        for y, x in zip(ys, xs):
            y0, y1, x0, x1 = max(0, y - reach), min(self.h, y + reach + 1), max(0, x - reach), min(self.w, x + reach + 1)
            yy, xx = np.mgrid[y0:y1, x0:x1]
            d = np.hypot(yy - y, xx - x)
            np.minimum(self.dist[y0:y1, x0:x1], d, out=self.dist[y0:y1, x0:x1])
        self.hard = self.dist * self.res < clearance_m
        self.soft_cells = soft_m / self.res
        self.wrong_way_cost = wrong_way_cost
        self.lanes = []
        lanes_path = bundle / "lanes.json"
        if lanes_path.is_file():
            for lane in json.loads(lanes_path.read_text(encoding="utf-8")).get("lanes", []):
                if lane.get("direction") != "ONE_WAY":
                    continue
                pts = lane["coordinates"]
                for a, b in zip(pts, pts[1:]):
                    self.lanes.append((a, b, lane["width_meters"] / 2.0))

    # ------------------------------------------------------------------ grid helpers

    def cell(self, x: float, y: float) -> tuple[int, int]:
        # map.pgm row 0 is the top (max y).
        return int((x - self.ox) / self.res), self.h - 1 - int((y - self.oy) / self.res)

    def world(self, cx: int, cy: int) -> tuple[float, float]:
        return self.ox + (cx + 0.5) * self.res, self.oy + (self.h - 1 - cy + 0.5) * self.res

    def _nearest_open(self, c: tuple[int, int]) -> tuple[int, int]:
        cx, cy = c
        if 0 <= cx < self.w and 0 <= cy < self.h and not self.hard[cy, cx]:
            return c
        for r in range(1, 40):
            best = None
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    x, y = cx + dx, cy + dy
                    if 0 <= x < self.w and 0 <= y < self.h and not self.hard[y, x]:
                        d = dx * dx + dy * dy
                        if best is None or d < best[0]:
                            best = (d, (x, y))
            if best:
                return best[1]
        return c

    def _lane_penalty(self, x: float, y: float, heading: float) -> float:
        for (ax, ay), (bx, by), half in self.lanes:
            vx, vy = bx - ax, by - ay
            seg = vx * vx + vy * vy
            t = max(0.0, min(1.0, ((x - ax) * vx + (y - ay) * vy) / seg)) if seg > 0 else 0.0
            if math.hypot(x - (ax + t * vx), y - (ay + t * vy)) <= half:
                if math.cos(heading - math.atan2(vy, vx)) < -0.3:
                    return self.wrong_way_cost
        return 0.0

    # ------------------------------------------------------------------ planning

    def plan(self, x0: float, y0: float, x1: float, y1: float) -> Optional[list[tuple[float, float]]]:
        start, goal = self._nearest_open(self.cell(x0, y0)), self._nearest_open(self.cell(x1, y1))
        moves = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        g = {start: 0.0}
        came: dict = {}
        heap = [(0.0, start)]
        while heap:
            _, cur = heapq.heappop(heap)
            if cur == goal:
                break
            cx, cy = cur
            wx, wy = self.world(cx, cy)
            for dx, dy in moves:
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < self.w and 0 <= ny < self.h) or self.hard[ny, nx]:
                    continue
                step = math.hypot(dx, dy)
                near = max(0.0, 1.0 - self.dist[ny, nx] / self.soft_cells)
                heading = math.atan2(-dy, dx)  # grid y grows downwards
                cost = step * (1.0 + 2.0 * near + self._lane_penalty(wx, wy, heading))
                ng = g[cur] + cost
                if ng < g.get((nx, ny), float("inf")):
                    g[(nx, ny)] = ng
                    came[(nx, ny)] = cur
                    heapq.heappush(heap, (ng + math.hypot(goal[0] - nx, goal[1] - ny), (nx, ny)))
        if goal not in g:
            return None
        cells = [goal]
        while cells[-1] != start:
            cells.append(came[cells[-1]])
        cells.reverse()
        pts = [self.world(*c) for c in cells[::2]] + [self.world(*goal)]
        pts[0] = (x0, y0)
        pts[-1] = (x1, y1)
        return pts

    def provider(self):
        """SimRobot.path_provider: (x, y, goal) -> [(x, y, yaw)]."""

        def provide(x: float, y: float, goal) -> list[tuple[float, float, float]]:
            pts = self.plan(x, y, goal.x, goal.y)
            if not pts:
                return []
            out = []
            for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                out.append((ax, ay, math.atan2(by - ay, bx - ax)))
            out.append((pts[-1][0], pts[-1][1], out[-1][2] if out else 0.0))
            return out

        return provide
