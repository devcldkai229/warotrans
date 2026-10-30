#!/usr/bin/env python3
"""Find PARK pose with footprint + buffer clearance near parking corridor."""
from __future__ import annotations
import math
import sys
from pathlib import Path

sys.path.insert(0, "/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/scripts")
from spawn_pose import footprint_clear, load_occupied_cells_ros, DEFAULT_HALF_L, DEFAULT_HALF_W

MAP = Path("/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/sim_warehouse.yaml")
cells, meta = load_occupied_cells_ros(MAP)
ox, oy, res = meta["origin_x"], meta["origin_y"], meta["resolution"]

# Prefer near EDGE-353 corridor y=-0.99, x in [-2.1,-0.7]
best = []
for x in [i * 0.05 for i in range(int((-2.2 - ox) / 0.05), int((-0.7 - ox) / 0.05))]:
    for y in [i * 0.05 for i in range(int((-1.2 - oy) / 0.05), int((-0.35 - oy) / 0.05))]:
        for yaw in (-math.pi / 2, 0.0, math.pi / 2):
            pose = {"x": x, "y": y, "yaw": yaw}
            # inflated footprint ~ collision monitor
            ok, reason = footprint_clear(
                pose,
                occupied_cells=cells,
                origin_x=ox,
                origin_y=oy,
                resolution=res,
                half_l=DEFAULT_HALF_L + 0.20,
                half_w=DEFAULT_HALF_W + 0.20,
            )
            if not ok:
                continue
            # prefer near nominal park x and corridor
            score = abs(x - (-1.932)) + abs(y - (-0.99)) * 0.5
            best.append((score, x, y, yaw))

best.sort()
print(f"buffered_clear={len(best)}")
for b in best[:20]:
    print(f"  score={b[0]:.3f} x={b[1]:.3f} y={b[2]:.3f} yaw={b[3]:.3f}")
