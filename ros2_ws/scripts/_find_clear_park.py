#!/usr/bin/env python3
"""Find a free PARK pose near EP-PARK-5 on sim_warehouse map."""
from __future__ import annotations
import math
import sys
from pathlib import Path

sys.path.insert(0, "/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/scripts")
from spawn_pose import footprint_clear, load_occupied_cells_ros  # noqa: E402

MAP = Path("/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/sim_warehouse.yaml")
cells, meta = load_occupied_cells_ros(MAP)
ox, oy, res = meta["origin_x"], meta["origin_y"], meta["resolution"]
print(f"occupied={len(cells)} origin=({ox},{oy}) res={res}")

base = (-1.932, -0.495, -math.pi / 2)
candidates = []
for dx in [i * 0.05 for i in range(-12, 13)]:
    for dy in [i * 0.05 for i in range(-12, 13)]:
        pose = {"x": base[0] + dx, "y": base[1] + dy, "yaw": base[2]}
        ok, reason = footprint_clear(
            pose,
            occupied_cells=cells,
            origin_x=ox,
            origin_y=oy,
            resolution=res,
        )
        if ok:
            dist = math.hypot(dx, dy)
            candidates.append((dist, pose["x"], pose["y"], pose["yaw"]))

candidates.sort()
print(f"clear_candidates={len(candidates)}")
for c in candidates[:15]:
    print(f"  dist={c[0]:.3f} x={c[1]:.3f} y={c[2]:.3f} yaw={c[3]:.3f}")

# also test current and charger/park6
for label, x, y in [
    ("PARK5", -1.932, -0.495),
    ("PARK6", -1.455, -0.493),
    ("CHG", -1.019, -0.491),
]:
    ok, reason = footprint_clear(
        {"x": x, "y": y, "yaw": -math.pi / 2},
        occupied_cells=cells,
        origin_x=ox,
        origin_y=oy,
        resolution=res,
    )
    print(f"{label} ({x},{y}) clear={ok} {reason}")
