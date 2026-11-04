#!/usr/bin/env python3
"""Sample global costmap via GetCostmap service (no TF, no ros2cli daemon)."""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


TRANSIENT = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def classify(c: int) -> str:
    if c == 255:
        return "NO_INFORMATION"
    if c == 254:
        return "LETHAL"
    if c == 253:
        return "INSCRIBED"
    if c == 0:
        return "FREE"
    if c < 253:
        return f"INFLATED({c})"
    return f"OTHER({c})"


def sample_grid(grid: OccupancyGrid, x: float, y: float):
    info = grid.info
    mx = int((x - info.origin.position.x) / info.resolution)
    my = int((y - info.origin.position.y) / info.resolution)
    if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
        return {"cost": None, "class": "OOB", "mx": mx, "my": my}
    # nav2 Costmap service returns nav2_msgs/Costmap; OccupancyGrid path uses data[]
    v = int(grid.data[my * info.width + mx])
    return {"cost": v, "class": classify(v), "mx": mx, "my": my}


class Diag(Node):
    def __init__(self) -> None:
        super().__init__(
            "diag_get_costmap",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.map_msg = None
        self.keepout_msg = None
        self.create_subscription(OccupancyGrid, "/map", self._on_map, TRANSIENT)
        self.create_subscription(
            OccupancyGrid, "/keepout_filter_mask", self._on_ko, TRANSIENT
        )
        self.cli = self.create_client(GetCostmap, "/global_costmap/get_costmap")

    def _on_map(self, msg):
        self.map_msg = msg

    def _on_ko(self, msg):
        self.keepout_msg = msg


def costmap_to_occ(cmap) -> OccupancyGrid:
    """Convert nav2_msgs/Costmap to OccupancyGrid-like structure for sampling."""
    g = OccupancyGrid()
    g.info.resolution = cmap.metadata.resolution
    g.info.width = cmap.metadata.size_x
    g.info.height = cmap.metadata.size_y
    g.info.origin = cmap.metadata.origin
    # Costmap data is already 0-255 costs
    g.data = list(cmap.data)
    return g


def main() -> int:
    poses = [
        ("EP-PARKING-A", -1.848, -0.525, -math.pi / 2),
        ("EP-INBOUND", 1.747, 1.543, -math.pi / 2),
        ("EP-OUTBOUND", 1.793, -0.977, math.pi / 2),
        ("EP-SHELF-A-01", -1.325, 1.644, math.pi),
        ("ONEWAY_MID", 1.1, -0.327, 0.0),
        ("FREEISH", -1.5, 0.2, 0.0),
        ("EP-CHARGING-A", -1.02, -0.529, -math.pi / 2),
    ]
    out = Path("/tmp/nav2_live_artifacts/diag_costmap_lethal.json")
    out.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Diag()
    try:
        t0 = time.time()
        while time.time() - t0 < 30.0:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.map_msg:
                break

        if not node.cli.wait_for_service(timeout_sec=30.0):
            raise RuntimeError("get_costmap service not available")

        fut = node.cli.call_async(GetCostmap.Request())
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 15.0:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not fut.done():
            raise RuntimeError("get_costmap call timed out")
        resp = fut.result()
        cost_occ = costmap_to_occ(resp.map)

        # drain keepout a bit more
        t0 = time.time()
        while time.time() - t0 < 5.0:
            rclpy.spin_once(node, timeout_sec=0.2)

        samples = []
        for name, x, y, yaw in poses:
            row = {"name": name, "world": {"x": x, "y": y, "yaw": yaw}}
            if node.map_msg:
                row["map"] = sample_grid(node.map_msg, x, y)
            row["costmap"] = sample_grid(cost_occ, x, y)
            if node.keepout_msg:
                row["keepout"] = sample_grid(node.keepout_msg, x, y)
            # footprint corners max
            hl, hw = 0.16, 0.13
            cy, sy = math.cos(yaw), math.sin(yaw)
            fp = []
            for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):
                wx = x + dx * cy - dy * sy
                wy = y + dx * sy + dy * cy
                fp.append(sample_grid(cost_occ, wx, wy))
            row["footprint"] = fp
            row["footprint_max_class"] = classify(
                max(p["cost"] if p["cost"] is not None else -1 for p in fp)
            )
            samples.append(row)
            print(
                f"{name}: map={row.get('map')} cost={row['costmap']} "
                f"fp_max={row['footprint_max_class']} keepout={row.get('keepout')}"
            )

        meta = {
            "costmap_meta": {
                "res": cost_occ.info.resolution,
                "w": cost_occ.info.width,
                "h": cost_occ.info.height,
                "ox": cost_occ.info.origin.position.x,
                "oy": cost_occ.info.origin.position.y,
            },
            "map_received": node.map_msg is not None,
            "keepout_received": node.keepout_msg is not None,
            "samples": samples,
            "root_cause_hint": (
                "LaneAware rejects start/goal when cost>=253 (INSCRIBED). "
                "If samples show INSCRIBED or NO_INFORMATION, that explains start/goal lethal."
            ),
        }
        out.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
        print(f"WROTE {out}", file=sys.stderr)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
