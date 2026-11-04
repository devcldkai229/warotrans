#!/usr/bin/env python3
"""Dump OccupancyGrid vs Nav2 costmap cells for known MAP-ONE poses (Task A)."""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import Any, Optional

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import Buffer, TransformListener

TRANSIENT = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)
VOLATILE = QoSProfile(
    depth=5,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


def classify_cost(c: int) -> str:
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


def world_to_cell(ox: float, oy: float, res: float, wx: float, wy: float) -> tuple[int, int]:
    return int((wx - ox) / res), int((wy - oy) / res)


def cell_value(grid: OccupancyGrid, mx: int, my: int) -> Optional[int]:
    w = grid.info.width
    h = grid.info.height
    if mx < 0 or my < 0 or mx >= w or my >= h:
        return None
    return int(grid.data[my * w + mx])


class Diag(Node):
    def __init__(self) -> None:
        super().__init__(
            "diag_costmap_lethal",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.map_msg: Optional[OccupancyGrid] = None
        self.cost_msg: Optional[OccupancyGrid] = None
        self.keepout_msg: Optional[OccupancyGrid] = None
        for qos in (TRANSIENT, VOLATILE):
            self.create_subscription(OccupancyGrid, "/map", self._on_map, qos)
            self.create_subscription(
                OccupancyGrid, "/global_costmap/costmap", self._on_cost, qos
            )
            self.create_subscription(
                OccupancyGrid, "/global_costmap/costmap_raw", self._on_cost, qos
            )
            self.create_subscription(
                OccupancyGrid, "/keepout_filter_mask", self._on_keepout, qos
            )
        self.tf = Buffer(cache_time=Duration(seconds=30.0))
        self._tfl = TransformListener(self.tf, self)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _on_cost(self, msg: OccupancyGrid) -> None:
        self.cost_msg = msg

    def _on_keepout(self, msg: OccupancyGrid) -> None:
        self.keepout_msg = msg

    def wait_msgs(self, timeout: float = 60.0) -> None:
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.map_msg and self.cost_msg:
                print(
                    f"got map={self.map_msg.info.width}x{self.map_msg.info.height} "
                    f"cost={self.cost_msg.info.width}x{self.cost_msg.info.height} "
                    f"keepout={self.keepout_msg is not None}",
                    file=sys.stderr,
                )
                return
        raise RuntimeError(
            f"timeout waiting grids map={self.map_msg is not None} "
            f"cost={self.cost_msg is not None} keepout={self.keepout_msg is not None}"
        )

    def lookup_base(self) -> Optional[dict[str, float]]:
        try:
            tf = self.tf.lookup_transform(
                "map", "base_footprint", rclpy.time.Time(), timeout=Duration(seconds=3.0)
            )
            return {
                "x": float(tf.transform.translation.x),
                "y": float(tf.transform.translation.y),
                "z": float(tf.transform.translation.z),
            }
        except Exception as ex:  # noqa: BLE001
            print(f"TF warn: {ex}", file=sys.stderr)
            return None

    def sample(self, name: str, x: float, y: float, yaw: float = 0.0) -> dict[str, Any]:
        assert self.map_msg and self.cost_msg
        m = self.map_msg.info
        c = self.cost_msg.info
        mmx, mmy = world_to_cell(m.origin.position.x, m.origin.position.y, m.resolution, x, y)
        cmx, cmy = world_to_cell(c.origin.position.x, c.origin.position.y, c.resolution, x, y)
        occ = cell_value(self.map_msg, mmx, mmy)
        cost = cell_value(self.cost_msg, cmx, cmy)
        ko = None
        if self.keepout_msg:
            k = self.keepout_msg.info
            kmx, kmy = world_to_cell(
                k.origin.position.x, k.origin.position.y, k.resolution, x, y
            )
            ko = cell_value(self.keepout_msg, kmx, kmy)
        hl, hw = 0.16, 0.13
        corners = []
        cy, sy = math.cos(yaw), math.sin(yaw)
        for dx, dy in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)):
            wx = x + dx * cy - dy * sy
            wy = y + dx * sy + dy * cy
            fx, fy = world_to_cell(
                c.origin.position.x, c.origin.position.y, c.resolution, wx, wy
            )
            fv = cell_value(self.cost_msg, fx, fy)
            corners.append(
                {
                    "x": round(wx, 4),
                    "y": round(wy, 4),
                    "cost": fv,
                    "class": classify_cost(fv) if fv is not None else "OOB",
                }
            )
        max_fp = max((p["cost"] if p["cost"] is not None else -1) for p in corners)
        return {
            "name": name,
            "world": {"x": x, "y": y, "yaw": yaw},
            "map_cell": {"mx": mmx, "my": mmy, "occ": occ},
            "costmap_cell": {
                "mx": cmx,
                "my": cmy,
                "cost": cost,
                "class": classify_cost(cost) if cost is not None else "OOB",
            },
            "keepout_mask_cell": ko,
            "footprint_corners": corners,
            "footprint_max_cost": max_fp,
            "footprint_max_class": classify_cost(max_fp) if max_fp >= 0 else "OOB",
        }


def main() -> int:
    poses = [
        ("EP-PARKING-A", -1.848, -0.525, -math.pi / 2),
        ("EP-INBOUND", 1.747, 1.543, -math.pi / 2),
        ("EP-OUTBOUND", 1.793, -0.977, math.pi / 2),
        ("EP-SHELF-A-01", -1.325, 1.644, math.pi),
        ("EP-CHARGING-A", -1.02, -0.529, -math.pi / 2),
        ("ONEWAY_MID", 1.1, -0.327, 0.0),
        ("FREEISH", -1.5, 0.2, 0.0),
    ]
    out = Path("/tmp/nav2_live_artifacts/diag_costmap_lethal.json")
    out.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Diag()
    try:
        node.wait_msgs(60.0)
        # Extra spins for keepout mask
        for _ in range(25):
            rclpy.spin_once(node, timeout_sec=0.2)
        base = node.lookup_base()
        meta = {
            "map_info": {
                "res": node.map_msg.info.resolution,
                "origin": [
                    node.map_msg.info.origin.position.x,
                    node.map_msg.info.origin.position.y,
                ],
                "w": node.map_msg.info.width,
                "h": node.map_msg.info.height,
            },
            "costmap_info": {
                "res": node.cost_msg.info.resolution,
                "origin": [
                    node.cost_msg.info.origin.position.x,
                    node.cost_msg.info.origin.position.y,
                ],
                "w": node.cost_msg.info.width,
                "h": node.cost_msg.info.height,
                "stamp_sec": node.cost_msg.header.stamp.sec,
            },
            "keepout_mask_received": node.keepout_msg is not None,
            "base_footprint_tf": base,
            "samples": [node.sample(n, x, y, yaw) for n, x, y, yaw in poses],
        }
        out.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
        # Compact summary to stdout only
        for s in meta["samples"]:
            cc = s["costmap_cell"]
            print(
                f"{s['name']}: occ={s['map_cell']['occ']} cost={cc['cost']} "
                f"class={cc['class']} fp_max={s['footprint_max_class']} "
                f"keepout={s['keepout_mask_cell']}"
            )
        print(
            f"keepout_mask_received={meta['keepout_mask_received']} base={base}",
            file=sys.stderr,
        )
        print(f"WROTE {out}", file=sys.stderr)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
