"""Live Nav2 costmap → MQTT `robots/{code}/costmap`, for the admin map overlay (view only).

The global costmap (whole map, ~86x72 cells on MAP-ONE) every `global_period_s`, the local one (rolling window
around the robot) every `local_period_s`. Cells keep Nav2's values (0 free … 99 inscribed, 100 lethal, -1 unknown
→ 255); a grid larger than `max_cells` per side is max-pooled so the most restrictive cost survives. Nothing on the
robot or in the backend acts on this message: it is only drawn.
"""

from __future__ import annotations

import base64
import time
from typing import Any, Callable, Sequence

UNKNOWN = 255


def encode_grid(values: Sequence[int], width: int, height: int, max_cells: int = 160) -> tuple[bytes, int, int, int]:
    """Row-major OccupancyGrid data (row 0 = bottom, ROS order) → (bytes, width, height, factor).

    Downsamples by an integer factor with max-pooling (unknown counts as lowest priority). Output rows stay bottom-up.
    """
    factor = max(1, -(-max(width, height) // max_cells))
    out_w, out_h = -(-width // factor), -(-height // factor)
    out = bytearray(out_w * out_h)
    for oy in range(out_h):
        for ox in range(out_w):
            best = -1
            unknown = False
            for y in range(oy * factor, min(height, (oy + 1) * factor)):
                row = y * width
                for x in range(ox * factor, min(width, (ox + 1) * factor)):
                    v = values[row + x]
                    if v < 0:
                        unknown = True
                    elif v > best:
                        best = v
            out[oy * out_w + ox] = UNKNOWN if best < 0 and unknown else max(0, min(100, best))
    return bytes(out), out_w, out_h, factor


def build_message(robot_code: str, kind: str, frame_id: str, resolution: float, origin_x: float, origin_y: float,
                  values: Sequence[int], width: int, height: int, stamp_s: float, max_cells: int = 160) -> dict[str, Any]:
    data, w, h, factor = encode_grid(values, width, height, max_cells)
    return {
        "robotCode": robot_code,
        "kind": kind,
        "frameId": frame_id,
        "resolution": resolution * factor,
        "originX": origin_x,
        "originY": origin_y,
        "width": w,
        "height": h,
        "rowOrder": "bottom-up",
        "data": base64.b64encode(data).decode("ascii"),
        "stamp": stamp_s,
    }


class CostmapUplink:
    """Subscribes the Nav2 costmaps on `node` and publishes them, rate-limited, through `publish(topic_leaf, body)`."""

    def __init__(self, node: Any, robot_code: str, publish: Callable[[str, dict[str, Any]], Any],
                 global_topic: str = "/global_costmap/costmap", local_topic: str = "/local_costmap/costmap",
                 global_period_s: float = 3.0, local_period_s: float = 1.0) -> None:
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self._robot = robot_code
        self._publish = publish
        self._period = {"global": global_period_s, "local": local_period_s}
        self._last: dict[str, float] = {"global": 0.0, "local": 0.0}
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, global_topic, lambda m: self._on(m, "global"), qos)
        node.create_subscription(OccupancyGrid, local_topic, lambda m: self._on(m, "local"), 1)

    def _on(self, msg: Any, kind: str) -> None:
        now = time.monotonic()
        if now - self._last[kind] < self._period[kind]:
            return
        self._last[kind] = now
        info = msg.info
        body = build_message(self._robot, kind, msg.header.frame_id or "map", float(info.resolution),
                             float(info.origin.position.x), float(info.origin.position.y),
                             msg.data, int(info.width), int(info.height), time.time())
        try:
            self._publish("costmap", body)
        except Exception:  # noqa: BLE001 - a view sample is never worth disturbing the gateway
            pass
