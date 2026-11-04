#!/usr/bin/env python3
import json
from collections import Counter
from pathlib import Path

import rclpy
from nav2_msgs.srv import GetCostmap
from rclpy.node import Node
from rclpy.parameter import Parameter


class N(Node):
    def __init__(self):
        super().__init__(
            "dump_cm",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.cli = self.create_client(GetCostmap, "/global_costmap/get_costmap")


def main():
    rclpy.init()
    n = N()
    if not n.cli.wait_for_service(timeout_sec=20.0):
        print("no service")
        return 1
    fut = n.cli.call_async(GetCostmap.Request())
    import time

    t0 = time.time()
    while not fut.done() and time.time() - t0 < 15:
        rclpy.spin_once(n, timeout_sec=0.1)
    resp = fut.result()
    m = resp.map
    meta = m.metadata
    data = list(m.data)
    print(
        "meta",
        meta.resolution,
        meta.size_x,
        meta.size_y,
        meta.origin.position.x,
        meta.origin.position.y,
        "len",
        len(data),
    )
    print("hist", Counter(data).most_common(10))
    # sample parking cell
    ox, oy, res = meta.origin.position.x, meta.origin.position.y, meta.resolution
    for name, x, y in [
        ("park", -1.848, -0.525),
        ("in", 1.747, 1.543),
        ("mid", 1.1, -0.327),
    ]:
        mx = int((x - ox) / res)
        my = int((y - oy) / res)
        v = data[my * meta.size_x + mx] if 0 <= mx < meta.size_x and 0 <= my < meta.size_y else None
        print(name, "cell", mx, my, "val", v)
    Path("/tmp/nav2_live_artifacts/costmap_hist.json").write_text(
        json.dumps({"hist": Counter(data).most_common(20), "len": len(data)}, indent=2)
    )
    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
