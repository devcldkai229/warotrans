#!/usr/bin/env python3
"""Print one NavigateToPose command per endpoint (from Parking A spawn)."""
from __future__ import annotations

import json
import math
from pathlib import Path

EP_JSON = Path(
    "/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/config/map_one_v001_endpoints.json"
)


def main() -> None:
    data = json.loads(EP_JSON.read_text(encoding="utf-8"))
    eps = [e for e in data["endpoints"] if e["code"] != "EP-PARKING-A"]
    eps.sort(key=lambda e: e["code"])
    print("# Source setup (WSL, once):")
    print("source /opt/ros/jazzy/setup.bash")
    print("source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash")
    print("export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}")
    print()
    print("# Cancel active goal before each new one (sim01 namespace):")
    print(
        "ros2 service call /sim01/navigate_to_pose/_action/cancel_goal "
        "action_msgs/srv/CancelGoal \"{}\""
    )
    print()
    for e in eps:
        yaw = float(e.get("yaw") or 0.0)
        z = math.sin(yaw / 2.0)
        w = math.cos(yaw / 2.0)
        x = float(e["x"])
        y = float(e["y"])
        code = e["code"]
        print(f"# --- Parking A → {code} ({e.get('name')}) ---")
        print(
            "timeout 180 ros2 action send_goal /sim01/navigate_to_pose nav2_msgs/action/NavigateToPose "
            f"\"{{pose: {{header: {{frame_id: map}}, pose: {{position: {{x: {x}, y: {y}, z: 0.0}}, "
            f"orientation: {{z: {z:.6f}, w: {w:.6f}}}}}}}}}\""
        )
        print()


if __name__ == "__main__":
    main()
