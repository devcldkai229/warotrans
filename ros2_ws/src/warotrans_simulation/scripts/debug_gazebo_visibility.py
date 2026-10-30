#!/usr/bin/env python3
"""Debug: is warotrans in Gazebo, where, vs GUI camera aim (session 100d90)."""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
import time
from pathlib import Path

# #region agent log
LOG = Path("/mnt/e/programming/my_project/warotrans-system/debug-100d90.log")
SESSION = "100d90"


def dbg(hypothesis_id: str, location: str, message: str, data: dict) -> None:
    LOG.parent.mkdir(parents=True, exist_ok=True)
    line = {
        "sessionId": SESSION,
        "runId": os.environ.get("DEBUG_RUN_ID", "gazebo-visibility"),
        "hypothesisId": hypothesis_id,
        "location": location,
        "message": message,
        "data": data,
        "timestamp": int(time.time() * 1000),
    }
    with LOG.open("a", encoding="utf-8") as f:
        f.write(json.dumps(line, ensure_ascii=True) + "\n")


# #endregion


def main() -> int:
    models_out = ""
    try:
        models_out = subprocess.check_output(
            ["gz", "model", "--list"], text=True, stderr=subprocess.STDOUT, timeout=15
        )
    except Exception as exc:
        dbg("A", "debug_gazebo_visibility.py:list", "gz model --list failed", {"error": str(exc)})
        print("FAIL list", exc)
        return 1

    has_robot = bool(re.search(r"^\s*-\s*warotrans\s*$", models_out, re.M))
    dbg(
        "A",
        "debug_gazebo_visibility.py:list",
        "model list",
        {"has_warotrans": has_robot, "snippet": models_out[-500:]},
    )

    pose = None
    if has_robot:
        try:
            pout = subprocess.check_output(
                ["gz", "model", "-m", "warotrans", "-p"],
                text=True,
                stderr=subprocess.STDOUT,
                timeout=15,
            )
            m = re.search(
                r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]\s*\n\s*\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]",
                pout,
            )
            if m:
                pose = {
                    "x": float(m.group(1)),
                    "y": float(m.group(2)),
                    "z": float(m.group(3)),
                    "roll": float(m.group(4)),
                    "pitch": float(m.group(5)),
                    "yaw": float(m.group(6)),
                }
            dbg("C", "debug_gazebo_visibility.py:pose", "warotrans pose", {"pose": pose, "raw": pout[-800:]})
        except Exception as exc:
            dbg("C", "debug_gazebo_visibility.py:pose", "pose failed", {"error": str(exc)})

    # Camera from generated world SDF (install or source)
    cam = None
    for world in (
        Path("/mnt/e/programming/my_project/warotrans/ros2_ws/install/warotrans_simulation/share/warotrans_simulation/worlds/warehouse.sdf"),
        Path("/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/worlds/warehouse.sdf"),
    ):
        if not world.is_file():
            continue
        text = world.read_text(encoding="utf-8")
        cm = re.search(r"<camera name=\"user_camera\">\s*<pose>([^<]+)</pose>", text)
        if cm:
            vals = [float(x) for x in cm.group(1).split()]
            cam = {"x": vals[0], "y": vals[1], "z": vals[2], "rpy": vals[3:6], "world": str(world)}
            break

    # Approximate ground look-at: camera looks +Y (yaw~pi/2) with pitch~0.85
    look_at = None
    if cam:
        pitch = cam["rpy"][1]
        forward = cam["z"] / max(math.tan(pitch), 1e-3)
        look_at = {"x": cam["x"], "y": cam["y"] + forward}
        dbg("B", "debug_gazebo_visibility.py:camera", "gui camera vs look-at", {"camera": cam, "approx_look_at": look_at})

    if pose and look_at:
        dx = pose["x"] - look_at["x"]
        dy = pose["y"] - look_at["y"]
        dist = math.hypot(dx, dy)
        dbg(
            "B",
            "debug_gazebo_visibility.py:delta",
            "robot vs camera look-at",
            {
                "robot_xy": [pose["x"], pose["y"]],
                "look_at_xy": [look_at["x"], look_at["y"]],
                "delta_m": dist,
                "park_expected": [2.959, 0.357],
                "legacy_spawn": [0.0, 0.0],
            },
        )

    # MQTT sample (FE / Host path)
    try:
        import paho.mqtt.client as mqtt

        msgs: list[str] = []

        def on_message(_c, _u, msg):
            msgs.append(msg.payload[:200].decode("utf-8", "replace"))

        c = mqtt.Client(client_id="dbg-vis-100d90")
        c.on_message = on_message
        c.connect("127.0.0.1", 1883, 60)
        c.subscribe("warotrans/v1/robots/RBT-SIM-01/telemetry")
        c.loop_start()
        time.sleep(2.0)
        c.loop_stop()
        pose_mqtt = None
        if msgs:
            try:
                j = json.loads(msgs[-1])
                p = j.get("pose") or j.get("data") or j
                pose_mqtt = {k: p.get(k) for k in ("x", "y", "yaw", "poseX", "poseY") if isinstance(p, dict)}
            except Exception:
                pose_mqtt = {"raw": msgs[-1][:180]}
        dbg(
            "E",
            "debug_gazebo_visibility.py:mqtt",
            "telemetry sample",
            {"count": len(msgs), "pose_fields": pose_mqtt},
        )
    except Exception as exc:
        dbg("E", "debug_gazebo_visibility.py:mqtt", "mqtt failed", {"error": str(exc)})

    print(json.dumps({"has_robot": has_robot, "pose": pose, "camera": cam, "look_at": look_at}, indent=2))
    return 0 if has_robot else 2


if __name__ == "__main__":
    raise SystemExit(main())
