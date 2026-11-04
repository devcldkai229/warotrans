#!/usr/bin/env python3
from pathlib import Path
import re

p = Path(
    "/mnt/e/programming/my_project/warotrans-system/docs/audits/"
    "autonomous_gazebo_acceptance/runtime/dual_orch_bringup.log"
)
text = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", p.read_text(errors="replace"))
print("nav_active", len(re.findall(r"lifecycle_manager_navigation.*Managed nodes are active", text)))
print("gw semantic", text.count("traffic_mode=semantic_active"))
print("gateway started", text.count("Robot gateway started"))
print("MQTT connected", text.count("MQTT connected"))
print("process has died", text.count("process has died"))
print("IndentationError", text.count("IndentationError"))
print("Traceback", text.count("Traceback"))
keys = (
    "robot_gateway",
    "MQTT",
    "semantic_active",
    "Indentation",
    "Traceback",
    "process has died",
    "gateway started",
    "paho",
    "ImportError",
    "ModuleNotFound",
)
for i, line in enumerate(text.splitlines()):
    if any(k in line for k in keys):
        print(f"{i}: {line[:260]}")
