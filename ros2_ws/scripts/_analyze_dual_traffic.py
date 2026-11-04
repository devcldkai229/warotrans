#!/usr/bin/env python3
from pathlib import Path
import re

p = Path(
    "/mnt/e/programming/my_project/warotrans-system/docs/audits/"
    "autonomous_gazebo_acceptance/runtime/dual_orch_bringup.log"
)
text = re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", p.read_text(errors="replace"))
keys = (
    "traffic_plan",
    "authority_ok",
    "authority gate",
    "BLOCKING",
    "semantic_active",
    "SemanticTraffic",
    "grant",
    "fingerprint",
    "traffic_mode",
    "Propose",
    "on_plan",
    "MQTT",
    "queue full",
    "NavigateToPose",
    "Goal accepted",
    "cmd_vel",
)
for i, line in enumerate(text.splitlines()):
    if any(k.lower() in line.lower() for k in keys):
        print(f"{i}: {line[:280]}")
print("---COUNTS---")
for k in [
    "traffic_mode=semantic_active",
    "authority gate BLOCKING",
    "MQTT outbound queue full",
    "Goal succeeded",
    "Received a goal",
]:
    print(k, text.count(k))
