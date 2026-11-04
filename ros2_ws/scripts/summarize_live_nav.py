#!/usr/bin/env python3
from pathlib import Path
import re

art = Path("/tmp/nav2_live_artifacts")
for name in [
    "nav_parking_to_inbound",
    "nav_inbound_to_shelf",
    "nav_shelf_to_outbound",
    "nav_outbound_to_parking",
]:
    t = (art / f"{name}.txt").read_text(errors="ignore")
    codes = re.findall(r"error_code:\s*(\d+)", t)
    print(
        name,
        "SUCCEEDED" if "SUCCEEDED" in t else "",
        "ABORTED" if "ABORTED" in t else "",
        "codes",
        codes[-3:],
        "lines",
        t.count("\n"),
    )
    print("  ", " | ".join(t.strip().splitlines()[-6:]))

log = (art / "live_bringup.log").read_text(errors="ignore")
rejects = re.findall(r"path_policy REJECT: ([^\n]+)", log)
from collections import Counter

print("reject_counts", Counter(rejects).most_common(10))
print("reject_total", len(rejects))
for pat in ("Navigation completed", "Goal succeeded", "progress_checker", "Timed out", "ABORTED"):
    print(pat, log.count(pat))
