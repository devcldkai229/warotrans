#!/usr/bin/env python3
"""Sample occupancy + keepout PGM at key poses (bottom-left origin assumption)."""
from pathlib import Path


def load_p5(path: Path):
    data = path.read_bytes()
    assert data.startswith(b"P5")
    idx = 2
    tokens = []
    while len(tokens) < 3:
        while idx < len(data) and data[idx] in b" \t\n\r":
            idx += 1
        if data[idx : idx + 1] == b"#":
            while idx < len(data) and data[idx] not in b"\n\r":
                idx += 1
            continue
        start = idx
        while idx < len(data) and data[idx] not in b" \t\n\r":
            idx += 1
        tokens.append(data[start:idx].decode())
    w, h, maxval = map(int, tokens)
    while idx < len(data) and data[idx] in b" \t\r":
        idx += 1
    if idx < len(data) and data[idx] in b"\n\r":
        idx += 1
    return w, h, list(data[idx : idx + w * h])


base = Path("/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/map_one_v001")
ow, oh, occ = load_p5(base / "map.pgm")
kw, kh, keep = load_p5(base / "keepout.pgm")
print(f"map {ow}x{oh} keepout {kw}x{kh}")
origin_x, origin_y, res = -2.929, -1.415, 0.05
poses = [
    ("OUTBOUND", 1.793, -0.977),
    ("stuck", 1.774, -0.125),
    ("west_of_stuck", 1.40, -0.125),
    ("WAIT", 1.292, 0.83),
    ("between_lanes", 1.20, -0.05),
    ("RTL_entry", 1.60, 0.186),
]
for name, x, y in poses:
    cx = int((x - origin_x) / res)
    cy = int((y - origin_y) / res)
    # Try both row conventions
    for label, yy in [("bl", cy), ("tl", oh - 1 - cy)]:
        i = yy * ow + cx
        o = occ[i] if 0 <= i < len(occ) else None
        k = keep[i] if 0 <= i < len(keep) else None
        print(f"{name:16} {label} ({cx},{yy}) occ={o} keep={k}")
