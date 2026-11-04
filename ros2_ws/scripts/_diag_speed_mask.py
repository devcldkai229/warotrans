#!/usr/bin/env python3
from collections import Counter
from pathlib import Path

p = Path("/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/map_one_v001/speed.pgm")
data = p.read_bytes()
if not data.startswith(b"P5"):
    raise SystemExit(f"not P5: {data[:16]!r}")
# Parse P5 header: magic, dims, maxval — comments allowed
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
w, h, maxval = int(tokens[0]), int(tokens[1]), int(tokens[2])
while idx < len(data) and data[idx] in b" \t\r":
    idx += 1
if idx < len(data) and data[idx : idx + 1] in (b"\n", b"\r"):
    idx += 1
    if idx < len(data) and data[idx - 1 : idx + 1] == b"\r\n":
        pass
pixels = list(data[idx : idx + w * h])
print(f"size={w}x{h} maxval={maxval} pixels={len(pixels)}")
c = Counter(pixels)
print("top", c.most_common(12))
print("nonzero", sum(1 for v in pixels if v), "zero", sum(1 for v in pixels if v == 0))

origin_x, origin_y, res = -2.929, -1.415, 0.05
for name, x, y in [
    ("OUTBOUND", 1.793, -0.977),
    ("stuck", 1.774, -0.125),
    ("WAIT", 1.292, 0.83),
    ("INBOUND", 1.747, 1.543),
    ("RTL-C-east", 1.638, 0.186),
    ("LTR-C-east", 1.648, -0.327),
]:
    cx = int((x - origin_x) / res)
    cy = int((y - origin_y) / res)
    idx_bl = cy * w + cx
    idx_tl = (h - 1 - cy) * w + cx
    v_bl = pixels[idx_bl] if 0 <= idx_bl < len(pixels) else None
    v_tl = pixels[idx_tl] if 0 <= idx_tl < len(pixels) else None
    print(f"{name} cell=({cx},{cy}) mask_bl={v_bl} mask_tl={v_tl}")
