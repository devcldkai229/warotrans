#!/usr/bin/env python3
from collections import Counter
from pathlib import Path

pgm_path = Path(
    "/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/map_one_v001/map.pgm"
)
pgm = pgm_path.read_bytes()
assert pgm.startswith(b"P5")
i = 2
while pgm[i] in b" \t\n\r":
    i += 1
while True:
    if pgm[i : i + 1] == b"#":
        while pgm[i] not in b"\n\r":
            i += 1
        while pgm[i] in b" \t\n\r":
            i += 1
        continue
    break
parts = []
while len(parts) < 3:
    while pgm[i] in b" \t\n\r":
        i += 1
    if pgm[i : i + 1] == b"#":
        while pgm[i] not in b"\n\r":
            i += 1
        continue
    j = i
    while pgm[j] not in b" \t\n\r":
        j += 1
    parts.append(pgm[i:j])
    i = j
w, h, maxv = map(int, parts)
while pgm[i] in b" \t\r":
    i += 1
if pgm[i : i + 1] == b"\n":
    i += 1
data = pgm[i : i + w * h]
print("wh", w, h, "max", maxv, "datalen", len(data), "unique", len(set(data)))
print("top", Counter(data).most_common(5))
ox, oy, res = -2.929, -1.415, 0.05


def sample(x: float, y: float):
    col = int((x - ox) / res)
    row_from_bottom = int((y - oy) / res)
    rr = h - 1 - row_from_bottom
    v = data[rr * w + col]
    neigh = []
    for dc, dr in [(0, 0), (1, 0), (-1, 0), (0, 1), (0, -1), (5, 0), (0, 5)]:
        cc, rr2 = col + dc, rr + dr
        if 0 <= cc < w and 0 <= rr2 < h:
            neigh.append(int(data[rr2 * w + cc]))
    return int(v), col, row_from_bottom, rr, neigh


for name, x, y in [
    ("parking", -1.848, -0.525),
    ("oneway", 1.1, -0.327),
    ("inbound", 1.747, 1.543),
    ("center", 0.0, 0.0),
]:
    print(name, sample(x, y))
