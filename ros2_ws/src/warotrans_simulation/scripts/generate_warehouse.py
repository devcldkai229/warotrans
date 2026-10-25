#!/usr/bin/env python3
"""Generate the simulated warehouse from a layout file (default: config/warehouse_layout.yaml).

Two kinds of layout:
    source_map   an existing ROS occupancy map (yaml + pgm), e.g. the one uploaded to the web. The map is used
                 as-is for Nav2 / AMCL and every occupied cell becomes a solid block in Gazebo, so the
                 simulated lidar sees exactly what AMCL expects. Optional `shelves` rectangles turn the cells
                 inside them into racks (detail above the lidar plane only). Optional `web.overlay` (written by
                 scripts/sync_web_map.py) paints the web Map Editor's Zones, lanes and Endpoints on the floor.
    hand-made    walls / shelves / obstacles as boxes (the active config/warehouse_layout.yaml); the map is
                 rasterised from the same boxes.

Outputs (paths relative to the warotrans_simulation package, always the same names):
    worlds/warehouse.sdf          Gazebo Harmonic world
    maps/sim_warehouse.pgm/.yaml  Nav2 occupancy map in the same frame (world == map)
    config/sim_endpoints.json     docking poses for backend Navigation Endpoints

Usage:
    python3 scripts/generate_warehouse.py                                   # active layout
    python3 scripts/generate_warehouse.py --layout path/to/other_layout.yaml
    python3 scripts/generate_warehouse.py --check                           # fail if outputs are stale

Only the standard library + PyYAML are used so it runs anywhere ROS runs.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import yaml

PKG_ROOT = Path(__file__).resolve().parent.parent
LAYOUT = PKG_ROOT / "config" / "warehouse_layout.yaml"
WORLD_OUT = PKG_ROOT / "worlds" / "warehouse.sdf"
MAP_PGM_OUT = PKG_ROOT / "maps" / "sim_warehouse.pgm"
MAP_YAML_OUT = PKG_ROOT / "maps" / "sim_warehouse.yaml"
ENDPOINTS_OUT = PKG_ROOT / "config" / "sim_endpoints.json"

FREE, OCCUPIED = 254, 0
WALL_COLOR = "0.6 0.6 0.6 1"


# --------------------------------------------------------------------------- PGM helpers

def read_pgm(data: bytes) -> tuple[int, int, bytes]:
    """Binary P5 PGM → (width, height, pixels). Comments are allowed in the header."""
    tokens: list[bytes] = []
    i = 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    if tokens[0] != b"P5" or int(tokens[3]) > 255:
        raise ValueError("only 8-bit binary PGM (P5) maps are supported")
    width, height = int(tokens[1]), int(tokens[2])
    pixels = data[i + 1:i + 1 + width * height]
    if len(pixels) != width * height:
        raise ValueError("truncated PGM")
    return width, height, pixels


def occupied_rectangles(width: int, height: int, pixels: bytes, threshold: int) -> list[tuple[int, int, int, int]]:
    """Cover occupied pixels (value < threshold) with axis-aligned rectangles (x0, y0, x1, y1), exclusive ends.

    Row runs are merged downwards while the run keeps the same x-span: a hollow rack outline becomes ~4 blocks
    instead of thousands of pixels.
    """
    rects: list[tuple[int, int, int, int]] = []
    open_runs: dict[tuple[int, int], int] = {}  # (x0, x1) -> start row
    for y in range(height + 1):
        runs: set[tuple[int, int]] = set()
        if y < height:
            row = pixels[y * width:(y + 1) * width]
            x = 0
            while x < width:
                if row[x] < threshold:
                    start = x
                    while x < width and row[x] < threshold:
                        x += 1
                    runs.add((start, x))
                else:
                    x += 1
        for span, start_row in list(open_runs.items()):
            if span not in runs:
                rects.append((span[0], start_row, span[1], y))
                del open_runs[span]
        for span in runs:
            open_runs.setdefault(span, y)
    return rects


# --------------------------------------------------------------------------- world SDF

def box_model(b: dict) -> str:
    size = f"{b['sx']} {b['sy']} {b['sz']}"
    return f"""
    <model name="{b['name']}">
      <static>true</static>
      <pose>{b['x']} {b['y']} {b['sz'] / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{size}</size></box></geometry></collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material><ambient>{b['color']}</ambient><diffuse>{b['color']}</diffuse></material>
        </visual>
      </link>
    </model>"""


def map_obstacles_model(boxes_: list[dict], color: str, name: str = "map_obstacles") -> str:
    """All map blocks in ONE static model / link (thousands of models would slow Gazebo down).
    A block may carry `z0` (bottom height) and its own `color`: walls are stacked skirt + upper part."""
    parts = []
    for i, b in enumerate(boxes_):
        c = b.get("color", color)
        size = f"{b['sx']:.4f} {b['sy']:.4f} {b['sz']:.3f}"
        pose = f"{b['x']:.4f} {b['y']:.4f} {b.get('z0', 0.0) + b['sz'] / 2:.3f} 0 0 0"
        parts.append(
            f'        <collision name="c{i}"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry></collision>\n'
            f'        <visual name="v{i}"><pose>{pose}</pose><geometry><box><size>{size}</size></box></geometry>'
            f"<material><ambient>{c}</ambient><diffuse>{c}</diffuse></material></visual>")
    body = "\n".join(parts)
    return f"""
    <model name="{name}">
      <static>true</static>
      <link name="link">
{body}
      </link>
    </model>"""


PAD_COLOR = {
    "STORAGE": "0.15 0.45 0.85 1",
    "INBOUND": "0.20 0.65 0.30 1",
    "OUTBOUND": "0.90 0.50 0.10 1",
    "CHARGING": "0.95 0.80 0.10 1",
    "PARKING": "0.45 0.45 0.50 1",
    "MAINTENANCE": "0.25 0.55 0.95 1",
    "INSPECTION": "0.20 0.80 0.50 1",
    "RECOVERY": "0.98 0.70 0.15 1",
}


def hex_rgba(color: str) -> str:
    """'#3279f9' -> '0.196 0.475 0.976 1' (SDF material)."""
    r, g, b = (int(color[i:i + 2], 16) / 255 for i in (1, 3, 5))
    return f"{r:.3f} {g:.3f} {b:.3f} 1"


def tint(color: str, amount: float, base: str = "#d6d6d2") -> str:
    """Opaque mix of `color` over the floor colour: SDF paint has no cheap transparency."""
    c = [int(color[i:i + 2], 16) for i in (1, 3, 5)]
    f = [int(base[i:i + 2], 16) for i in (1, 3, 5)]
    r, g, b = (((1 - amount) * fv + amount * cv) / 255 for cv, fv in zip(c, f))
    return f"{r:.3f} {g:.3f} {b:.3f} 1"


# Same colours as the web Map Editor (web/src/shared/map/mapStyle.ts).
WEB_ZONE_COLOR = {"KEEP_OUT": "#d93025", "SPEED": "#8430ce", "NO_STOP": "#e53935", "SINGLE_OCCUPANCY": "#f9ab00",
                  "JUNCTION": "#3279f9"}
WEB_ENDPOINT_COLOR = {"WAYPOINT": "#45474d", "CHARGER": "#0f8a99", "PARK": "#6a6a71"}
WEB_EDGE_COLOR = {"BIDIRECTIONAL": "#3279f9", "ONE_WAY": "#3279f9"}
PAD_COLOR.update({t: hex_rgba(c) for t, c in WEB_ENDPOINT_COLOR.items()})
FLOOR = "#d6d6d2"
FLOOR_JOINT = "#c4c4bf"
WALL_SKIRT = "0.30 0.32 0.35 1"
WALL_UPPER = "0.88 0.87 0.84 1"
SAFETY_YELLOW = "0.95 0.78 0.05 1"
RACK_UPRIGHT = "0.10 0.30 0.70 1"
RACK_BEAM = "0.95 0.45 0.10 1"
RACK_DECK = "0.62 0.63 0.65 1"
RACK_BASE = "0.22 0.24 0.28 1"
CARTON = "0.72 0.55 0.35 1"
BIN = "0.20 0.42 0.78 1"


def _material(color: str) -> str:
    return f"<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material>"


def visual(name: str, pose: tuple, size: tuple, color: str) -> str:
    """Visual-only element (no collision): floor paint, cartons, door panels."""
    p = " ".join(f"{v:.4f}" for v in pose)
    s = " ".join(f"{v:.4f}" for v in size)
    return (f'        <visual name="{name}"><pose>{p} 0 0 0</pose>'
            f"<geometry><box><size>{s}</size></box></geometry>{_material(color)}</visual>\n")


def solid(name: str, pose: tuple, size: tuple, color: str) -> str:
    """Collision + visual element."""
    p = " ".join(f"{v:.4f}" for v in pose)
    s = " ".join(f"{v:.4f}" for v in size)
    return (f'        <collision name="{name}_c"><pose>{p} 0 0 0</pose><geometry><box><size>{s}</size></box></geometry></collision>\n'
            + visual(name, pose, size, color))


def static_model(name: str, elements: str, x: float = 0.0, y: float = 0.0) -> str:
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
{elements}      </link>
    </model>"""


def rack_model(shelf: dict) -> str:
    """Pallet-style racking: solid kick plate up to lidar height (so the scan sees the same rectangle as the
    map), blue uprights, orange beams, grey decks and cartons / bins on every level."""
    sx, sy, sz = shelf["sx"], shelf["sy"], shelf["sz"]
    levels = int(shelf.get("levels", 3))
    base_h = 0.45  # above the lidar plane: the map footprint == what the lidar sees
    # solid_base false (source map): the base is drawn from the map cells instead, so nothing of the rack
    # itself reaches below base_h and the lidar keeps seeing exactly the uploaded map.
    solid_base = shelf.get("solid_base", True)
    e = solid("base", (0, 0, base_h / 2), (sx, sy, base_h), RACK_BASE) if solid_base else ""
    post_z0 = 0.0 if solid_base else base_h

    long_x = sx >= sy
    length, depth = (sx, sy) if long_x else (sy, sx)
    post = 0.07
    bays = max(1, round(length / 1.2))
    for i in range(bays + 1):
        a = -length / 2 + post / 2 + i * (length - post) / bays
        for side in (-1, 1):
            b = side * (depth / 2 - post / 2)
            x, y = (a, b) if long_x else (b, a)
            e += solid(f"upright_{i}_{side}", (x, y, (post_z0 + sz) / 2), (post, post, sz - post_z0), RACK_UPRIGHT)

    deck_t = 0.03
    pitch = (sz - base_h) / levels
    slot = 0.45
    for level in range(levels):
        z = base_h + level * pitch
        size = (sx, sy, deck_t)
        e += solid(f"deck_{level}", (0, 0, z + deck_t / 2), size, RACK_DECK)
        for side in (-1, 1):
            b = side * (depth / 2 - 0.02)
            pose = (0, b, z + 0.06) if long_x else (b, 0, z + 0.06)
            beam = (length, 0.04, 0.08) if long_x else (0.04, length, 0.08)
            e += visual(f"beam_{level}_{side}", pose, beam, RACK_BEAM)
        # Goods: one item per slot, a deterministic gap pattern so racks do not look copy-pasted.
        slots = int((length - 0.1) // slot)
        for k in range(slots):
            if (k + level + len(shelf["code"])) % 4 == 3:
                continue
            a = -length / 2 + 0.05 + slot * (k + 0.5)
            is_bin = (k + level) % 3 == 0
            h = 0.22 if is_bin else min(0.32, pitch - 0.12)
            w, d = 0.38, depth * 0.75
            x, y = (a, 0) if long_x else (0, a)
            item = (w, d, h) if long_x else (d, w, h)
            e += visual(f"goods_{level}_{k}", (x, y, z + deck_t + h / 2), item, BIN if is_bin else CARTON)
    return static_model(f"rack_{shelf['code']}", e, shelf["x"], shelf["y"])


def charger_model(o: dict) -> str:
    """Charging dock: a low solid base (= map footprint), a tower and a green status strip facing the room."""
    sx, sy, sz = o["sx"], o["sy"], o["sz"]
    yaw = o.get("front_yaw", -math.pi / 2)
    fx, fy = math.cos(yaw), math.sin(yaw)
    e = solid("base", (0, 0, sz / 2), (sx, sy, sz), "0.20 0.20 0.22 1")
    tower = (sx * 0.6, 0.08, 0.9) if abs(fy) > abs(fx) else (0.08, sy * 0.6, 0.9)
    e += visual("tower", (-fx * (sy / 2 - 0.05), -fy * (sy / 2 - 0.05), sz + 0.45), tower, "0.25 0.25 0.28 1")
    strip = (sx * 0.4, 0.01, 0.06) if abs(fy) > abs(fx) else (0.01, sy * 0.4, 0.06)
    e += visual("led", (fx * (sy / 2 + 0.006), fy * (sy / 2 + 0.006), sz * 0.6), strip, "0.10 0.85 0.30 1")
    e += visual("contacts", (fx * (sy / 2 + 0.004), fy * (sy / 2 + 0.004), 0.12), strip[:2] + (0.03,), "0.80 0.70 0.20 1")
    return static_model(o["name"], e, o["x"], o["y"])


def floor_zone_model(zone: dict) -> str:
    """Painted floor area: a tinted fill (optional) and a safety-yellow border. Visual only, 2 mm high —
    far below the lidar plane, so it never reaches the map."""
    x0, y0, x1, y1 = zone["x0"], zone["y0"], zone["x1"], zone["y1"]
    cx, cy, w, h = (x0 + x1) / 2, (y0 + y1) / 2, x1 - x0, y1 - y0
    line = zone.get("line", 0.06)
    color = zone.get("line_color", SAFETY_YELLOW)
    z = 0.003 + zone.get("lift", 0.0)  # docking squares sit 2 mm above zone paint (no z-fighting)
    e = ""
    if "fill" in zone:
        # `lift` stacks overlapping paint (lane floor < zone fill < docking pad) without z-fighting.
        e += visual("fill", (0, 0, 0.001 + zone.get("lift", 0.0)), (w, h, 0.002), zone["fill"])
    e += visual("n", (0, h / 2 - line / 2, z), (w, line, 0.002), color)
    e += visual("s", (0, -h / 2 + line / 2, z), (w, line, 0.002), color)
    e += visual("w", (-w / 2 + line / 2, 0, z), (line, h - 2 * line, 0.002), color)
    e += visual("e", (w / 2 - line / 2, 0, z), (line, h - 2 * line, 0.002), color)
    return static_model(f"floor_{zone['name']}", e, cx, cy)


def door_model(door: dict, size: dict) -> str:
    """Roll-up dock door painted on the inner face of a wall (visual only, 2 cm deep)."""
    width, height = door.get("width", 1.8), door.get("height", 2.0)
    wall = door["wall"]
    inner = {"west": (0.012, door["at"]), "east": (size["x"] - 0.012, door["at"]),
             "south": (door["at"], 0.012), "north": (door["at"], size["y"] - 0.012)}[wall]
    along_y = wall in ("west", "east")

    def box(a: float, z: float, w: float, h: float, t: float = 0.02) -> tuple:
        return ((0, a, z), (t, w, h)) if along_y else ((a, 0, z), (w, t, h))

    e = ""
    pose, s = box(0, height / 2, width, height)
    e += visual("panel", pose, s, "0.55 0.57 0.60 1")
    for i in range(1, int(height / 0.2)):
        pose, s = box(0, i * 0.2, width, 0.025, 0.03)
        e += visual(f"slat_{i}", pose, s, "0.42 0.44 0.47 1")
    for side in (-1, 1):
        pose, s = box(side * (width / 2 + 0.06), (height + 0.15) / 2, 0.12, height + 0.15, 0.04)
        e += visual(f"frame_{side}", pose, s, SAFETY_YELLOW)
    pose, s = box(0, height + 0.1, width + 0.24, 0.1, 0.04)
    e += visual("header", pose, s, SAFETY_YELLOW)
    return static_model(f"door_{door['name']}", e, *inner)


def marker_model(code: str, x: float, y: float, endpoint_type: str = "STORAGE", size: float = 0.6) -> str:
    """Painted docking square on the floor at an endpoint (visual only): a coloured outline by type."""
    color = PAD_COLOR.get(endpoint_type, PAD_COLOR["STORAGE"])
    half = size / 2
    return floor_zone_model({"name": f"pad_{code}", "x0": x - half, "y0": y - half, "x1": x + half, "y1": y + half,
                             "line": min(0.05, size / 8), "line_color": color, "lift": 0.004})


def world_sdf(name: str, models: str, center: tuple[float, float], span: float) -> str:
    cx, cy = center
    cam_back, cam_up = 0.6 * span, 0.7 * span
    return f"""<?xml version="1.0"?>
<!-- GENERATED by scripts/generate_warehouse.py from the warehouse layout. Do not edit. -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <gui fullscreen="0">
      <camera name="user_camera">
        <pose>{cx:.2f} {cy - cam_back:.2f} {cam_up:.2f} 0 0.85 1.5708</pose>
      </camera>
    </gui>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.78 0.82 0.86 1</background>
      <grid>false</grid>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <light type="directional" name="fill">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.35 0.35 0.38 1</diffuse>
      <specular>0 0 0 1</specular>
      <direction>0.4 -0.3 -0.8</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material><ambient>0.66 0.67 0.66 1</ambient><diffuse>0.66 0.67 0.66 1</diffuse></material>
        </visual>
      </link>
    </model>
{models}
  </world>
</sdf>
"""


# --------------------------------------------------------------------------- source-map layout

def paint_segment(name: str, a: tuple, b: tuple, width: float, z: float, color: str, extend: bool = True) -> str:
    """Floor paint stripe from a to b (visual only, 1 mm thick). `extend`: lengthened by width/2 at both ends so
    polylines and outlines close at the corners; off for a lane band, whose ends are square at the lane ends."""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    if length < 1e-6:
        return ""
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    size_x = length + width if extend else length
    return (f'        <visual name="{name}"><pose>{(a[0] + b[0]) / 2:.4f} {(a[1] + b[1]) / 2:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>'
            f"<geometry><box><size>{size_x:.4f} {width:.4f} 0.0010</size></box></geometry>{_material(color)}</visual>\n")


def floor_model(x0: float, y0: float, size_x: float, size_y: float, grid: float = 0.5) -> str:
    """Concrete floor over the map area with a joint every `grid` metres, aligned to map (0, 0): the same
    0.5 m grid as the map plot, so a coordinate read in Gazebo matches the web."""
    e = visual("slab", (x0 + size_x / 2, y0 + size_y / 2, 0.0005), (size_x, size_y, 0.001), hex_rgba(FLOOR))
    joint = hex_rgba(FLOOR_JOINT)
    for i in range(math.ceil(x0 / grid), math.floor((x0 + size_x) / grid) + 1):
        e += paint_segment(f"jx{i}", (i * grid, y0), (i * grid, y0 + size_y), 0.01, 0.0015, joint)
    for j in range(math.ceil(y0 / grid), math.floor((y0 + size_y) / grid) + 1):
        e += paint_segment(f"jy{j}", (x0, j * grid), (x0 + size_x, j * grid), 0.01, 0.0015, joint)
    return static_model("floor", e)


def web_overlay_models(overlay: dict) -> str:
    """Zones, lanes (Edges) and Endpoints of the web Map Editor painted on the floor, web colours.
    Everything is visual only and a few mm high: far below the lidar plane, it never changes the scan."""
    e = ""
    order = ["JUNCTION", "SPEED", "NO_STOP", "SINGLE_OCCUPANCY", "KEEP_OUT"]  # web ZONE_ORDER: later paints on top
    zones = sorted((z for z in overlay.get("zones", []) if z.get("isActive", True)),
                   key=lambda z: order.index(z["zoneType"]) if z["zoneType"] in order else 0)
    for i, z in enumerate(zones):
        ring = [tuple(p[:2]) for p in z["geometry"]["coordinates"][0]]
        color = WEB_ZONE_COLOR.get(z["zoneType"], "#6a6a71")
        xs, ys = [p[0] for p in ring], [p[1] for p in ring]
        corners = {(round(x, 4), round(y, 4)) for x, y in ring}
        is_rect = len(corners) == 4 and all(
            (round(x, 4) in (round(min(xs), 4), round(max(xs), 4))) and (round(y, 4) in (round(min(ys), 4), round(max(ys), 4)))
            for x, y in corners)
        lift = i * 0.0002
        if is_rect:  # axis-aligned rectangle: tinted fill like the web (other polygons: outline only)
            amount = 0.35 if z["zoneType"] == "KEEP_OUT" else 0.18
            e += visual(f"zone_{i}_fill", ((min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2, 0.0025 + lift),
                        (max(xs) - min(xs), max(ys) - min(ys), 0.001), tint(color, amount, FLOOR))
        for k in range(len(ring) - 1):
            e += paint_segment(f"zone_{i}_e{k}", ring[k], ring[k + 1], 0.025, 0.0040 + lift, hex_rgba(color))

    # Lanes like the web (laneGlyphs.ts): a tinted band as wide as the lane with chevrons; one-way = one row in the
    # middle, two-way = a dashed centre line and two opposite rows (forward on the right of the drawing direction).
    for i, edge in enumerate(overlay.get("edges", [])):
        pts = [tuple(p[:2]) for p in edge["geometry"]["coordinates"]]
        hex_color = WEB_EDGE_COLOR.get(edge["direction"], "#3279f9") if edge.get("isActive", True) else "#a3adb9"
        color = hex_rgba(hex_color)
        width = float(edge.get("widthMeters") or 1.0)
        one_way = edge["direction"] == "ONE_WAY"
        for k in range(len(pts) - 1):
            (ax, ay), (bx, by) = pts[k], pts[k + 1]
            length = math.hypot(bx - ax, by - ay)
            if length < 1e-6:
                continue
            ux, uy = (bx - ax) / length, (by - ay) / length
            rx, ry = uy, -ux  # right of the drawing direction (map frame, y up)
            e += paint_segment(f"lane_{i}_{k}_band", pts[k], pts[k + 1], width, 0.0050, tint(hex_color, 0.30, FLOOR),
                               extend=False)
            rows = [(0.0, 1, width * 0.55, width * 1.6)] if one_way else \
                [(width / 4, 1, width * 0.3, width * 1.1), (-width / 4, -1, width * 0.3, width * 1.1)]
            if not one_way:
                for n in range(int(length / 0.12)):
                    if n % 2 == 0:
                        s0, s1 = n * 0.12, min(length, n * 0.12 + 0.07)
                        e += paint_segment(f"lane_{i}_{k}_d{n}", (ax + ux * s0, ay + uy * s0), (ax + ux * s1, ay + uy * s1),
                                           0.012, 0.0058, color)
            for r, (off, sign, size, spacing) in enumerate(rows):
                stations = [length / 2] if length < spacing else \
                    [spacing / 2 + n * spacing for n in range(int((length - spacing / 2) / spacing) + 1)]
                for n, s in enumerate(stations):
                    cx, cy = ax + ux * s + rx * off, ay + uy * s + ry * off
                    tip = (cx + sign * ux * size * 0.18, cy + sign * uy * size * 0.18)
                    for side in (-1, 1):
                        back = (tip[0] - sign * ux * size * 0.5 + rx * side * size * 0.5,
                                tip[1] - sign * uy * size * 0.5 + ry * side * size * 0.5)
                        e += paint_segment(f"lane_{i}_{k}_c{r}_{n}{side}", back, tip, max(0.015, size * 0.12), 0.0062, color)

    for i, ep in enumerate(overlay.get("endpoints", [])):
        color = hex_rgba(WEB_ENDPOINT_COLOR.get(ep["endpointType"], "#45474d") if ep.get("isEnabled", True) else "#a3adb9")
        x, y, half = ep["x"], ep["y"], 0.12
        square = [(x - half, y - half), (x + half, y - half), (x + half, y + half), (x - half, y + half), (x - half, y - half)]
        for k in range(4):
            e += paint_segment(f"ep_{i}_{k}", square[k], square[k + 1], 0.02, 0.0070, color)
        # Heading tick: the docking yaw set on the web.
        tip = (x + 0.12 * math.cos(ep["yaw"]), y + 0.12 * math.sin(ep["yaw"]))
        e += paint_segment(f"ep_{i}_yaw", (x, y), tip, 0.025, 0.0075, color)
    return static_model("web_overlay", e) if e else ""


def render_from_source_map(layout: dict, layout_dir: Path) -> dict[Path, bytes]:
    map_yaml_path = (PKG_ROOT / layout["source_map"]).resolve()
    meta = yaml.safe_load(map_yaml_path.read_text(encoding="utf-8"))
    pgm_bytes = (map_yaml_path.parent / meta["image"]).read_bytes()
    width, height, pixels = read_pgm(pgm_bytes)

    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    if int(meta.get("negate", 0)) != 0:
        raise ValueError("negate: 1 maps are not supported")
    if len(meta["origin"]) > 2 and abs(float(meta["origin"][2])) > 1e-9:
        raise ValueError("maps with a rotated origin (yaw != 0) are not supported")
    # map_server (trinary, negate 0): occupancy = (255 - value) / 255 > occupied_thresh
    threshold = math.ceil(255 * (1 - occupied_thresh))
    wall_h = float(layout.get("wall_height", layout.get("obstacle_height", 1.0)))
    skirt_h = min(0.30, wall_h)  # dark skirting up to just above the lidar plane, light wall above

    # Split the occupied cells: inside a `shelves` rectangle -> that rack, otherwise -> wall. Only the colour /
    # the detail above differ: every occupied cell stays a solid block from the floor up.
    shelves = layout.get("shelves", [])
    wall_px = bytearray(pixels)
    shelf_px = [bytearray([FREE]) * len(pixels) for _ in shelves]
    for row in range(height):
        cy = oy + (height - row - 0.5) * res
        for col in range(width):
            i = row * width + col
            if pixels[i] >= threshold:
                continue
            cx = ox + (col + 0.5) * res
            for k, s in enumerate(shelves):
                if s["x0"] <= cx <= s["x1"] and s["y0"] <= cy <= s["y1"]:
                    wall_px[i], shelf_px[k][i] = FREE, OCCUPIED
                    break

    def cell_blocks(px: bytes) -> list[dict]:
        out = []
        for x0, y0, x1, y1 in occupied_rectangles(width, height, px, threshold):
            # PGM row 0 is the top (max y)
            wx0, wx1 = ox + x0 * res, ox + x1 * res
            wy0, wy1 = oy + (height - y1) * res, oy + (height - y0) * res
            out.append({"x": (wx0 + wx1) / 2, "y": (wy0 + wy1) / 2, "sx": wx1 - wx0, "sy": wy1 - wy0})
        return out

    walls = []
    for b in cell_blocks(wall_px):
        walls.append({**b, "sz": skirt_h, "color": WALL_SKIRT})
        if wall_h > skirt_h:
            walls.append({**b, "z0": skirt_h, "sz": wall_h - skirt_h, "color": WALL_UPPER})
    models = map_obstacles_model(walls, WALL_COLOR, "map_walls")
    n_blocks = len(walls)
    for k, s in enumerate(shelves):
        base = [{**b, "sz": 0.45, "color": RACK_BASE} for b in cell_blocks(shelf_px[k])]
        if not base:
            print(f"warning: shelf {s['code']} has no occupied cell inside its rectangle", file=sys.stderr)
        n_blocks += len(base)
        models += map_obstacles_model(base, RACK_BASE, f"rack_base_{s['code']}")
        models += rack_model({"code": s["code"], "x": (s["x0"] + s["x1"]) / 2, "y": (s["y0"] + s["y1"]) / 2,
                              "sx": s["x1"] - s["x0"], "sy": s["y1"] - s["y0"], "sz": s.get("sz", 1.0),
                              "levels": s.get("levels", 2), "solid_base": False})

    size_x, size_y = width * res, height * res
    models += floor_model(ox, oy, size_x, size_y)
    endpoints = list(layout.get("endpoints", []))
    models += "".join(marker_model(e["code"], e["x"], e["y"]) for e in endpoints)

    overlay_rel = layout.get("web", {}).get("overlay")
    overlay = None
    if overlay_rel:
        overlay_path = PKG_ROOT / overlay_rel
        if overlay_path.is_file():
            overlay = json.loads(overlay_path.read_text(encoding="utf-8"))
            models += web_overlay_models(overlay)
            endpoints += [{"code": e["code"], "name": e["name"], "type": e["endpointType"],
                           "x": e["x"], "y": e["y"], "yaw": e["yaw"]}
                          for e in overlay.get("endpoints", []) if e.get("isEnabled", True)]
        else:
            print(f"note: {overlay_rel} not found (run scripts/sync_web_map.py): no web paint")
    world = world_sdf(layout["name"], models, (ox + size_x / 2, oy + size_y / 2), max(size_x, size_y))

    map_meta = {
        "image": MAP_PGM_OUT.name,
        "mode": "trinary",
        "resolution": res,
        "origin": [ox, oy, float(meta["origin"][2]) if len(meta["origin"]) > 2 else 0.0],
        "negate": 0,
        "occupied_thresh": occupied_thresh,
        "free_thresh": float(meta.get("free_thresh", 0.25)),
    }
    print(f"source map {map_yaml_path.name}: {width}x{height} px @ {res} m -> {n_blocks} Gazebo blocks, "
          f"{len(shelves)} racks" + (f", web overlay {overlay.get('mapVersionCode')}: {len(overlay.get('zones', []))} zones, "
                                     f"{len(overlay.get('edges', []))} lanes, {len(overlay.get('endpoints', []))} endpoints"
                                     if overlay else ""))
    return {
        WORLD_OUT: world.encode(),
        MAP_PGM_OUT: pgm_bytes,
        MAP_YAML_OUT: yaml.safe_dump(map_meta, sort_keys=False).encode(),
        ENDPOINTS_OUT: endpoints_json(layout["name"], layout["spawn"], [
            {"code": e["code"], "name": e["name"], "endpointType": e.get("type", "STORAGE"),
             "x": e["x"], "y": e["y"], "yaw": e["yaw"]}
            for e in endpoints]),
    }


# --------------------------------------------------------------------------- hand-made layout

def boxes(layout: dict) -> list[dict]:
    """Every solid box in the world: walls, shelves, obstacles (axis-aligned)."""
    sx, sy = layout["size"]["x"], layout["size"]["y"]
    t, h = layout["wall"]["thickness"], layout["wall"]["height"]
    result = [
        {"name": "wall_south", "x": sx / 2, "y": -t / 2, "sx": sx + 2 * t, "sy": t, "sz": h, "color": WALL_COLOR},
        {"name": "wall_north", "x": sx / 2, "y": sy + t / 2, "sx": sx + 2 * t, "sy": t, "sz": h, "color": WALL_COLOR},
        {"name": "wall_west", "x": -t / 2, "y": sy / 2, "sx": t, "sy": sy, "sz": h, "color": WALL_COLOR},
        {"name": "wall_east", "x": sx + t / 2, "y": sy / 2, "sx": t, "sy": sy, "sz": h, "color": WALL_COLOR},
    ]
    for shelf in layout["shelves"]:
        result.append({**shelf, "name": f"shelf_{shelf['code']}", "color": "0.85 0.55 0.2 1"})
    for obstacle in layout.get("obstacles", []):
        result.append({**obstacle, "color": "0.4 0.3 0.2 1"})
    return result


def occupancy_map(layout: dict) -> tuple[bytes, dict]:
    res = layout["map"]["resolution"]
    border = layout["wall"]["thickness"] + layout["map"]["margin"]
    origin_x, origin_y = -border, -border
    width = math.ceil((layout["size"]["x"] + 2 * border) / res)
    height = math.ceil((layout["size"]["y"] + 2 * border) / res)

    grid = bytearray([FREE]) * (width * height)
    for b in boxes(layout):
        x0 = math.floor((b["x"] - b["sx"] / 2 - origin_x) / res)
        x1 = math.ceil((b["x"] + b["sx"] / 2 - origin_x) / res)
        y0 = math.floor((b["y"] - b["sy"] / 2 - origin_y) / res)
        y1 = math.ceil((b["y"] + b["sy"] / 2 - origin_y) / res)
        for py in range(max(0, y0), min(height, y1)):
            row = height - 1 - py  # PGM row 0 is the top (max y)
            for px in range(max(0, x0), min(width, x1)):
                grid[row * width + px] = OCCUPIED

    pgm = f"P5\n# {layout['name']} generated from warehouse layout\n{width} {height}\n255\n".encode() + bytes(grid)
    meta = {
        "image": MAP_PGM_OUT.name,
        "mode": "trinary",
        "resolution": res,
        "origin": [round(origin_x, 4), round(origin_y, 4), 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.25,
    }
    return pgm, meta


def render_hand_made(layout: dict) -> dict[Path, bytes]:
    # The map is rasterised from boxes(); the world draws the same footprints with more detail on top.
    walls = [b for b in boxes(layout) if b["name"].startswith("wall_")]
    models = "".join(box_model({**b, "color": layout["wall"].get("color", WALL_COLOR)}) for b in walls)
    models += "".join(rack_model(s) for s in layout["shelves"])
    for obstacle in layout.get("obstacles", []):
        models += charger_model(obstacle) if obstacle.get("style") == "charger" else box_model(
            {**obstacle, "color": "0.4 0.3 0.2 1"})
    models += "".join(floor_zone_model(z) for z in layout.get("floor_zones", []))
    models += "".join(door_model(d, layout["size"]) for d in layout.get("doors", []))
    pad = layout.get("pad_size", 0.6)
    # A shelf without `endpoint` is only an obstacle: its docking stop is set up later (web Map Editor).
    docked = [s for s in layout["shelves"] if "endpoint" in s]
    models += "".join(marker_model(s["code"], s["endpoint"]["x"], s["endpoint"]["y"], size=pad) for s in docked)
    models += "".join(marker_model(e["code"], e["x"], e["y"], e["type"], pad) for e in layout.get("extra_endpoints", []))
    size_x, size_y = layout["size"]["x"], layout["size"]["y"]
    pgm, meta = occupancy_map(layout)
    endpoints = [
        {"code": s["code"], "name": s["name"], "endpointType": "STORAGE",
         "x": s["endpoint"]["x"], "y": s["endpoint"]["y"], "yaw": s["endpoint"]["yaw"]}
        for s in docked
    ] + [
        {"code": e["code"], "name": e["name"], "endpointType": e["type"], "x": e["x"], "y": e["y"], "yaw": e["yaw"]}
        for e in layout.get("extra_endpoints", [])
    ]
    # Painted areas that carry a `zone:` block become backend Navigation Zones (rectangles in map metres).
    zones = [
        {"name": z["zone"]["name"], "zoneType": z["zone"]["type"], "capacity": z["zone"].get("capacity", 1),
         "maxSpeed": z["zone"].get("max_speed"),
         "polygon": [[z["x0"], z["y0"]], [z["x1"], z["y0"]], [z["x1"], z["y1"]], [z["x0"], z["y1"]], [z["x0"], z["y0"]]]}
        for z in layout.get("floor_zones", []) if "zone" in z
    ]
    return {
        WORLD_OUT: world_sdf(layout["name"], models, (size_x / 2, size_y / 2), max(size_x, size_y) * 0.75).encode(),
        MAP_PGM_OUT: pgm,
        MAP_YAML_OUT: yaml.safe_dump(meta, sort_keys=False).encode(),
        ENDPOINTS_OUT: endpoints_json(layout["name"], layout["spawn"], endpoints, zones),
    }


# --------------------------------------------------------------------------- main

def endpoints_json(name: str, spawn: dict, endpoints: list[dict], zones: list[dict] | None = None) -> bytes:
    return (json.dumps({"mapName": name, "frame": "map", "spawn": spawn, "endpoints": endpoints, "zones": zones or []},
                       indent=2, ensure_ascii=False) + "\n").encode("utf-8")


def render(layout_path: Path) -> dict[Path, bytes]:
    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    if "source_map" in layout:
        return render_from_source_map(layout, layout_path.parent)
    return render_hand_made(layout)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--layout", type=Path, default=LAYOUT, help="layout yaml (default: config/warehouse_layout.yaml)")
    parser.add_argument("--check", action="store_true", help="exit 1 if generated files are out of date")
    args = parser.parse_args()

    layout_path = args.layout if args.layout.is_absolute() else (Path.cwd() / args.layout)
    if not layout_path.is_file():
        layout_path = PKG_ROOT / args.layout
    outputs = render(layout_path)

    if args.check:
        stale = [p for p, data in outputs.items() if not p.is_file() or p.read_bytes() != data]
        for p in stale:
            print(f"stale: {p.relative_to(PKG_ROOT)}")
        return 1 if stale else 0

    for path, data in outputs.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        print(f"wrote {path.relative_to(PKG_ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
