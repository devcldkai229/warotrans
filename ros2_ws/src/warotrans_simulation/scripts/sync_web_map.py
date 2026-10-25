#!/usr/bin/env python3
"""Sync the simulated warehouse with a map version of the web (backend): map image + Map Editor data.

For a `source_map` layout with a `web:` block (config/warehouse_layout.yaml):
  1. GET /api/navigation/maps/{id}/detail                 -> map metadata, Endpoints, Edges (lanes), Zones
  2. download the uploaded map.pgm / map.yaml (/map-assets) into `source_map` (byte-identical pgm; only the
     yaml `image:` is renamed). Nav2 / AMCL and the Gazebo walls are built from the SAME file as the web.
  3. write `web.overlay` (Endpoints / Edges / Zones as JSON) -> painted on the Gazebo floor
  4. run generate_warehouse.py -> worlds/warehouse.sdf, maps/sim_warehouse.*, config/sim_endpoints.json

    python3 scripts/sync_web_map.py --user admin --password '...'          # ids from the layout
    python3 scripts/sync_web_map.py --map-id <guid> --user admin --password '...'
    WAROTRANS_TOKEN=<jwt> python3 scripts/sync_web_map.py --backend http://localhost:5090

Run it again after editing the map on the web; restart the simulation to load the new world.
Standard library + PyYAML only.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path

import yaml

PKG_ROOT = Path(__file__).resolve().parent.parent
LAYOUT = PKG_ROOT / "config" / "warehouse_layout.yaml"


def http(url: str, token: str | None = None, body: dict | None = None) -> bytes:
    headers = {"Content-Type": "application/json"} if body is not None else {}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    data = json.dumps(body).encode() if body is not None else None
    request = urllib.request.Request(url, data=data, headers=headers, method="POST" if data else "GET")
    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            return response.read()
    except urllib.error.HTTPError as err:
        raise SystemExit(f"{err.code} {url}: {err.read().decode(errors='replace')[:300]}") from None
    except urllib.error.URLError as err:
        raise SystemExit(f"backend not reachable at {url}: {err.reason}") from None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--layout", type=Path, default=LAYOUT)
    parser.add_argument("--backend", default=None, help="backend URL (default: layout web.backend or http://localhost:5090)")
    parser.add_argument("--map-id", default=None, help="MapVersion id (default: layout web.map_version_id)")
    parser.add_argument("--user", default=os.environ.get("WAROTRANS_USER"))
    parser.add_argument("--password", default=os.environ.get("WAROTRANS_PASSWORD"))
    parser.add_argument("--no-generate", action="store_true", help="only download, do not run generate_warehouse.py")
    args = parser.parse_args()

    layout_path = args.layout if args.layout.is_absolute() else (Path.cwd() / args.layout)
    if not layout_path.is_file():
        layout_path = PKG_ROOT / args.layout
    layout = yaml.safe_load(layout_path.read_text(encoding="utf-8"))
    web = layout.get("web") or {}
    if "source_map" not in layout or "overlay" not in web:
        raise SystemExit(f"{layout_path}: needs `source_map` and `web.overlay` (see config/warehouse_layout.yaml)")
    base = (args.backend or web.get("backend") or "http://localhost:5090").rstrip("/")
    map_id = args.map_id or web.get("map_version_id")
    if not map_id:
        raise SystemExit("no map id: pass --map-id or set web.map_version_id in the layout")

    token = os.environ.get("WAROTRANS_TOKEN")
    if not token:
        if not (args.user and args.password):
            raise SystemExit("auth: set WAROTRANS_TOKEN or pass --user / --password")
        login = json.loads(http(f"{base}/api/identity/login", body={"username": args.user, "password": args.password}))
        token = login.get("accessToken")
        if not token:
            raise SystemExit(f"login failed: {login}")

    detail = json.loads(http(f"{base}/api/navigation/maps/{map_id}/detail", token))
    if abs(detail.get("originYaw", 0.0)) > 1e-9 or detail.get("negate"):
        raise SystemExit(f"{detail['code']}: rotated (originYaw) or negated maps are not supported by the generator")

    # Map files exactly as uploaded on the web.
    map_yaml_path = PKG_ROOT / layout["source_map"]
    pgm_path = map_yaml_path.with_suffix(".pgm")
    def asset(uri: str) -> bytes:  # local /map-assets/... or an absolute (S3) URL
        return http(uri if uri.startswith("http") else base + uri)

    pgm = asset(detail["mapUri"])
    meta = yaml.safe_load(asset(detail["mapYamlUri"]).decode("utf-8"))
    meta["image"] = pgm_path.name
    map_yaml = yaml.safe_dump(meta, sort_keys=False)
    changed = not pgm_path.is_file() or pgm_path.read_bytes() != pgm
    map_yaml_path.parent.mkdir(parents=True, exist_ok=True)
    pgm_path.write_bytes(pgm)
    map_yaml_path.write_text(map_yaml, encoding="utf-8")

    overlay = {
        "mapVersionId": detail["id"],
        "mapVersionCode": detail["code"],
        "status": detail["status"],
        "endpoints": [{k: e[k] for k in ("code", "name", "endpointType", "x", "y", "yaw", "isEnabled")}
                      for e in detail["endpoints"]],
        "edges": [{k: e[k] for k in ("code", "geometry", "widthMeters", "direction", "maxSpeed", "isActive")}
                  for e in detail["edges"]],
        "zones": [{k: z[k] for k in ("code", "name", "zoneType", "geometry", "maxSpeed", "isActive")} for z in detail["zones"]],
    }
    overlay_path = PKG_ROOT / web["overlay"]
    overlay_path.parent.mkdir(parents=True, exist_ok=True)
    overlay_path.write_text(json.dumps(overlay, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(f"{detail['code']} ({detail['status']}): map {'CHANGED' if changed else 'unchanged'} "
          f"{detail['imageWidthPixels']}x{detail['imageHeightPixels']} px @ {detail['resolution']} m, "
          f"{len(overlay['endpoints'])} endpoints, {len(overlay['edges'])} lanes, {len(overlay['zones'])} zones", flush=True)
    if detail["code"] != web.get("map_version_code"):
        print(f"note: layout web.map_version_code is {web.get('map_version_code')!r}, the backend map is {detail['code']!r}; "
              "update the layout and start the simulation with map_version_code:=" + detail["code"])
    if args.no_generate:
        return 0
    return subprocess.call([sys.executable, str(PKG_ROOT / "scripts" / "generate_warehouse.py"), "--layout", str(layout_path)])


if __name__ == "__main__":
    sys.exit(main())
