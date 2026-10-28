#!/usr/bin/env python3
"""Fail-closed SIM init checks before Traffic Active / MOVE.

Checks (offline + optional live):
  1. Clear stale AuthorityStore (/tmp/warotrans_authority_state.json)
  2. Layout spawn_endpoint_code resolves to PARK and matches map yaml meta
  3. Optional: MQTT/backend robot LOCALIZED and pose within PARK bay radius
  4. Optional: map_version_code matches expected

Exit 0 = OK to start jobs. Exit 1 = blocked (print reasons).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path

import yaml

# co-located
sys.path.insert(0, str(Path(__file__).resolve().parent))
from spawn_pose import resolve_spawn  # noqa: E402


def _load_yaml(path: Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def clear_stale_authority(path: Path, *, dry_run: bool) -> str:
    if not path.exists():
        return f"authority state absent ({path}) — OK"
    if dry_run:
        return f"would remove stale authority {path}"
    path.unlink()
    return f"removed stale authority {path}"


def pose_in_bay(x: float, y: float, park: dict, radius_m: float) -> bool:
    return math.hypot(x - float(park["x"]), y - float(park["y"])) <= radius_m


def fetch_robot(backend: str, robot_code: str, token: str | None) -> dict | None:
    url = f"{backend.rstrip('/')}/api/fleet/robots"
    req = urllib.request.Request(url, headers={"Accept": "application/json"})
    if token:
        req.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(req, timeout=5) as resp:
        body = json.loads(resp.read().decode("utf-8"))
    items = body if isinstance(body, list) else body.get("items") or body.get("data") or []
    for r in items:
        if str(r.get("code") or "") == robot_code:
            return r
    return None


def main() -> int:
    root = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--layout", type=Path, default=root / "config" / "warehouse_layout.yaml")
    ap.add_argument("--map", type=Path, default=root / "maps" / "sim_warehouse.yaml")
    ap.add_argument("--endpoint", default="")
    ap.add_argument("--expected-map-version", default="MAP-WHSIM-V018")
    ap.add_argument("--robot-code", default="RBT-SIM-01")
    ap.add_argument("--backend", default=os.environ.get("WAROTRANS_BACKEND", ""))
    ap.add_argument("--token", default=os.environ.get("WAROTRANS_TOKEN", ""))
    ap.add_argument("--park-radius-m", type=float, default=0.35)
    ap.add_argument(
        "--authority-state",
        type=Path,
        default=Path(os.environ.get("WAROTRANS_AUTHORITY_STATE", "/tmp/warotrans_authority_state.json")),
    )
    ap.add_argument("--skip-live", action="store_true", help="Only offline layout/map/authority checks")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    failures: list[str] = []
    notes: list[str] = []

    notes.append(clear_stale_authority(args.authority_state, dry_run=args.dry_run))

    layout = _load_yaml(args.layout)
    web = layout.get("web") or {}
    code = (args.endpoint or layout.get("spawn_endpoint_code") or "").strip()
    if not code:
        failures.append("layout missing spawn_endpoint_code (refuse hard 0,0 default)")
    else:
        try:
            spawn = resolve_spawn(
                args.layout,
                endpoint_code_override=code,
                map_yaml=args.map,
                require_clearance=True,
            )
            notes.append(
                f"spawn OK {code} -> ({spawn['x']:.3f},{spawn['y']:.3f},{spawn['yaw']:.3f}) via {spawn['source']}"
            )
            if not str(code).startswith("EP-PARK"):
                failures.append(f"spawn endpoint {code} is not a PARK bay")
        except Exception as exc:
            failures.append(f"spawn resolve failed: {exc}")
            spawn = None

    map_meta = _load_yaml(args.map)
    origin = map_meta.get("origin") or []
    if abs(float(map_meta.get("resolution") or 0) - 0.05) > 1e-9:
        failures.append(f"map resolution mismatch: {map_meta.get('resolution')}")
    if web.get("map_version_code") and args.expected_map_version:
        if web["map_version_code"] != args.expected_map_version:
            failures.append(
                f"layout map_version_code {web['map_version_code']} != expected {args.expected_map_version}"
            )
        else:
            notes.append(f"map_version_code match {args.expected_map_version}")

    if args.backend and not args.skip_live:
        try:
            robot = fetch_robot(args.backend, args.robot_code, args.token or None)
            if robot is None:
                failures.append(f"robot {args.robot_code} not found on backend")
            else:
                status = str(robot.get("status") or robot.get("localizationStatus") or "")
                px = robot.get("poseX")
                py = robot.get("poseY")
                notes.append(f"robot {args.robot_code} status={status} pose=({px},{py})")
                if "LOCALIZED" not in status.upper() and status.upper() not in ("IDLE", "ONLINE", "AVAILABLE"):
                    # Fleet may use RobotStatus enum; accept pose presence as soft LOCALIZED proxy
                    if px is None or py is None:
                        failures.append(f"robot not localized (status={status}, no pose)")
                if spawn is not None and px is not None and py is not None:
                    if not pose_in_bay(float(px), float(py), spawn, args.park_radius_m):
                        failures.append(
                            f"pose ({px},{py}) outside PARK bay radius {args.park_radius_m} m of "
                            f"({spawn['x']},{spawn['y']})"
                        )
                    else:
                        notes.append("pose inside PARK bay")
        except (urllib.error.URLError, TimeoutError, json.JSONDecodeError) as exc:
            failures.append(f"live backend check failed: {exc}")
    else:
        notes.append("live robot check skipped")

    print("=== sim_init_gate ===")
    for n in notes:
        print(f"OK  {n}")
    for f in failures:
        print(f"FAIL {f}")
    if failures:
        print("RESULT: BLOCKED — do not start Traffic Active jobs")
        return 1
    print("RESULT: READY")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
