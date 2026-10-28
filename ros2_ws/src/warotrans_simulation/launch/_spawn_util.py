"""Launch-time helper: import scripts/spawn_pose.py from share or source tree."""

from __future__ import annotations

import importlib.util
import os
from typing import Any, Optional


def _load_spawn_pose_module():
    candidates: list[str] = []
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(here, "..", "scripts", "spawn_pose.py"))
    try:
        from ament_index_python.packages import get_package_prefix

        prefix = get_package_prefix("warotrans_simulation")
        candidates.append(os.path.join(prefix, "lib", "warotrans_simulation", "spawn_pose.py"))
    except Exception:
        pass
    for path in candidates:
        path = os.path.normpath(path)
        if not os.path.isfile(path):
            continue
        spec = importlib.util.spec_from_file_location("warotrans_spawn_pose", path)
        if spec is None or spec.loader is None:
            continue
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod
    raise ImportError("spawn_pose.py not found next to launch or in lib/warotrans_simulation")


def resolve_layout_spawn(
    layout_path: str,
    *,
    endpoint_code: Optional[str] = None,
    map_yaml: Optional[str] = None,
    backend: Optional[str] = None,
) -> dict[str, Any]:
    mod = _load_spawn_pose_module()
    return mod.resolve_spawn(
        layout_path,
        endpoint_code_override=endpoint_code or None,
        backend=backend or os.environ.get("WAROTRANS_BACKEND") or None,
        map_yaml=map_yaml,
        require_clearance=bool(map_yaml),
    )
