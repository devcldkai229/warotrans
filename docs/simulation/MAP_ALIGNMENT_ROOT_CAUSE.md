# Map Alignment — Root Cause

**Date:** 2026-10-25  
**MapVersion:** MAP-WHSIM-V018  
**Status:** Primary cause identified (offline audit); spawn fix is Phase 2.

## Symptom

On Fleet Map, the simulated robot appears near the **left / SLAM-origin** side of the warehouse, while PARK endpoints (EP-PARK-1 / EP-PARK-2) sit toward the **upper-right** of the published map. Operators expect SIM robots to start in a PARK bay.

## What is *not* broken

| Boundary | Finding |
|---|---|
| Occupancy ↔ Gazebo | Generator builds solid cells from the same PGM/YAML used by Nav2 (`world ≡ map`). Origin (−0.594, −2.719), 0.05 m/px, 107×68. |
| Occupancy ↔ FE | Fleet Map / editor use `worldToPixel` with Y-flip only (image v vs world Y). No extra pixel fudge. |
| Pose telemetry | Gateway publishes map-frame pose; backend stores PoseX/Y/Yaw with **no** transform. |
| TF ownership | `map→odom` AMCL; `odom→base_footprint` Gazebo bridge (or EKF when enabled). One publisher per edge. |

## Root cause

`config/warehouse_layout.yaml` hardcoded:

```yaml
spawn: { x: 0.0, y: 0.0, yaw: 0.0 }
```

`simulation.launch.py` and `sim_bringup.launch.py` feed that pose into:

1. `ros_gz_sim create -x/-y/-Y`
2. AMCL `initial_pose.*`

Published PARK poses (overlay / route graph) are approximately:

| Code | X | Y | Yaw |
|---|---|---|---|
| EP-PARK-1 | 2.959 | 0.357 | −π/2 |
| EP-PARK-2 | 3.415 | 0.357 | −π/2 |

So Gazebo, AMCL, MQTT, and FE all agree — they correctly show a robot at the **SLAM start**, not at PARK. The UX bug is an **init contract** bug, not a coordinate-frame bug.

## Secondary traps (do not “fix” with offsets)

- Stale docs citing old 11×7.5 m demos / spawn (6.0, 0.8).
- `nav_regression.py` demo coordinates unrelated to V018.
- Layout / map / route_graph drift if `sync_web_map.py` + `generate_warehouse.py` are skipped after Map Editor publishes.
- Traffic Active false short goals when spawn is near origin while the route starts at PARK.

## Fix direction (Phase 2+)

1. Drive spawn from `spawn_endpoint_code: EP-PARK-*` resolved via overlay (offline) or Navigation API.
2. Fail-closed init: LOCALIZED, pose in PARK bay, map/graph match, clear stale authority.
3. Dev-only FE overlay for landmark deltas — reuse `worldToPixel` only.
4. Re-run Traffic Active E2E only after PARK spawn evidence.

## Explicit non-fixes

- No silent PGM/geometry edits.
- No second `map→odom` publisher.
- No FE pixel offsets to “move” the robot onto PARK.
