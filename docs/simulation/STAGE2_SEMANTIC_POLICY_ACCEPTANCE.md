# Stage 2 — Semantic navigation & policy enforcement (MAP-ONE-V001)

**Date:** 2026-10-31  
**DB:** READ-ONLY (snapshot only)  
**Scope:** Semantic Map Adapter + LaneAwareNavfn ONE_WAY + KEEP_OUT/SPEED filters + NO_STOP validator.  
**Non-goals:** Fleet SINGLE_OCCUPANCY enforcement, multi-robot, RouteGraph removal, Backend pathfinding.

## Artifacts (from snapshot)

```text
maps/map_one_v001/
  map.pgm|.yaml
  keepout.pgm|.yaml|.json
  speed.pgm|.yaml
  lanes.json
  no_stop.json
  single_occupancy.json   # metadata only → NOT_SUPPORTED
  semantic_manifest.json  # semanticMapFingerprint bumped
```

Rebuild:

```bash
python3 ros2_ws/src/warotrans_simulation/scripts/build_map_one_sim_bundle.py \
  --snapshot /path/to/warotrans-system/samples/maps/MAP-ONE-V001
```

## Runtime wiring

| Piece | Mechanism |
|---|---|
| KEEP_OUT / SPEED | `enable_costmap_filters:=true` + mask YAML from bundle |
| ONE_WAY | `warotrans_lane_planner/LaneAwareNavfn` via `nav2_params_lane_aware.yaml` |
| Defense-in-depth | `path_policy_validator` on `/plan` + `/goal_pose` |
| JUNCTION | Ignored in masks (compat only) |
| SINGLE_OCCUPANCY | Exported IDs/polygons; enforcement **NOT_SUPPORTED** |
| Authority | Stage-1 `stage1_open_grant` remains (Stage 3 replaces) |

Launch:

```bash
bash ros2_ws/scripts/launch_sim_map_one_semantic.sh gui:=true
```

## Unit tests (always run)

```bash
python -m pytest ros2_ws/src/warotrans_simulation/test/test_semantic_map_adapter.py -q
# or after colcon: colcon test --packages-select warotrans_simulation
```

| Check | Result (2026-10-31) |
|---|---|
| Raster keepout; JUNCTION omitted | **PASS** |
| SPEED overlap → most restrictive | **PASS** |
| ONE_WAY move_allowed / reverse reject | **PASS** |
| NO_STOP goal reject | **PASS** |
| Fingerprint stable + lanes tangent_samples | **PASS** |
| Lane corridor membership | **PASS** |
| LaneAware A* reverse-only corridor → empty | **PASS** |
| `colcon build` warotrans_lane_planner (Jazzy) | **PASS** |

## SIM-01 Gazebo matrix

Evidence rule: Motion PASS requires live Gazebo/Nav2. Static/unit evidence alone → do not mark Motion PASS.

| # | Scenario | Expected | Result | Notes |
|---|---|---|---|---|
| 1 | Navigate along ONE_WAY forward | Path found; motion OK | **NOT_RUN** | Needs WSL/Gazebo display |
| 2 | Goal requiring reverse on ONE_WAY | `NO_VALID_PATH` / empty plan; no motion | **NOT_RUN** | LaneAware + validator |
| 3 | Cross KEEP_OUT footprint | Blocked by filter / validator | **NOT_RUN** | |
| 4 | Traverse SPEED zone | Speed limit from SpeedFilter | **NOT_RUN** | |
| 5 | Goal inside NO_STOP | Reject `NO_STOP_GOAL` | **NOT_RUN** | Validator on `/goal_pose` |
| 6 | Continuous traverse through NO_STOP | Allowed if no stop planned | **NOT_RUN** | |
| 7 | Free-space off-lane (no corridor) | Normal Navfn-like search | **NOT_RUN** | |
| 8 | Replan after blocked cell | Validator re-checks new `/plan` | **NOT_RUN** | |
| 9 | Recovery does not clear keepout | Keepout filter remains | **NOT_RUN** | Stage-1 recovery policy |
| 10 | SINGLE_OCCUPANCY two robots | Reservation | **NOT_SUPPORTED** | Stage 3 |

### Blockers for NOT_RUN rows

- Gazebo/ROS Jazzy SIM-01 not executed in this Windows agent session (no live sim clock / display / broker evidence this turn).
- Re-run on WSL with `launch_sim_map_one_semantic.sh` and fill PASS/FAIL per row.

## Explicit NOT_SUPPORTED

- Fleet `SINGLE_OCCUPANCY` mutual exclusion
- Multi-robot conflict resolution
- RouteGraph deletion / Backend RouteGraph for MOVE
- Production traffic Active without Stage 3 grants
