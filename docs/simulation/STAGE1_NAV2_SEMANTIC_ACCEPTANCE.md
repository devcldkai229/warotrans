# Stage 1 — Nav2 foundation & SIM-01 (MAP-ONE-V001)

**Date:** 2026-10-31  
**DB:** READ-ONLY (snapshot only; no MapVersion writes)  
**Scope:** Free-space Nav2 for SIM-01. No ONE_WAY plugin, no multi-robot, no RouteGraph deletion.

## Dependency map (audit)

### KEEP (reuse)

| Component | Path / notes |
|---|---|
| Gazebo bringup | `warotrans_simulation/launch/sim_bringup.launch.py`, `simulation.launch.py` |
| Nav2 free-space | `warotrans_navigation/launch/navigation.launch.py` with empty `route_graph` |
| Planner / controller | `nav2_params.yaml` GridBased + FollowPath RPP |
| AMCL + map_server | same as Pi; initial pose = spawn |
| cmd_vel pipeline | velocity_smoother → collision_monitor → (optional) authority gate → `/cmd_vel` |
| MQTT gateway | `robot_gateway_node` NavigateToPose |
| MAP-ONE snapshot | `warotrans-system/samples/maps/MAP-ONE-V001/` |

### MODIFY (this stage)

| Change | Purpose |
|---|---|
| `nav_runtime:=nav2_semantic` | Force empty route graph / free-space |
| `world:=map_one_v001.sdf` | Derived Gazebo collision from MAP-ONE occupancy |
| `stage1_open_grant:=true` | Publish `/traffic/authority_ok` while gate stays ON |
| `build_map_one_sim_bundle.py` | Offline bundle + `semantic_manifest.json` |
| `spawn_pose` layout.endpoints | Resolve spawn without web overlay |

### RETIRE from *this* runtime path (code kept)

- Nav2 Route Server / `navigate_to_pose_route.xml`
- Backend RouteGraphBuilder / TrafficGraphCompiler for MOVE path
- Segmented edge grants / graphHash path identity
- Junction portals / TURN as navigation prerequisites

## Semantic map bundle

Built from snapshot (no DB):

```text
maps/map_one_v001/map.pgm|.yaml
maps/map_one_v001/semantic_manifest.json
config/layouts/map_one_v001.yaml
worlds/map_one_v001.sdf
config/map_one_v001_endpoints.json
```

Rebuild:

```bash
python3 ros2_ws/src/warotrans_simulation/scripts/build_map_one_sim_bundle.py \
  --snapshot /path/to/warotrans-system/samples/maps/MAP-ONE-V001
```

Focus endpoints (from snapshot): EP-PARKING-A, EP-INBOUND, EP-SHELFA01, EP-OUTBOUND.

Note: `spawn_require_clearance: false` — Parking-A footprint is tight vs occupancy; spawn still uses authored pose.

## Launch commands

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
# rebuild simulation package after pull
colcon build --packages-select warotrans_simulation --symlink-install
source install/setup.bash

bash scripts/launch_sim_map_one_semantic.sh gui:=true rviz:=true
# or headless:
bash scripts/launch_sim_map_one_semantic.sh gui:=false
```

Equivalent explicit args:

```bash
ros2 launch warotrans_simulation sim_bringup.launch.py \
  layout:=$(ros2 pkg prefix warotrans_simulation)/share/warotrans_simulation/config/layouts/map_one_v001.yaml \
  map:=$(ros2 pkg prefix warotrans_simulation)/share/warotrans_simulation/maps/map_one_v001/map.yaml \
  world:=$(ros2 pkg prefix warotrans_simulation)/share/warotrans_simulation/worlds/map_one_v001.sdf \
  spawn_endpoint_code:=EP-PARKING-A \
  nav_runtime:=nav2_semantic \
  route_graph:= \
  enable_authority_gate:=true \
  stage1_open_grant:=true \
  gateway_params_file:=$(ros2 pkg prefix warotrans_simulation)/share/warotrans_simulation/config/robot_gateway_sim_semantic.yaml \
  map_version_code:=MAP-ONE-V001
```

### Test goals (after Nav2 ACTIVE)

```bash
# Example: EP-INBOUND (1.758, 1.617, -π/2) — use ros2 action or MQTT NAVIGATE_TO_POSE
ros2 topic echo /plan --once   # planned path
ros2 topic echo /odom --once   # actual trajectory source
# Look for log: STAGE1_OPEN_GRANT
# Gate must be present: ros2 node list | grep cmd_vel_authority_gate
```

## Acceptance matrix

| # | Criterion | Result | Evidence |
|---|---|---|---|
| 1 | SIM-01 launches and localizes | **NOT_RUN** | Full `sim_bringup` + AMCL not held long enough in CI/agent; SDF load smoke PASS |
| 2 | Occupancy / Gazebo alignment | **PARTIAL** | Same PGM extruded to SDF (134 blocks); `gz sim -s` smoke exit 0; visual RViz overlay NOT_RUN |
| 3 | Free-space path without RouteGraph | **PASS** (config) | `nav_runtime:=nav2_semantic` clears `route_graph` |
| 4 | Physical motion under gate | **NOT_RUN** | Requires interactive Gazebo + Nav2 ACTIVE |
| 5 | Reach representative Endpoint | **NOT_RUN** | Requires interactive Gazebo |
| 6 | Collision monitor + authority gate active | **PASS** (config) | `enable_cmd_vel_pipeline` + `enable_authority_gate` + `stage1_open_grant` |
| 7 | Plan vs trajectory distinguishable | **NOT_RUN** | Documented topics `/plan` vs `/odom` |
| — | ONE_WAY / SINGLE_OCCUPANCY / multi-robot | **NOT_SUPPORTED** | Stage 2+ |

Offline / smoke checks that **did** run:

- Bundle build from snapshot → SDF/map/layout/manifest
- `colcon build --packages-select warotrans_simulation` PASS
- `spawn_pose` for Parking-A / Inbound / ShelfA01 / Outbound PASS
- `timeout 12 gz sim -s -r worlds/map_one_v001.sdf` exit 0

## Stage-2 blockers

1. Live Gazebo E2E on WSL (GUI/display, MQTT broker, colcon install of fleet+navigation) to flip NOT_RUN → PASS/FAIL.
2. Parking-A footprint clearance vs occupancy (may need map/editor pose move — **not** done in Stage 1).
3. Extruded-PGM world is coarse; shelves are walls-only until refined props.
4. ONE_WAY hard planner plugin + KEEP_OUT/SPEED masks.
5. Semantic traffic reservations (`semanticMapFingerprint` + `planRevision`).
6. Dual-robot namespaces.

## Safety note

`STAGE1_OPEN_GRANT` is an **explicit, logged, Stage-1-only** open of `/traffic/authority_ok`. It is not production fleet authorization and must not ship to the physical robot.
