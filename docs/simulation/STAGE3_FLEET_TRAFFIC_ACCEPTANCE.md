# Stage 3 — Multi-robot traffic & RouteGraph retirement

**Date:** 2026-11-01  
**DB map entities:** READ-ONLY  
**Mode:** `Traffic:Mode=Semantic` + `nav_runtime:=nav2_semantic`

## What shipped

| Area | Artifact |
|---|---|
| Dual Gazebo | `multi_sim_map_one.launch.py`, `launch_sim_map_one_dual.sh` (SIM-01@EP-PARKING-A, SIM-02@EP-PARKING-B) |
| Resources | `zone:{code}` SINGLE_OCCUPANCY + `cell:ix:iy` swept corridor (horizon-limited) |
| Coordinator | `BindSemanticPolicy` / `AcquireSemanticPlan` on `TrafficState` |
| MQTT | `traffic_plan` uplink; authority carries `semanticMapFingerprint`, `planRevision`, `resourceIds` |
| Robot | `traffic_mode:=semantic_active` (`semantic_traffic_client.py`) |
| Jobs | Semantic MOVE issues NavigateToPose; grant after plan propose |
| Monitor | Snapshot shows semantic resource holders |
| Editor | Check/Export RouteGraph demoted to offline/legacy |
| Retirement | Unlinked from Semantic Active path; sources kept — see `ROUTEGRAPH_RETIREMENT_REPORT.md` |

## Launch

```bash
# rebuild MAP-ONE bundle if needed
python3 ros2_ws/src/warotrans_simulation/scripts/build_map_one_sim_bundle.py \
  --snapshot /path/to/warotrans-system/samples/maps/MAP-ONE-V001

# Backend: Traffic:Mode=Semantic, Traffic:SemanticMapDirectory=<.../maps/map_one_v001>
colcon build --packages-select warotrans_simulation warotrans_fleet warotrans_navigation warotrans_lane_planner
source install/setup.bash
bash ros2_ws/scripts/launch_sim_map_one_dual.sh gui:=true
```

## Unit / integration

| Check | Result |
|---|---|
| SemanticReservation zone+cell collect | **PASS** (unit) |
| Exclusive SINGLE_OCCUPANCY second robot queues | **PASS** (unit) |
| Fingerprint mismatch reject | **PASS** (unit) |
| Physical clearance of cells | **PASS** (unit) |
| Dual-robot Gazebo bringup | **NOT_RUN** (needs live WSL/Gazebo session) |

## Gazebo E2E matrix (SIM-01 + SIM-02)

| # | Scenario | Result | Notes |
|---|---|---|---|
| 1 | Simultaneous separate routes | **NOT_RUN** | |
| 2 | Same SINGLE_OCCUPANCY; one enters | **NOT_RUN** | unit covers queue |
| 3 | First exits; second grant | **NOT_RUN** | |
| 4 | Paths cross unmarked area | **NOT_RUN** | cell locks |
| 5 | Head-on narrow corridor | **NOT_RUN** | |
| 6 | Wait outside NO_STOP | **NOT_RUN** | Stage-2 validator |
| 7 | Replan → new planRevision | **NOT_RUN** | |
| 8 | MQTT disconnect while holding | **NOT_RUN** | quarantine path reused |
| 9 | Failure inside reserved area | **NOT_RUN** | fail-closed |
| 10 | Two concurrent Transport Requests | **NOT_RUN** | |
| 11 | ONE_WAY / KEEP_OUT / SPEED | **NOT_RUN** | Stage-2 filters+LaneAware |
| 12 | No collision / unauthorized motion | **NOT_RUN** | |

Static/unit evidence alone does **not** count as Gazebo E2E PASS.

## Remaining limitations

- Dual Nav2 namespacing + Gazebo TF prefixes need live soak; first bringup may need remapping tweaks.
- Station `bay:` holds on Semantic path are deferred (endpoint service holds still edge-graph oriented).
- Fleet Monitor path polyline preview from `proposedPath` is data-ready; canvas drawing may need a follow-up polish.
- Offline RouteGraph Check/Compiler sources remain for map QA until a later delete pass.
