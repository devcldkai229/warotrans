# SIM Traffic Active E2E Results (Map / PARK alignment)

**Date:** 2026-10-25  
**MapVersion:** MAP-WHSIM-V018  
**Claim:** Do **not** mark SIMULATION ACCEPTED — Host inbound Job not completed this session.

## Environment

| Component | Status |
|---|---|
| WSL Gazebo + Nav2 route + Active gate | Running (`_agent_launch_sim_active.sh`) |
| MQTT :1883 | LISTEN |
| WaroTrans.Host :5090 | Down during run → live MQTT/FE Job **BLOCKED** |
| FE `VITE_MAP_ALIGN_DEBUG` | Implemented; enable in `.env.local` for overlay |

## Acceptance M01–M12

| ID | Check | Result | Evidence |
|---|---|---|---|
| M01 | Occupancy meta matches V018 (origin/res/size) | **PASS** | `measure_landmarks.py` + map_server load 107×68 @ 0.05, origin (−0.594,−2.719) |
| M02 | FE worldToPixel round-trip on PARK | **PASS** | Report PARK table dx/dy ≈ 0 |
| M03 | SIM-01 Gazebo spawn at EP-PARK-1 | **PASS** | `spawn=(2.9590,0.3570,-1.5708) source=overlay:EP-PARK-1` |
| M04 | SIM-02 spawn at EP-PARK-2 (dedicated) | **PASS** (resolve) / live spawn **NOT_RUN** | `spawn_pose.py --endpoint EP-PARK-2` → (3.415, 0.357, −π/2); overlay `robot_gateway_sim_02_active_overlay.yaml` |
| M05 | Concurrent dual robot same world | **BLOCKED** | No namespaced dual-AMCL/Nav2 stack |
| M06 | Dual MQTT clients distinct | **PASS** (config) | SIM-01 `…sim-01-active` vs SIM-02 `…sim-02-active` client ids |
| M07 | AMCL initial_pose = PARK spawn | **PASS** | `amcl: Setting pose: 2.959 0.357 -1.571` |
| M08 | Init gate fail-closed + clear authority | **PASS** | `sim_init_gate.py` RESULT READY; clears `/tmp/warotrans_authority_state.json` |
| M09 | Fleet Map debug overlay | **PASS** (code) | `VITE_MAP_ALIGN_DEBUG` → `MapAlignDebugOverlay` |
| M10 | Clean inbound Traffic Active E2E from PARK | **BLOCKED** | Needs Host :5090 + TransportRequest; spawn fixed so false short-goal from (0,0) should stop |
| M11 | Real motion (not instant FollowPath success) | **NOT_RUN** | Depends on M10 |
| M12 | Light dual concurrency / traffic wait | **BLOCKED** | Depends on M05 |

## Commands used

```bash
python3 scripts/measure_landmarks.py --out docs/simulation/MAP_ALIGNMENT_TEST_REPORT.md
python3 scripts/sim_init_gate.py --skip-live
bash ros2_ws/scripts/_agent_launch_sim_active.sh
# SIM-02 bay (single stack):
ros2 launch warotrans_simulation sim_bringup.launch.py \
  spawn_endpoint_code:=EP-PARK-2 robot_code:=RBT-SIM-02 robot_name:=warotrans_sim_02 \
  gateway_params_file:=.../robot_gateway_sim_02_active_overlay.yaml ...
```

## Changed files (alignment work)

**warotrans**

- `ros2_ws/src/warotrans_simulation/config/warehouse_layout.yaml` — `spawn_endpoint_code: EP-PARK-1`
- `scripts/spawn_pose.py`, `measure_landmarks.py`, `sim_init_gate.py`
- `launch/simulation.launch.py`, `sim_bringup.launch.py`, `launch/_spawn_util.py`
- `config/robot_gateway_sim_02_active_overlay.yaml`
- `ros2_ws/scripts/_agent_launch_sim_active.sh`
- `docs/simulation/MAP_ALIGNMENT_ROOT_CAUSE.md`, `GAZEBO_COORDINATE_CONTRACT.md`, `SIM_ROBOT_INITIALIZATION.md`, `MAP_ALIGNMENT_TEST_REPORT.md`, this file

**warotrans-system**

- `web/src/features/robots/MapAlignDebugOverlay.tsx`, `FleetMap.tsx`, `FleetPage.tsx`
- `web/.env.example` — `VITE_MAP_ALIGN_DEBUG`

## Next to close M10–M11

1. Start Host (`samples/simulation/run-backend-sim.ps1` or equivalent) with JWT key.
2. Confirm robot LOCALIZED near PARK via `sim_init_gate.py` without `--skip-live`.
3. Create inbound TransportRequest on V018; watch FollowPath length ≫ 0.01 m and Fleet Map track.
4. Only then re-evaluate Active collision/gate notes in `docs/SIM-TRAFFIC-E2E-AGENT-NOTES.md`.
