# SIM Robot Initialization

Clean boot for Traffic Active Gazebo + backend. Fail closed on mismatch.

## Contract

1. **Map:** `MAP-WHSIM-V018` occupancy (`sim_warehouse.yaml` / `warotrans_map3`) matches layout `web.map_version_*`.
2. **Spawn:** `spawn_endpoint_code` → published `EP-PARK-*` (overlay or API). Same pose for Gazebo `create` and AMCL `initial_pose`.
3. **Authority:** delete stale `/tmp/warotrans_authority_state.json` before Active gateway (avoids `AUTHORITY_STALE_SEQ`).
4. **Localization:** wait until gateway reports localized / valid map pose near PARK (±0.35 m).
5. **No mid-job teleport.** Re-spawn only after cancelling Jobs and clearing authority.

## Commands

```bash
# Resolve spawn (offline)
ros2 run warotrans_simulation spawn_pose.py \
  --layout $(ros2 pkg prefix warotrans_simulation)/share/warotrans_simulation/config/warehouse_layout.yaml \
  --map $(ros2 pkg prefix warotrans_simulation)/share/warotrans_simulation/maps/sim_warehouse.yaml

# Fail-closed gate (clears authority state)
ros2 run warotrans_simulation sim_init_gate.py \
  --expected-map-version MAP-WHSIM-V018 \
  --robot-code RBT-SIM-01 \
  --skip-live   # use without --skip-live when Host + MQTT are up

# Active bringup (SIM-01 @ EP-PARK-1)
rm -f /tmp/warotrans_authority_state.json
ros2 launch warotrans_simulation sim_bringup.launch.py \
  gui:=true \
  enable_authority_gate:=true \
  map_version_code:=MAP-WHSIM-V018 \
  route_graph:=$HOME/maps/route_graph_v018.geojson \
  gateway_params_file:=.../robot_gateway_sim_active_overlay.yaml

# SIM-02 bay only (single stack — not concurrent dual Nav2)
ros2 launch warotrans_simulation sim_bringup.launch.py \
  spawn_endpoint_code:=EP-PARK-2 \
  robot_code:=RBT-SIM-02 \
  robot_name:=warotrans_sim_02 \
  ...
```

Helper script: `ros2_ws/scripts/_agent_launch_sim_active.sh` (clears authority, Active overlay).

## Ready checklist (before first MOVE)

| Check | Pass criteria |
|---|---|
| Init gate | `sim_init_gate.py` exit 0 |
| Spawn log | `[sim_bringup] spawn≈(2.959,0.357,-π/2) endpoint=EP-PARK-1` |
| TF | `map→odom` AMCL; `odom→base_footprint` bridge/EKF |
| MQTT | Heartbeat + telemetry pose near PARK |
| Fleet Map | Marker on PARK (enable `VITE_MAP_ALIGN_DEBUG=true` for overlay) |
| Nav2 | bt_navigator ACTIVE; no orphan RUNNING navigate |
| Traffic | Active authority publisher renewing; gate not BLOCKING on stale |

## Dual robot note

Full concurrent dual-AMCL/Nav2 in one process is **not** implemented. Validate SIM-02 PARK pose with a dedicated bringup (`spawn_endpoint_code:=EP-PARK-2`, unique `robot_code` / MQTT client / `robot_name`). Concurrency scenarios M05/M12 stay BLOCKED until a namespaced dual stack exists.
