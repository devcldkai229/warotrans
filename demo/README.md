# WaroTrans Demo — Phase 3 (Semantic Navigation Integration)

Phase 1 Endpoint Navigate + Phase 2 editor + Phase 3 Nav2 hooks.

```text
demo/
  backend/    FastAPI + rclpy (Pi)
  frontend/   Vite + React + TypeScript
```

## Phase 3 behavior

| Feature | Mechanism |
|---|---|
| Lane route | ordered lanes → centerline poses → `/navigate_through_poses` |
| Displayed path | still `/plan` (Nav2 actual plan) |
| KEEP_OUT | polygon → mask → `KeepoutFilter` (global+local) |
| SPEED_LIMIT | polygon → mask → `SpeedFilter` → `/speed_limit` → controller |
| JUNCTION | semantic only — allows lane sequence gaps |
| SINGLE_ROBOT | config-only (no reservation) |

Lane does **not** rail-follow; Nav2 still plans/avoids obstacles.

## Prerequisite

```bash
# generates empty keepout/speed masks then starts Nav2 (+ filters)
bash ~/tools/warotrans-nav-start.sh ~/maps/warotrans.yaml
# RViz: 2D Pose Estimate
bash ~/warotrans/tools/warotrans-demo-phase1-start.sh
```

UI: `http://warotrans.local:8000/`

## REST (added in Phase 3)

| Method | Path |
|---|---|
| POST | `/api/navigate/lanes` body `{lane_ids[], spacing_m?}` |
| POST | `/api/filters/sync` rebuild KEEP_OUT/SPEED masks + LoadMap |

Zone create/update/delete also auto-calls filter sync when map is ready.

## Operator checks

1. Add Lane route → **Navigate Lanes** → green `/plan` on map, robot moves.
2. Place KEEP_OUT on a free corridor → Sync Filters → replan avoids region.
3. Place SPEED_LIMIT → enter zone → watch `/speed_limit` / UI speed readout / slower `/cmd_vel`.
