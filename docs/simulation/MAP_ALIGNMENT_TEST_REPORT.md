# MAP Alignment Test Report

Generated: 2026-10-25T15:10:17.212919+00:00

## Map meta (occupancy)

| Field | Value |
|---|---|
| map yaml | `E:\programming\my_project\warotrans\ros2_ws\src\warotrans_simulation\maps\sim_warehouse.yaml` |
| image | `sim_warehouse.pgm` 107×68 |
| resolution | 0.05 m/px |
| origin | (-0.594, -2.719, 0.0) |
| map_version_code | MAP-WHSIM-V018 |
| map_version_id | 7cb001f7-cbf3-4f3e-8400-fddc00d63b19 |

## Contract check

| Check | Expected | Actual | Result |
|---|---|---|---|
| origin yaw | 0 | 0.0 | PASS |
| resolution | 0.05 | 0.05 | PASS |
| size | 107×68 | 107×68 | PASS |
| origin ~ (-0.594,-2.719) | yes | (-0.594,-2.719) | PASS |

## World corners (FE worldToPixel inverse)

| Landmark | world X | world Y |
|---|---|---|
| origin_ll | -0.5940 | -2.7190 |
| top_left | -0.5940 | 0.6810 |
| bottom_right | 4.7560 | -2.7190 |

## PARK endpoints (overlay) vs FE pixel round-trip

| Code | world X | world Y | yaw | FE u | FE v | round-trip dx | dy |
|---|---|---|---|---|---|---|---|
| EP-PARK-1 | 2.9590 | 0.3570 | -1.5708 | 71.06 | 6.48 | 0.000000 | 0.000000 |
| EP-PARK-2 | 3.4150 | 0.3570 | -1.5708 | 80.18 | 6.48 | 0.000000 | 0.000000 |

## Layout spawn

- `spawn_endpoint_code`: `EP-PARK-1`
- explicit `spawn`: `{}`
- resolved PARK pose: (2.959, 0.357, -1.5707963267948966) — must drive Gazebo create + AMCL initial_pose

## Runtime columns (live Gazebo 2026-10-25)

| Landmark | Gazebo model pose | ROS AMCL initial | MQTT PoseX/Y | FE marker | d Gazebo-FE |
|---|---|---|---|---|---|
| EP-PARK-1 | create `-x 2.959 -y 0.357 -Y -1.571` (Entity creation successful) | `Setting pose: 2.959 0.357 -1.571` | NOT_RUN (Host :5090 down) | NOT_RUN | — |
| EP-PARK-2 | resolved offline (3.415, 0.357, -1.571); dedicated bringup only | NOT_RUN this session | NOT_RUN | NOT_RUN | — |
| map origin cell | still landmark (0,0) SLAM start — not default spawn | — | — | — | — |

Log excerpt (`_agent_launch_sim_active.sh`):

```text
[sim_bringup] spawn=(2.9590,0.3570,-1.5708) source=overlay:EP-PARK-1 endpoint=EP-PARK-1
[amcl]: Setting pose (0.000000): 2.959 0.357 -1.571
[spawn_warotrans]: Entity creation successful.
```

## Verdict

Offline contract PASS (world==map, FE round-trip). Primary UX bug fixed by `spawn_endpoint_code: EP-PARK-1`. Live Gazebo+AMCL confirm PARK-1 pose. Full Fleet Map / MQTT / inbound Job evidence still needs Host on :5090 + FE with `VITE_MAP_ALIGN_DEBUG=true`.
