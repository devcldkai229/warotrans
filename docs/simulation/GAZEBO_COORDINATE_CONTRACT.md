# Gazebo Coordinate Contract

**Authority:** `docs/simulation/05-map-tu-web.md`, MapVersion MAP-WHSIM-V018, `warehouse_layout.yaml`.

## Single frame

```text
world ≡ map
```

Gazebo world XY, ROS `map` frame, backend PoseX/Y, Fleet Map world meters, and published Endpoints all share one right-handed metre frame. Occupancy origin and resolution define the raster; they do **not** introduce a second “image frame” for robot poses.

## Occupancy meta (V018)

| Property | Value |
|---|---|
| Image | `warotrans_map3.pgm` (also `maps/sim_warehouse.*`) |
| Size | 107 × 68 px |
| Resolution | 0.05 m/px |
| Origin | (−0.594, −2.719, 0) — lower-left of the grid in `map` |
| Negate / rotation | Rejected by generator; yaw must be 0 |

## TF

```text
map → odom          AMCL (sim_bringup)
odom → base_footprint   Gazebo odom bridge (default) or EKF if enable_ekf:=true
base_footprint → …  robot_state_publisher / URDF
```

One publisher per TF edge. Do not add a second map–odom source to “fix” alignment.

## Spawn

| Rule | Detail |
|---|---|
| Source of truth | Published `EP-PARK-*` on the active MapVersion (overlay or Navigation API) |
| Layout field | `spawn_endpoint_code` (e.g. `EP-PARK-1`) |
| Pipeline | resolved (x,y,yaw) → Gazebo `create` **and** AMCL `initial_pose` |
| Forbidden | Hardcoding (0,0) as “demo park”; teleport mid-Job |

`(0,0)` remains the historical SLAM start cell — useful as a landmark, **not** the default SIM parking bay.

## Frontend

`mapCoordinateTransform.worldToPixel` / `pixelToWorld` apply resolution + origin + image-height Y flip only. Fleet Map markers consume backend PoseX/Y/Yaw in world metres. Alignment debug overlays must reuse this transform; they must not invent offsets.

## Fail-closed

If overlay/API cannot resolve `spawn_endpoint_code`, or footprint clearance fails against occupancy, **abort launch** with an explicit reason. Do not fall back silently to (0,0).
