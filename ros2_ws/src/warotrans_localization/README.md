# warotrans_localization

Owns localization/state-estimation configuration for WaroTrans.

## Components

- **AMCL** — Adaptive Monte Carlo Localization for static map
- **Map Server** — serves pre-built map from YAML/PGM files

## Configuration Files

| File | Purpose |
|---|---|
| `config/amcl.yaml` | AMCL parameters (particles, motion model, laser model) |
| `config/ekf.yaml` | (Placeholder) robot_localization EKF params |

## Usage

### Launch localization (with saved map)

```bash
ros2 launch warotrans_localization localization.launch.py map:=/path/to/map.yaml
```

### Check lifecycle state

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

Both should be `active`.

### Set initial pose (from RViz)

1. Open RViz with Nav2 config
2. Click "2D Pose Estimate" tool
3. Click and drag on map to set robot's initial pose
4. AMCL particle cloud should converge

## TF Ownership

When this launch is running:

- **AMCL owns `map → odom`**
- Do NOT run `slam_toolbox` simultaneously

## DoD (Definition of Done)

- [ ] `ros2 lifecycle get /map_server` = `active`
- [ ] `ros2 lifecycle get /amcl` = `active`
- [ ] `ros2 run tf2_ros tf2_echo map odom` shows valid transform
- [ ] Particle cloud converges after setting initial pose
- [ ] AMCL continues localizing during robot motion

## Planned Stages

1. ✓ Wheel odometry baseline (Phase 7)
2. BNO055 verification (future)
3. Optional `robot_localization` EKF (future)
4. ✓ Static-map localization with AMCL (Phase 9)

## Invariant

Do not enable multiple publishers for:

```text
odom → base_footprint
map → odom
```
