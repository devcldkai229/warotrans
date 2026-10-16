# warotrans_localization

Owns localization/state-estimation configuration for WaroTrans.

## Components

- **AMCL** — Adaptive Monte Carlo Localization for static map
- **Map Server** — serves pre-built map from YAML/PGM files
- **EKF** (`robot_localization`) — fuse `/odom` + `/imu/data` → TF `odom→base_footprint`

## Configuration Files

| File | Purpose |
|---|---|
| `config/amcl.yaml` | AMCL parameters (particles, motion model, laser model) |
| `config/ekf.yaml` | EKF fuse wheel odom + BNO055 yaw (provisional covariances) |

## Usage

### Launch localization (with saved map)

```bash
ros2 launch warotrans_localization localization.launch.py map:=/path/to/map.yaml
```

### Launch EKF only (after wheel odom + IMU)

```bash
ros2 launch warotrans_sensors imu.launch.py
# elsewhere: hardware with publish_tf:=false
ros2 launch warotrans_localization ekf.launch.py
```

Bringup flag (mapping / navigation / mobile_base):

```bash
ros2 launch warotrans_bringup mobile_base.launch.py enable_ekf:=true
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

When AMCL localization launch is running:

- **AMCL owns `map → odom`**
- Do NOT run `slam_toolbox` simultaneously

When `enable_ekf:=true`:

- **EKF owns `odom → base_footprint`**
- `wheel_odom_node` must use `publish_tf:=false` (still publishes `/odom`)

## DoD (Definition of Done)

- [ ] `ros2 lifecycle get /map_server` = `active`
- [ ] `ros2 lifecycle get /amcl` = `active`
- [ ] `ros2 run tf2_ros tf2_echo map odom` shows valid transform
- [ ] Particle cloud converges after setting initial pose
- [ ] AMCL continues localizing during robot motion

### EKF / IMU DoD

- [ ] `ros2 topic hz /imu/data` stable; `frame_id=imu_link`
- [ ] Standing: `|acc| ≈ 9.8`; rotate: yaw sense not flipped
- [ ] With EKF: one publisher for `odom→base_footprint` (no TF_REPEATED_DATA)
- [ ] `/odometry/filtered` does not jump

## Planned Stages

1. ✓ Wheel odometry baseline (Phase 7)
2. ✓ BNO055 driver + mount geometry (code); runtime DoD on robot
3. ✓ `robot_localization` EKF config + `enable_ekf` bringup gate
4. ✓ Static-map localization with AMCL (Phase 9 / Nav2 Phase 0)

## Invariant

Do not enable multiple publishers for:

```text
odom → base_footprint
map → odom
```
