# warotrans_description

Owns robot geometry and static TF relationships published by `robot_state_publisher`.

## TF ownership

```text
odom -> base_footprint          # wheel_odom_node (warotrans_hardware)
base_footprint -> base_link     # robot_state_publisher (this package)
base_link -> laser              # robot_state_publisher (this package)
```

This package does **not** publish `odom` or any `odom -> *` transform.

## Layout

| Path | Purpose |
|---|---|
| `urdf/warotrans.urdf.xacro` | Robot model (base_footprint, base_link, laser) |
| `config/geometry.yaml` | Static URDF geometry (machine values) |
| `config/calibration.yaml` | Cross-package calibration (wheel, limits); sync `lidar.*` here when measured |
| `launch/description.launch.py` | Start `robot_state_publisher` |

## Geometry policy

- Chassis envelope (`0.320 x 0.260 x 0.003 m`) is confirmed in `docs/STATUS.md`.
- Laser pose must be measured and recorded in `docs/calibration.md`.
- Launch **fails closed** when required laser / base_link_z fields are still `null`.
- Ultrasonic **FRONT + REAR**: URDF links added automatically when
  `geometry.yaml` `ultrasonic.front` and `ultrasonic.rear` are all non-null.
  LEFT/RIGHT are **not** added (Nav2 Phase 0.5 scope). Check:
  `tools/warotrans-check-ultrasonic-front-rear-geometry.sh`.

## Launch

After `colcon build` on the robot:

```bash
source ~/ros2_ws/install/setup.bash
```

### Measured geometry (production)

Fill `config/geometry.yaml` after calibration, then:

```bash
ros2 launch warotrans_description description.launch.py
```

### Structural smoke test (explicit overrides only)

Use only when geometry is not yet measured. Values are operator-provided at
runtime, not committed to the repo:

```bash
ros2 launch warotrans_description description.launch.py \
  lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.0 lidar_yaw:=0.0 base_link_z:=0.0
```

## TF verification

Static TF only:

```bash
ros2 launch warotrans_description description.launch.py \
  lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.0 lidar_yaw:=0.0 base_link_z:=0.0

ros2 run tf2_tools view_frames
# Expect: base_footprint -> base_link -> laser
# robot_state_publisher must NOT publish odom -> *

ros2 run tf2_ros tf2_echo base_footprint base_link
ros2 run tf2_ros tf2_echo base_link laser
```

Full chain with wheel odometry (separate terminals):

```bash
ros2 launch warotrans_description description.launch.py \
  lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.0 lidar_yaw:=0.0 base_link_z:=0.0

ros2 launch warotrans_hardware hardware.launch.py

ros2 run tf2_ros tf2_echo odom base_footprint
ros2 run tf2_ros tf2_echo odom laser
```

## Xacro sanity check

```bash
xacro $(ros2 pkg prefix warotrans_description)/share/warotrans_description/urdf/warotrans.urdf.xacro \
  geometry_file:=$(ros2 pkg prefix warotrans_description)/share/warotrans_description/config/geometry.yaml \
  base_link_z:=0.0 lidar_x:=0.0 lidar_y:=0.0 lidar_z:=0.0 lidar_yaw:=0.0
```

Expected: four links (`base_footprint`, `base_link`, `laser`, no `odom`), two fixed joints.

## Must not contain

- serial parsing
- encoder calculation
- odometry integration
- SLAM / Nav2 / EKF logic

Sensor poses come from physical measurement, not estimation.
