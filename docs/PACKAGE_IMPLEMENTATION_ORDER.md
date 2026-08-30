# Package implementation order

## Stage A — first map

### A1. `warotrans_description`
Implement measured base/wheel/LiDAR geometry.

**Gate:** TF model is structurally correct.

### A2. `warotrans_sensors`
Implement the LiDAR driver launch and driver configuration.

**Gate:** the scan topic publishes at a stable rate with valid ranges, and
unplugging the device produces a meaningful error instead of taking down the
system.

> Order note: sensors comes before hardware because a working scan is useful on
> its own and needs no encoder work, so it de-risks the LiDAR/USB/udev layer
> while the encoder wiring is still being measured.

### A3. `warotrans_hardware`
Implement only what is necessary for:
- stable ESP32 serial link;
- encoder telemetry;
- `/odom`;
- `odom -> base_footprint`;
- `/cmd_vel` forwarding.

**Gate:** `/odom` and TF are stable.

### A4. `warotrans_bringup`
Compose description + hardware.

**Gate:** one launch starts the robot base.

### A5. `warotrans_slam`
Add SLAM Toolbox config.

**Gate:** valid `/map` and saved map.

## Stage B — navigation quality

### B1. `warotrans_localization`
Add IMU/EKF only if needed, then static map localization.

### B2. firmware PID
Closed-loop wheel/base response.

### B3. `warotrans_navigation`
Add Nav2 after all prerequisites pass.

## Stage C — operator/system integration

### C1. `warotrans_perception`
Camera stream first; perception later.

### C2. `warotrans_fleet`
High-level task/status integration.

## Stage D — simulation / multi-robot

Only after one real robot works end-to-end.
