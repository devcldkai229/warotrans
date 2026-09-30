# warotrans_hardware

Owns the low-level software boundary between ROS 2 and the physical robot.

## Nodes

```text
esp32_bridge_node.py
wheel_odom_node.py
diagnostics_node.py
```

`esp32_bridge_node` reads cumulative encoder telemetry from `/dev/warotrans`,
publishes `/wheel_ticks`, and forwards `/cmd_vel` to the ESP32 as
`V <linear_mps> <angular_rps>`. `wheel_odom_node` consumes that topic and publishes
`/odom` plus the dynamic TF `odom -> base_footprint`.

## Responsibilities

- stable ESP32 serial connection
- parse encoder telemetry
- publish cumulative wheel ticks
- forward `/cmd_vel` to ESP32 velocity commands
- publish wheel odometry and its owned dynamic TF
- publish low-level diagnostics

## Does not own

- SLAM
- AMCL
- Nav2 planning
- camera perception
- fleet task logic

## TF ownership

Initially wheel odometry may publish:

```text
odom -> base_footprint
```

When EKF becomes authoritative, disable that TF publication in wheel odometry and let EKF own it.

## Calibration gate

`config/hardware.yaml` contains `0.0` sentinels until measured values are
recorded. `wheel_odom_node` refuses to start with those values; do not replace
them with estimates.
