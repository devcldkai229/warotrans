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
`/odom`. By default it also publishes TF `odom -> base_footprint`. When bringup
uses `enable_ekf:=true`, pass `publish_tf:=false` so EKF is the sole TF owner.

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

When EKF is authoritative (`enable_ekf:=true`), set `publish_tf:=false` on
`wheel_odom_node` (launch arg / `hardware.yaml`) and let EKF own the edge.

## Calibration gate

`config/hardware.yaml` contains `0.0` sentinels until measured values are
recorded. `wheel_odom_node` refuses to start with those values; do not replace
them with estimates.
