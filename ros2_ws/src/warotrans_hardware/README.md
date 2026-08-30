# warotrans_hardware

Owns the low-level software boundary between ROS 2 and the physical robot.

## Future nodes

```text
esp32_bridge_node.py
wheel_odom_node.py
diagnostics_node.py
```

Do not create them all at once. Add each after its previous hardware checkpoint passes.

## Responsibilities

- stable ESP32 serial connection
- parse encoder telemetry
- send agreed velocity/control commands
- publish wheel odometry
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
