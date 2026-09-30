# warotrans_sensors

## Responsibility

Sensor **driver layer**. Turns physical devices into ROS topics.

Owns:
- LiDAR driver launch and driver configuration;
- Ultrasonic (MKE-S01 ×4 on Pi GPIO) — monitoring only until Nav2 phase;
  **Nav2 Phase 0.5 chỉ FRONT + REAR** (sau đo mount); LEFT/RIGHT không wire Nav2;
- IMU driver later;
- any additional raw sensor source later.

Does **not** own:
- state estimation (`warotrans_localization`);
- mapping (`warotrans_slam`);
- camera and vision (`warotrans_perception`);
- the ESP32 base link and odometry (`warotrans_hardware`).

## Why this is separate from `warotrans_hardware`

`warotrans_hardware` owns the actuation path and the base I/O that will later be
migrated behind a `ros2_control` hardware interface. Keeping passive sensor
drivers out of that package keeps the migration contained.

The split is: **`hardware` moves the robot, `sensors` observes the world.**

## Contract

- Topic names follow `docs/interfaces.md`.
- Device paths must be stable udev symlinks, never raw enumeration order
  (LiDAR).
- Ultrasonic `frame_id` values: `ultrasonic_front` / `ultrasonic_rear` appear in
  URDF **only after** `geometry.yaml` front+rear are measured (not null).
  LEFT/RIGHT frames are **not** added for Nav2 Phase 0.5. Do not invent poses.
  See `docs/calibration.md` §4b and `docs/ultrasonic-front-rear-nav2.md`.

## LiDAR

```bash
ros2 launch warotrans_sensors lidar.launch.py
ros2 topic hz /scan
```

## Ultrasonic (MKE-S01 ×4, Pi GPIO)

GPIO library: **python3-lgpio** on `/dev/gpiochip4` (Pi 5 RP1). Do not use
legacy `RPi.GPIO`.

```bash
sudo apt install -y python3-lgpio gpiod
# waro must access gpiochip4 (group dialout on Ubuntu 24.04 Pi image)
```

BCM pin contract (do not change):

| Sensor | TRIG BCM | ECHO BCM |
|---|---:|---:|
| FRONT | 17 | 27 |
| REAR | 22 | 23 |
| LEFT | 24 | 25 |
| RIGHT | 5 | 6 |

Sequential poll: FRONT → RIGHT → REAR → LEFT (`inter_sensor_delay_ms` default **50**).

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch warotrans_sensors ultrasonic.launch.py
# or: ros2 run warotrans_sensors ultrasonic_node
```

Validate (Pi):

```bash
bash ~/warotrans/tools/warotrans-ultrasonic-validate.sh
# or after deploy: bash ~/tools/warotrans-ultrasonic-validate.sh
```

Physical check: flat target at ~0.2 / 0.5 / 1.0 m in front of one sensor at a
time. Expect `range` to track distance; `inf` = timeout / out of range.
Record results in `docs/calibration.md` §4b.

**Mount TF / Nav2:** blocked until `geometry.yaml` ultrasonic x/y/z/yaw are measured
(see STOP in Phase 0.5 plan). Do not invent poses.

Not in full bringup until STEP 6 validation PASS. No motor-stop / Nav2 yet.

## Checkpoint

Driver is done when the topic publishes at a stable rate with valid data, and
unplugging / no-echo produces a meaningful error instead of taking down the
system.
