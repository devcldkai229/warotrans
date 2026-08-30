# warotrans_sensors

## Responsibility

Sensor **driver layer**. Turns physical devices into ROS topics.

Owns:
- LiDAR driver launch and driver configuration;
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

- Sensor `frame_id` must match a link that `warotrans_description` actually
  publishes. Do not invent a frame name here.
- Topic names follow `docs/interfaces.md`.
- Device paths must be stable udev symlinks, never raw enumeration order.

## Checkpoint

Driver is done when the topic publishes at a stable rate with valid data, and
unplugging the device produces a meaningful error instead of taking down the
system.
