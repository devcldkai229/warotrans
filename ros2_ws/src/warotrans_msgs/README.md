# warotrans_msgs

Owns **WaroTrans-specific** ROS interfaces only.

Before creating a custom interface, ask:

> Can a standard ROS message already express this contract?

Examples that usually **do not** need custom messages:
- laser scan → `sensor_msgs/msg/LaserScan`
- odometry → `nav_msgs/msg/Odometry`
- velocity → `geometry_msgs/msg/Twist`
- IMU → `sensor_msgs/msg/Imu`
- map → `nav_msgs/msg/OccupancyGrid`

Possible future custom interfaces:
- robot fleet status
- task lifecycle events
- low-level wheel telemetry if a standard message is not suitable

Do not add messages speculatively.
