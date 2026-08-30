# warotrans_bringup

This package is **orchestration only**.

It should eventually provide convenient top-level commands such as:

```bash
ros2 launch warotrans_bringup robot.launch.py
ros2 launch warotrans_bringup mapping.launch.py
ros2 launch warotrans_bringup navigation.launch.py
```

The launch files should include subsystem launch files from their owning packages.

Do not put:
- serial parsing
- odometry math
- SLAM parameters
- Nav2 implementation
- camera processing
- fleet business logic

inside this package.
