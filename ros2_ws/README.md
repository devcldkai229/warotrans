# WaroTrans ROS 2 workspace

Source packages live under `src/`.

Build:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Useful checks:

```bash
colcon list
colcon test
colcon test-result --verbose
```

Do not store runtime maps, rosbags, or logs inside `src/`.
