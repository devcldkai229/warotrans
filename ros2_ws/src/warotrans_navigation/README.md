# warotrans_navigation

Owns Nav2 configuration and navigation assets for WaroTrans.

## Components

- **Controller Server** — Regulated Pure Pursuit local planner
- **Planner Server** — NavFn global planner
- **BT Navigator** — Behavior tree for navigation actions
- **Behavior Server** — Recovery behaviors (spin, backup, wait)
- **Velocity Smoother** — Smooths velocity commands to `/cmd_vel`
- **Smoother Server** — Path smoothing

## Configuration Files

| File | Purpose |
|---|---|
| `config/nav2_params.yaml` | Full Nav2 stack parameters (outdoor: filters off) |
| `config/nav2_params_outdoor.yaml` | Overlay — keep filters [] |
| `config/nav2_params_filters_on.yaml` | Overlay — Phase 3 keepout/speed ON |
| `behavior_trees/navigate_to_pose_w_replanning_resilient.xml` | FollowPath fail → clear/wait/replan |
| `rviz/nav2_warotrans.rviz` | RViz config for navigation visualization |

Outdoor / Zenoh: `enable_costmap_filters:=false` (default).  
Phase 3 demo: `ENABLE_COSTMAP_FILTERS=1` or `enable_costmap_filters:=true`.

## Usage

### Launch navigation (standalone)

```bash
ros2 launch warotrans_navigation navigation.launch.py
```

### Launch full navigation stack (recommended)

```bash
ros2 launch warotrans_bringup navigation.launch.py map:=$HOME/maps/warotrans.yaml
```

This includes: description + hardware + lidar + localization + navigation + foxglove.

### Send navigation goal from RViz

1. Open RViz with Nav2 config:
   ```bash
   rviz2 -d ~/ros2_ws/src/warotrans_navigation/rviz/nav2_warotrans.rviz
   ```
2. Set initial pose with "2D Pose Estimate" tool
3. Click "Nav2 Goal" tool
4. Click on map to set destination

### Send navigation goal from command line

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

### Check lifecycle state

```bash
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /behavior_server
ros2 lifecycle get /velocity_smoother
```

All should be `active`.

## DoD (Definition of Done)

- [ ] All lifecycle nodes in `active` state
- [ ] `/plan` topic publishes when goal is set
- [ ] Robot moves towards goal following path
- [ ] ≥5 consecutive NavigateToPose goals SUCCEEDED
- [ ] Position error at goal < 0.2m (measured with ruler)

## Prerequisites

- ✓ Validated map (from Phase 8 SLAM)
- ✓ Stable `/scan` (RPLIDAR working)
- ✓ Stable `/odom` (wheel odometry calibrated)
- ✓ Valid TF tree (`map → odom → base_footprint → base_link → laser`)
- ✓ Localization working (AMCL from Phase 9)
- ✓ Reliable `/cmd_vel` path to ESP32 (watchdog active)

## Robot Footprint

```text
Chassis: 0.320m × 0.260m
Footprint: [[0.16, 0.13], [0.16, -0.13], [-0.16, -0.13], [-0.16, 0.13]]
```

## Velocity Limits (PROVISIONAL)

| Parameter | Value | Status |
|---|---|---|
| `max_linear` | 0.128 m/s | PROVISIONAL — needs measurement |
| `max_angular` | 0.56 rad/s | PROVISIONAL — needs measurement |
| Nav2 `max_vel_x` | 0.10 m/s | 80% of max_linear |
| Nav2 `max_vel_theta` | 0.45 rad/s | 80% of max_angular |

## Nav2 does NOT control

- MDD10A GPIO/PWM details (handled by firmware + esp32_bridge_node)
- Encoder ticks (handled by wheel_odom_node)
- LiDAR driver (handled by rplidar_ros)
