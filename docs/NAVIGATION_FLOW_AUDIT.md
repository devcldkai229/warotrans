# WaroTrans — Luồng navigation hiện tại (audit)

> Snapshot kiến trúc navigation để audit / sửa chữa.
> Source of truth kèm theo: `docs/interfaces.md`, `docs/STATUS.md`,
> `docs/calibration.md`, launch/config trong `ros2_ws/src/`.

**Entry point:**

```bash
ros2 launch warotrans_bringup navigation.launch.py map:=$HOME/maps/warotrans.yaml
# tùy chọn: enable_ekf:=true
```

**Không** chạy đồng thời với: `mapping.launch.py` / `demo.launch.py` /
service teleop / `web_teleop_node` (xung đột `/cmd_vel` hoặc TF `map→odom`).

---

## 1. Thứ tự bringup (compose)

File: [`ros2_ws/src/warotrans_bringup/launch/navigation.launch.py`](../ros2_ws/src/warotrans_bringup/launch/navigation.launch.py)

| # | Subsystem | Role |
|---|---|---|
| 1 | `warotrans_description` / `description.launch.py` | URDF → `robot_state_publisher` (TF tĩnh) |
| 2 | `warotrans_hardware` / `hardware.launch.py` | ESP32 bridge + wheel odometry |
| 3 | `imu.launch` + `ekf.launch` | **chỉ khi** `enable_ekf:=true` (default **false**) |
| 4 | `warotrans_sensors` / `lidar.launch.py` | `/scan` |
| 5 | `ultrasonic.launch.py` | default **tắt** (`start_ultrasonic:=false`) |
| 6 | `warotrans_localization` / `localization.launch.py` | map_server + AMCL |
| 7 | **delay 12 s** rồi `warotrans_navigation` | Nav2 lifecycle |
| 8 | `foxglove_bridge` `:8765` | optional |

**Lưu ý audit:** arg `nav_start_delay_sec` có khai báo nhưng `TimerAction` đang
**hard-code `period=12.0`** — đổi launch arg hiện **không** có hiệu lực.

---

## 2. TF tree và ownership (bất biến)

```text
map
 └── odom                 ← AMCL (navigation) — KHÔNG slam_toolbox
      └── base_footprint  ← wheel_odom (default) HOẶC ekf_filter (enable_ekf:=true)
           └── base_link  ← robot_state_publisher
                ├── laser
                ├── imu_link          (z=0.112 m, axes REP-103)
                └── ultrasonic_*      (khi geometry FRONT+REAR đủ số đo)
```

| Transform | Owner |
|---|---|
| `map → odom` | AMCL (`localization.launch.py`) |
| `odom → base_footprint` | `wheel_odom_node` **hoặc** `ekf_filter_node`, không cả hai |
| `base_footprint → base_link` | `robot_state_publisher` |
| `base_link → laser` / `imu_link` | `robot_state_publisher` (URDF / `geometry.yaml`) |

**Invariant:** một TF edge chỉ một publisher. Bật EKF mà quên
`publish_tf:=false` trên wheel → `TF_REPEATED_DATA` / nhảy pose.

---

## 3. Hai chế độ odom

### 3A. Default — `enable_ekf:=false` (Phase 0 đã PASS)

```text
ESP32 serial (/dev/warotrans)
  → esp32_bridge_node
       • publish /wheel_ticks
       • subscribe /cmd_vel → lệnh "V <lin> <ang>"
  → wheel_odom_node
       • publish /odom
       • publish TF odom → base_footprint

RPLIDAR → /scan
map YAML → map_server → /map
AMCL: /scan + TF odom→base → TF map→odom

Nav2:
  odom_topic = /odom
  costmap dùng TF (local: odom, global: map)
  controller → /cmd_vel (geometry_msgs/Twist, không stamped)
            → esp32_bridge
```

### 3B. `enable_ekf:=true` (code landed; runtime DoD chưa PASS)

```text
wheel_odom_node: /odom only (publish_tf:=false)
bno055_node:     /imu/data (frame_id=imu_link, I²C-1 addr 0x29, OPR_MODE=NDOF)
ekf_filter_node (robot_localization):
  fuse /odom (x, y, vx, vy, vyaw) + /imu/data (yaw, yaw_rate)
  → /odometry/filtered
  → TF odom → base_footprint

Nav2 overlay: config/nav2_params_ekf.yaml
  bt_navigator / controller_server / velocity_smoother
  odom_topic = /odometry/filtered
```

AMCL vẫn chỉ sở hữu `map → odom` — không đổi khi bật EKF.

**Mount IMU (operator 2026-10-16):** tâm tầng 2; `imu_x/y=0`, `imu_z=0.112 m`
(base_link → IMU = 112.0 mm); roll/pitch/yaw = 0 (+X Front, +Y Left, +Z Up).
Evidence: `docs/calibration.md` §8.

---

## 4. Luồng goal → bánh xe (Nav2 internal)

```text
RViz "2D Pose Estimate" → /initialpose → AMCL
RViz "Nav2 Goal"        → /goal_pose   → bt_navigator

BT (navigate_to_pose_w_replanning_resilient.xml):
  ComputePathToPose (planner) → FollowPath (controller)
  FollowPath fail → clear local costmap → wait → replan
  (không abort ngay)

planner_server:
  nav2_navfn_planner::NavfnPlanner (Dijkstra)
  tolerance 0.15 m, allow_unknown: true

controller_server @ 10 Hz:
  Regulated Pure Pursuit (FollowPath)
  - desired_linear_vel: 0.10 m/s (PROVISIONAL)
  - lookahead: 0.18–0.45 m (nominal 0.25)
  - use_rotate_to_heading: true (≥ ~45° / 0.785 rad)
  - use_collision_detection: true
  - progress_checker: 0.08 m / 45 s
  - goal: xy_tol 0.15 m, yaw_tol 0.25 rad
  → publish /cmd_vel  (enable_stamped_cmd_vel: false)

esp32_bridge:
  open-loop velocity command (chưa closed-loop PID bánh — Phase 10)
```

**Không** dùng `velocity_smoother` / `/cmd_vel_nav` trong stack Phase 0 đã PASS
(interfaces: controller → `/cmd_vel` trực tiếp). Node `velocity_smoother` có
trong `nav2_params.yaml` nhưng **không** nằm lifecycle navigation hiện tại.

Params chính: [`nav2_params.yaml`](../ros2_ws/src/warotrans_navigation/config/nav2_params.yaml).

---

## 5. Costmap

| | Local | Global |
|---|---|---|
| Frame | `odom` | `map` |
| Base | `base_footprint` | `base_footprint` |
| Layers (default outdoor) | Voxel + Inflation; keepout plugin **disabled** (outdoor overlay) | Static + Obstacle + Inflation; keepout/speed **disabled** |
| Filters Phase 3 | `enable_costmap_filters:=true` + mask YAML | Keepout + Speed |

- Footprint: `0.32 × 0.26` m.
- Collision Monitor / Range layer ultrasonic: **tắt** trong bringup mặc định
  (Phase 0.5 — xem `docs/ultrasonic-front-rear-nav2.md`).

---

## 6. Topic xương sống khi đang navigate

| Topic | Ai ghi | Ai đọc |
|---|---|---|
| `/wheel_ticks` | `esp32_bridge_node` | `wheel_odom_node` |
| `/odom` | `wheel_odom_node` | EKF (nếu bật) / Nav2 (nếu EKF tắt) |
| `/imu/data` | `bno055_node` | EKF |
| `/odometry/filtered` | `ekf_filter_node` | Nav2 (nếu EKF bật) |
| `/scan` | LiDAR driver | AMCL + costmaps |
| `/map` | `map_server` | AMCL + global costmap |
| `/cmd_vel` | `controller_server` | `esp32_bridge_node` **only** (không teleop) |
| `/plan`, `/local_plan` | planner / controller | RViz |
| `/amcl_pose`, `/particle_cloud` | AMCL | RViz |
| `/initialpose` | RViz | AMCL |
| `/goal_pose` | RViz | BT navigator |

---

## 7. Sơ đồ tổng thể

```text
                    ┌──────────── map YAML ────────────┐
                    ▼                                  │
               map_server → /map                       │
                    │                                  │
 /scan ◄── LiDAR    ▼                                  │
    └──────────► AMCL ──TF──► map → odom               │
                                                       │
 /wheel_ticks ◄── esp32_bridge ◄── /cmd_vel ◄── Nav2 ──┘
       │                              ▲
       ▼                              │
  wheel_odom → /odom ──┐              │
                       ├──► EKF* ──► /odometry/filtered* + TF odom→base*
 /imu/data ◄── BNO055* ┘              │
                                      │
                         costmaps + RPP FollowPath
                         planner NavFn (map)

* chỉ khi enable_ekf:=true; mặc định wheel_odom tự publish TF odom→base
  và Nav2 đọc /odom.
```

---

## 8. Điểm audit / rủi ro (ưu tiên sửa)

1. **`enable_ekf` default false** — IMU+EKF chưa là đường navigate production;
   cần DoD runtime trước khi đổi baseline.
2. **`nav_start_delay_sec` bị ignore** — Timer hard-code 12 s trong bringup.
3. **Open-loop motor** — RPP giả định đáp ứng `cmd_vel`; trượt skid-steer vẫn có.
   EKF chỉ cải thiện ước lượng yaw, **không** đóng vòng tốc độ bánh.
4. **EKF covariance PROVISIONAL** — chưa characterization static/spin trên robot
   (`ekf.yaml`, `bno055.yaml`).
5. **BNO055 absolute yaw vs wheel yaw** — lần đầu bật EKF cần kiểm tra không
   nhảy heading / đảo chiều quay.
6. **Teleop vs Nav2** — bắt buộc `stop warotrans-teleop` trước khi navigate.
7. **Ultrasonic / Collision Monitor** — chưa trong luồng an toàn mặc định.
8. **Map path** — mặc định `~/maps/warotrans.yaml`; outdoor / Zenoh dùng map
   và script riêng (xem `docs/STATUS.md`, `docs/nav2-rviz-operator.md`).

---

## 9. Checklist audit nhanh trên robot

```bash
# Dừng teleop trước
sudo systemctl stop warotrans-teleop   # nếu đang enable

# Bringup (chọn một)
ros2 launch warotrans_bringup navigation.launch.py map:=$HOME/maps/warotrans.yaml
# hoặc có IMU yaw:
# ros2 launch warotrans_bringup navigation.launch.py map:=$HOME/maps/warotrans.yaml enable_ekf:=true

# TF — một publisher mỗi edge
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint

# Rate
ros2 topic hz /scan
ros2 topic hz /odom
ros2 topic hz /cmd_vel
# nếu enable_ekf:=true:
ros2 topic hz /imu/data
ros2 topic hz /odometry/filtered

# Ai publish /tf (debug conflict)
ros2 topic info /tf -v
```

### DoD EKF (khi bật)

- [ ] `/imu/data` rate ổn; `frame_id=imu_link`
- [ ] Đứng yên: `|acc| ≈ 9.8`; xoay tay: yaw đúng chiều (không đảo)
- [ ] Một publisher `odom→base_footprint` (không `TF_REPEATED_DATA`)
- [ ] `/odometry/filtered` không nhảy bước lớn
- [ ] Goal RViz: path + FollowPath ổn hơn wheel-only khi trượt (so sánh tay)

---

## 10. File liên quan

| File | Việc |
|---|---|
| `ros2_ws/src/warotrans_bringup/launch/navigation.launch.py` | Compose bringup |
| `ros2_ws/src/warotrans_navigation/launch/navigation.launch.py` | Nav2 nodes + overlays |
| `ros2_ws/src/warotrans_navigation/config/nav2_params.yaml` | Params Phase 0 |
| `ros2_ws/src/warotrans_navigation/config/nav2_params_outdoor.yaml` | Tắt keepout/speed |
| `ros2_ws/src/warotrans_navigation/config/nav2_params_ekf.yaml` | `odom_topic` → filtered |
| `ros2_ws/src/warotrans_localization/launch/localization.launch.py` | map_server + AMCL |
| `ros2_ws/src/warotrans_localization/config/ekf.yaml` | EKF fuse |
| `ros2_ws/src/warotrans_localization/launch/ekf.launch.py` | EKF node |
| `ros2_ws/src/warotrans_sensors/launch/imu.launch.py` | BNO055 |
| `ros2_ws/src/warotrans_hardware/launch/hardware.launch.py` | bridge + odom + `publish_tf` |
| `docs/interfaces.md` | Hợp đồng topic/TF |
| `docs/STATUS.md` | Trạng thái PASS / chưa PASS |
| `docs/calibration.md` §8 | IMU pose evidence |
| `docs/nav2-rviz-operator.md` | Runbook operator RViz |
