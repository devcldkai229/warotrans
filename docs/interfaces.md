# WaroTrans — Interface Contracts

> File này mô tả **hợp đồng giữa subsystem**. Implementation có thể refactor; contract không được breaking-change âm thầm.

## 1. TF contract

```text
map
 └── odom
      └── base_footprint
           └── base_link
                ├── laser
                └── camera_link   # chỉ tồn tại sau khi camera pose được xác nhận
```

Ownership:

| Transform | Owner |
|---|---|
| `map → odom` | SLAM Toolbox (mapping) **HOẶC** AMCL (navigation), **không đồng thời** |
| `odom → base_footprint` | wheel odom **hoặc** EKF, không phải cả hai |
| `base_footprint → base_link` | `robot_state_publisher` từ URDF |
| `base_link → laser` | `robot_state_publisher` từ URDF |
| `base_link → camera_link` | `robot_state_publisher` từ URDF |

**Invariant:** một transform cụ thể chỉ có một publisher.

### Mode: Mapping (Phase 8)

- `map → odom` thuộc `slam_toolbox` chạy từ `warotrans_slam/launch/slam.launch.py`
- Launch: `ros2 launch warotrans_bringup mapping.launch.py`
- Kết thúc mapping, dùng `ros2 run nav2_map_server map_saver_cli -f ~/maps/<name>` để lưu map

### Mode: Navigation (Phase 9 / Phase 0 PASS — FROZEN 2026-09-11)

- `map → odom` thuộc `nav2_amcl` chạy từ `warotrans_localization/launch/localization.launch.py`
- Launch (ổn định): `bash ~/tools/warotrans-nav-start.sh ~/maps/warotrans.yaml`
  hoặc `ros2 launch warotrans_bringup navigation.launch.py map:=$HOME/maps/warotrans.yaml`
- **PHẢI TẮT** `slam_toolbox` khi chạy navigation
- **PHẢI TẮT** `web_teleop_node` khi chạy navigation (xung đột `/cmd_vel`)
- Controller publish **`/cmd_vel`** trực tiếp (`geometry_msgs/Twist`); **không** dùng
  `velocity_smoother` / `/cmd_vel_nav` trong stack Phase 0 đã PASS.

---

## 2. ROS topic contract

| Topic | Type | Direction | Status |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR → SLAM/Nav2 | **stable (Phase 0)** |
| `/ultrasonic/front` | `sensor_msgs/msg/Range` | ultrasonic → monitor → (sau đo mount) Collision Monitor | target; **Nav2 Phase 0.5** |
| `/ultrasonic/rear` | `sensor_msgs/msg/Range` | ultrasonic → monitor → (sau đo mount) Collision Monitor | target; **Nav2 Phase 0.5** |
| `/ultrasonic/left` | `sensor_msgs/msg/Range` | ultrasonic → monitor only | target; **không** Nav2 Phase 0.5 |
| `/ultrasonic/right` | `sensor_msgs/msg/Range` | ultrasonic → monitor only | target; **không** Nav2 Phase 0.5 |
| `/wheel_ticks` | `warotrans_msgs/msg/WheelTicks` | ESP32 bridge → wheel odometry | target |

Owner của mỗi topic là package sinh ra nó: `/scan`, `/ultrasonic/*` và `/imu/data` thuộc
`warotrans_sensors`; `/wheel_ticks`, `/odom` và forwarding `/cmd_vel` thuộc `warotrans_hardware`;
`/motor_command_state` thuộc `warotrans_teleop`; `/map` và `/map_metadata` thuộc
`warotrans_slam` khi mapping, hoặc map server khi chạy static map;
`/camera/image_raw` thuộc `warotrans_perception`.

`/ultrasonic/*` hiện là **monitoring only** (không dừng motor). Collision Monitor /
RangeSensorLayer **chưa bật**. Sau khi đo mount **FRONT + REAR** (xem
`docs/calibration.md` §4b và `docs/ultrasonic-front-rear-nav2.md`), chỉ hai
topic đó vào Nav2 safety. LEFT/RIGHT không wire Nav2 Phase 0.5.
`frame_id` `ultrasonic_front` / `ultrasonic_rear` chỉ có trong URDF khi
`geometry.yaml` front+rear đủ số đo.

| `/odom` | `nav_msgs/msg/Odometry` | base → localization/SLAM/Nav2 | **stable (Phase 0)** |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | teleop/Nav2 → `esp32_bridge_node` | **stable (Phase 0)** |
| `/motor_command_state` | `warotrans_msgs/msg/MotorCommandState` | teleop → UI/debug | target |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU → EKF | future |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM/map server → clients | target |
| `/map_metadata` | `nav_msgs/msg/MapMetaData` | SLAM/map server → clients | target |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | camera → UI/perception | future |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | robot → operator | future |

### Nav2 topics (Phase 9 — Phase 0 PASS)

| Topic | Type | Direction | Status |
|---|---|---|---|
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | RViz → Nav2 | **stable (Phase 0)** |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | RViz → AMCL | **stable (Phase 0)** |
| `/plan` | `nav_msgs/msg/Path` | planner_server → RViz / clients | **stable (Phase 0)** |
| `/local_plan` | `nav_msgs/msg/Path` | controller_server → RViz | debug |
| `/cmd_vel_nav` | `geometry_msgs/msg/Twist` | (unused in Phase 0) | **not used** — controller → `/cmd_vel` |
| `/particle_cloud` | `nav2_msgs/msg/ParticleCloud` | AMCL → RViz | debug |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | AMCL → localization clients | **stable (Phase 0)** |
| `/local_costmap/costmap` | `nav2_msgs/msg/Costmap` | controller_server → RViz | debug |
| `/global_costmap/costmap` | `nav2_msgs/msg/Costmap` | planner_server → RViz | debug |

### Nav2 actions (Phase 9 — Phase 0 PASS)

| Action | Type | Status |
|---|---|---|
| `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | **stable (Phase 0)** |
| `/navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | **Phase 3** (Lane sequence) |

### Costmap filters (Phase 3)

| Topic / node | Role |
|---|---|
| `/keepout_filter_mask` | KEEP_OUT OccupancyGrid mask |
| `/keepout_costmap_filter_info` | KeepoutFilter metadata (type=0) |
| `/speed_filter_mask` | SPEED_LIMIT OccupancyGrid mask |
| `/speed_costmap_filter_info` | SpeedFilter metadata (type=2 absolute, base=0, mult=0.01) |
| `/speed_limit` | SpeedFilter → controller_server |

Mask files (runtime): `~/maps/warotrans_keepout.yaml`, `~/maps/warotrans_speed.yaml`  
Created by `tools/warotrans-ensure-filter-masks.sh`; updated by demo `POST /api/filters/sync`.

### Demo HTTP gateway

| HTTP | Role |
|---|---|
| `GET /api/map/meta`, `GET /api/map/image.png` | Occupancy từ `/map` |
| `CRUD /api/endpoints` | Endpoint RAM session |
| `CRUD /api/lanes`, `/api/zones` | Semantic map (Phase 2) |
| `GET/PUT /api/semantic-map` | Export/import JSON |
| `POST /api/navigate/{id}` | → `/navigate_to_pose` |
| `POST /api/navigate/lanes` | → `/navigate_through_poses` (Phase 3) |
| `POST /api/filters/sync` | KEEP_OUT/SPEED masks → LoadMap |
| `WS /ws` | nav + robot + path + speed_limit |

---

### `/wheel_ticks` message

`warotrans_msgs/msg/WheelTicks` carries cumulative encoder values:

```text
int64  left_ticks
int64  right_ticks
uint32 mcu_millis
```

`mcu_millis` is retained for observability and restart detection. ROS nodes use
ROS time for message and TF timestamps; MCU time is not copied into ROS time.

### `/motor_command_state` message

`warotrans_msgs/msg/MotorCommandState` exposes **computed PWM command values**
for commissioning UI/debug. These are not measured actuator feedback.

```text
string direction
float32 speed_level_percent
float32 linear_x
float32 angular_z
float32 left_pwm_percent
float32 right_pwm_percent
int32 left_pwm_raw
int32 right_pwm_raw
int32 pwm_raw_max
float32 pwm_limit_percent
bool command_active
uint32 last_command_age_ms
```

## 3. ESP32 ↔ Pi serial contract

Contract hiện tại của firmware prototype:

```text
ESP32 → Pi
O <tickL> <tickR> <millis>\n

Pi → ESP32
V <linear_mps> <angular_rps>\n
R\n
```

Quy tắc:

- Giữ text protocol để debug được bằng terminal.
- Thêm command mới bằng prefix mới, không đổi nghĩa command cũ.
- Breaking change phải sửa firmware + Pi + tests + tài liệu trong cùng một change set.
- Watchdog không được phụ thuộc vào việc parser nhận được packet "gần đúng".

**Firmware version đang nạp trên board phải được xác nhận trước khi coi contract này là VERIFIED.**

---

## 4. Coordinate conventions

Theo ROS REP-103:

```text
+X = phía trước robot
+Y = bên trái robot
+Z = phía trên
positive yaw = quay ngược chiều kim đồng hồ khi nhìn từ +Z
```

Encoder sign convention mục tiêu:

```text
robot đi thẳng về +X
→ tick trái tăng
→ tick phải tăng
```

Nếu wiring làm dấu ngược, normalize ở đúng một tầng và ghi lại.

---

## 4b. Power & ground contract

Đây là contract điện, không phải phần mềm, nhưng vi phạm nó biểu hiện thành lỗi
phần mềm khó chẩn đoán (tick encoder nhảy, serial rớt, LiDAR reset).

**Quy tắc:**

1. **Mass chung là bắt buộc.** ESP32, MDD10A và Pi phải chung một điểm GND.
   Không chung mass thì mức logic PWM/DIR không xác định.

2. **ESP32 chỉ được cấp nguồn từ MỘT nguồn tại một thời điểm.** Nếu ESP32 nối USB
   tới Pi để truyền tick, nó đã có 5 V từ cổng USB đó. Đồng thời cấp thêm 5 V từ
   buck vào chân 5V là cấp hai nguồn song song — chỉ làm khi đã xác nhận board có
   diode chống ngược dòng. Chọn một, và ghi lựa chọn vào `docs/calibration.md`.

3. **Cảnh giác vòng lặp mass.** Khi ESP32 nối USB tới Pi, GND của ESP32 về pin theo
   hai đường: qua USB → Pi → nguồn, và qua cáp tín hiệu → MDD10A → bus. Dây GND từ
   MDD10A về bus phải **ngắn và dày** để là đường trở kháng thấp hơn hẳn. Nếu không,
   một phần dòng motor sẽ đi qua cáp USB.

4. **Ngân sách dòng USB của Pi phải được tính, không được giả định.** Xem
   `MASTER_ENGINEERING_GUIDE.md` mục 6.1.

5. **Điện áp cấp cho encoder quyết định mức logic của kênh A/B.** Cấp encoder ở mức
   nào thì ngõ ra ở mức đó. Nếu mức đó cao hơn mức logic an toàn của MCU thì phải có
   mạch hạ mức, không nối thẳng. Điện áp thật phải được **đo** và ghi vào
   `docs/calibration.md` mục 6 trước khi nối vào GPIO.

Mọi thay đổi ở mục này phải được đo lại, không suy luận.

---

## 5. Fleet contract — chưa khóa protocol transport

Core domain không phụ thuộc MQTT/VDA5050/WebSocket cụ thể.

### Backend → Robot

```text
task_id
robot_id / intended robot
pickup or navigation goal
dropoff or navigation goal
action: assign / pause / resume / cancel
```

### Robot → Backend

```text
robot_id
online state
pose
battery state
current task
navigation state
error state
task result
```

Transport/protocol cụ thể phải được quyết định bằng ADR. **VDA5050 là một option tương lai, không phải mặc định bắt buộc.**
