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
| `map → odom` | SLAM Toolbox hoặc localization stack |
| `odom → base_footprint` | wheel odom **hoặc** EKF, không phải cả hai |
| `base_footprint → base_link` | `robot_state_publisher` từ URDF |
| `base_link → laser` | `robot_state_publisher` từ URDF |
| `base_link → camera_link` | `robot_state_publisher` từ URDF |

**Invariant:** một transform cụ thể chỉ có một publisher.

---

## 2. ROS topic contract

| Topic | Type | Direction | Status |
|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | LiDAR → SLAM/Nav2 | target |

Owner của mỗi topic là package sinh ra nó: `/scan` và `/imu/data` thuộc
`warotrans_sensors`; `/odom` và forwarding `/cmd_vel` thuộc `warotrans_hardware`;
`/map` thuộc `warotrans_slam` hoặc map server; `/camera/image_raw` thuộc
`warotrans_perception`.

| `/odom` | `nav_msgs/msg/Odometry` | base → localization/SLAM/Nav2 | target |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Nav2/teleop → base | target |
| `/imu/data` | `sensor_msgs/msg/Imu` | IMU → EKF | future |
| `/map` | `nav_msgs/msg/OccupancyGrid` | SLAM/map server → clients | target |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | camera → UI/perception | future |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | robot → operator | future |

Topic name thay đổi phải được review như breaking change.

---

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
