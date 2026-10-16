# WaroTrans — Audit trạng thái hiện tại (2026-10-16)

> Audit read-only: không sửa code. Nguồn: repo, git history, `docs/STATUS.md`,
> `docs/calibration.md`, và output runtime quan sát được trong session trước.
> Lúc audit, Pi **không truy cập được** (`warotrans.local` → "Host does not exist"),
> nên mục cần kiểm tra live được đánh dấu UNKNOWN.

## Nguyên tắc đánh giá

Đồng ý với nguyên tắc:

> "Một feature chỉ PASS khi Definition of Done có runtime evidence. Source code tồn tại là chưa đủ."

Bổ sung một điều kiện: **evidence phải thuộc đúng revision/config đang deploy**.
PASS ghi nhận trước khi config Nav2 thay đổi là lịch sử, không phải trạng thái hiện tại.

### Mốc thời gian quan trọng

- **2026-09-11**: PASS Nav2 real-robot cuối cùng (indoor map, operator RViz + CLI SUCCEEDED).
- **2026-09-12**: `nav2_params.yaml` đổi (rotate-to-heading, lookahead, inflation,
  collision detection, filters). STATUS tự ghi "chưa claim motion PASS".
- **2026-09-30**: commit `1ba075d` ("fix robot cannot move in intersection") đổi lại Nav2 config.
- **2026-10-16**: launch/hardware/odom/IMU/EKF changes đang **staged, chưa commit**
  (`enable_ekf`, `publish_tf`, `bno055_node`, `ekf.yaml`, `nav2_params_ekf.yaml`).
- Từ 2026-09-11 đến nay: **không có goal SUCCEEDED nào được ghi nhận**; operator báo
  lỗi mid-path stall / giao lộ / góc kệ.

---

## 1. CURRENT ENGINEERING STATE

| Subsystem | DoD | Status | Evidence | Missing |
|---|---|---|---|---|
| A. Motor / ESP32 bridge | `/cmd_vel` làm bánh quay; watchdog dừng khi mất lệnh | PASS (chỉ motion) | STATUS: robot chạy dưới Nav2 09-11; web teleop operator lái tháng này | Watchdog / deadman chưa verify (Phase 7.5); firmware reflash còn pending |
| B. Wheel encoders | Đếm đúng, đúng dấu hai bên | PASS (counts) | `calibration.md`: 10 rev → 2478.1 ticks; 2.00 m → ΔL=23934, ΔR=24172 | Bài test dấu riêng + đo tín hiệu điện chưa làm |
| C. Wheel odometry | `/odom` rate ổn, hằng số calibrated, sai số đo được | PASS | STATUS: `/odom` ~50 Hz; radius/separation calibrated 09-07 | Bài thẳng drift yaw ~29°, chưa test lại |
| D. RPLIDAR | `/scan` ổn định đúng rate | PASS | STATUS: ~7 Hz, `/dev/rplidar` | — |
| E. TF tree | `map→odom→base_footprint→base_link→laser`, mỗi edge một publisher | PASS (cây Phase 0) | STATUS 09-11 + acceptance matrix 1–4 | Edge mới `imu_link` chưa thấy runtime; lidar yaw "chờ xác nhận lại sau restart" |
| F. IMU / BNO055 | `/imu/data` đúng rate/frame/trục; pass bài yaw | NOT TESTED | Chỉ I²C: `0x29`, CHIP_ID `0xA0`; `bno055_node` start, log "NDOF ready" | `/imu/data` chưa đo; nghi ngờ trục (xem §4) |
| G. SLAM | Map đạt quality gate | NOT TESTED | Có map dùng được (09-07) | Quality gate formal chưa đạt; matrix 5–6 NOT TESTED |
| H. Map saving | `.pgm` + `.yaml` lưu và load lại được | PASS | `~/maps/warotrans.{pgm,yaml}` trên Pi (09-07); đã copy vào repo | — |
| I. Saved-map loading | `map_server` phát `/map` | PASS | Matrix 7–17; WSL thấy `/map` 09-12 | — |
| J. AMCL | `map→odom`; pose hội tụ sau 2D Pose Estimate | PASS | Matrix 09-11; `/amcl_pose` trên WSL 09-12 | Boot pose provisional (bắt buộc estimate tay) |
| K. Nav2 lifecycle | Mọi server Nav2 ACTIVE | PASS | 09-11; STATUS 09-12: ACTIVE sau hotfix restart | Chưa check lại sau commit 09-30 / staged changes |
| L. Global costmap | Static + obstacle + inflation active; LiDAR obstacle hiện | PASS (config 09-11) | Matrix 7–17 | Inflation/filter đổi 09-12, chưa kiểm tra lại bằng mắt |
| M. Local costmap | Voxel mark/clear từ `/scan` | PASS (config 09-11) | Như trên | Như trên |
| N. NavigateToPose | Goal accepted, kết thúc SUCCEEDED | NOT TESTED (hiện tại) | Lịch sử: SUCCEEDED 09-11 (CLI + RViz), Demo 1 API | Chưa có sau khi đổi config; báo cáo sau đó là lỗi |
| O. Global path | `/plan` hợp lệ | NOT TESTED (hiện tại) | Lịch sử 09-11; Demo 3 gặp "Failed to create plan" | Chạy lại trên config hiện tại |
| P. Controller output | `/cmd_vel` khác 0 khi follow path | NOT TESTED (hiện tại) | Lịch sử 09-11 | Params RPP đã đổi sau đó |
| Q. `/cmd_vel` → ESP32 | Bridge là subscriber duy nhất; bánh phản hồi | PASS | 09-11 (sole subscriber); teleop lái được | — |
| R. Robot thật tới goal | Tới nơi thật + SUCCEEDED | NOT TESTED (hiện tại) | Chỉ lịch sử 09-11 | Hành vi gần nhất được biết là lỗi |
| S. Ultrasonic | 4 Range topic, đọc đúng ở 0.2 / 0.5 / 1.0 m | NOT TESTED | Có code node; mount FRONT/REAR đã đo | Bảng range validation trong `calibration.md` trống |
| T. Low-obstacle avoidance | Vật thấp thật làm đổi costmap / hành vi robot | BLOCKED | — | Không có RangeSensorLayer; Collision Monitor không wire; blocked bởi S |

---

## 2. PHASE 0 VERDICT: **NOT TESTED** (với code hiện tại)

**Đã chứng minh nhưng stale:** STATUS ghi PASS end-to-end 2026-09-11 với indoor map:
goal → `/plan` → `/cmd_vel` → ESP32 → chuyển động → SUCCEEDED. Evidence là ghi chú
operator + kết quả CLI. Không giữ raw log, chưa tag git.

**Kết quả WSL (thấy `/map`, `/scan`, `/tf`, `/amcl_pose` qua Zenoh) + load saved map**
chỉ chứng minh tới bước localization: D, E, H, I, J và network.

**Chưa chứng minh trên config hiện tại:**

- NavigateToPose accepted
- global path hợp lệ
- controller active
- `/cmd_vel` khác 0
- robot thật di chuyển
- robot tới goal
- NavigateToPose trả về SUCCEEDED

Tức là toàn bộ đoạn từ goal xuống bánh xe.

**Vì sao PASS 09-11 không còn tính:** config controller, costmap, BT và launch đã
đổi; chưa chạy goal nào trên config hiện tại.

---

## 3. IMU STATUS

| Câu hỏi | Trả lời |
|---|---|
| Đã implement? | Code driver có (`warotrans_sensors/bno055_node.py`, staged, chưa commit). Topic runtime chưa verify |
| Nối vào đâu | Raspberry Pi I²C-1 (pin 1/3/5/9), **không** phải ESP32. Đã xác nhận bằng I²C probe |
| Driver | `bno055_node`: đọc raw `/dev/i2c-1`, `OPR_MODE=NDOF` |
| Topic / type / frame | `/imu/data`, `sensor_msgs/Imu`, `frame_id=imu_link` (theo code; chưa quan sát) |
| TF `imu_link` | Trong URDF tại (0, 0, 0.112), rpy 0. Chưa thấy runtime |
| Có node nào dùng? | Chỉ `ekf_filter_node` khi `enable_ekf:=true`. Default false → hiện **không ai dùng** |
| `robot_localization` | Đã cấu hình (`ekf.yaml`, `ekf.launch.py`). **Cài trên Pi: UNKNOWN** (`package.xml` khai báo; chưa kiểm tra) |
| Fuse vào `/odom`? | Không. `/odom` chỉ từ bánh. EKF xuất ra `/odometry/filtered` |
| Nav2 phụ thuộc IMU? | Không. Default dùng `/odom`; overlay EKF chỉ áp khi `enable_ekf:=true` |
| SLAM phụ thuộc IMU? | Không |

---

## 4. Rủi ro trong code IMU/EKF

(Code này do AI viết trong session trước — tự audit.)

1. **Lệch trục.** Lần đọc accelerometer duy nhất cho X ≈ 9.39, Y ≈ 1.97,
   Z ≈ 0.33 m/s² → trọng lực nằm trên trục X, mâu thuẫn với mount "Z up, không xoay".
   Có thể lúc đó chưa gắn lên robot, nhưng **bắt buộc kiểm tra lại**.
2. **Nguồn yaw.** NDOF dùng magnetometer → heading tuyệt đối theo từ trường; trong
   nhà, gần motor, heading có thể nhảy.
3. **Input EKF không nhất quán.** `ekf.yaml` fuse x/y tuyệt đối từ wheel odom cùng yaw
   tuyệt đối IMU (`imu0_relative: false`). Hai nguồn này lệch nhau về heading.
4. **Covariance bằng 0.** `wheel_odom_node` không set covariance (toàn 0) → EKF gần
   như bỏ qua trọng số IMU.

Không rủi ro nào ảnh hưởng đường mặc định `enable_ekf:=false`.

---

## 5. IMU TEST COMMANDS

Chạy trên Pi, robot đứng yên trên sàn, **không** chạy Nav2:

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch warotrans_sensors imu.launch.py &
ros2 launch warotrans_description description.launch.py &
```

| # | Test | Lệnh | Kỳ vọng |
|---|---|---|---|
| 1 | Topic discovery | `ros2 topic list \| grep imu` | `/imu/data` |
| 2 | Message type | `ros2 topic info /imu/data` | `sensor_msgs/msg/Imu`, publisher count 1 |
| 3 | Tần số | `ros2 topic hz /imu/data` (30 s) | ~50 Hz, jitter thấp |
| 4 | Orientation | `ros2 topic echo /imu/data --field orientation` | Quaternion đơn vị; robot phẳng → x, y ≈ 0 |
| 5 | Angular velocity | `ros2 topic echo /imu/data --field angular_velocity` | Đứng yên: mỗi trục trong ±0.01 rad/s |
| 6 | Linear acceleration | `ros2 topic echo /imu/data --field linear_acceleration` | Phẳng, Z-up: **z ≈ +9.8**, x, y ≈ 0 (±0.3). Nếu x ≈ 9.8 → sai mount/trục, **dừng lại sửa trước** |
| 7 | frame_id | `ros2 topic echo /imu/data --field header.frame_id --once` | `imu_link` |
| 8 | TF | `ros2 run tf2_ros tf2_echo base_link imu_link` | Translation (0, 0, 0.112), rotation identity |

Test 9–12 dùng lệnh in yaw live:

```bash
python3 -c "import rclpy,math;from sensor_msgs.msg import Imu;rclpy.init();n=rclpy.create_node('yaw');n.create_subscription(Imu,'/imu/data',lambda m:print(f\"yaw={math.degrees(math.atan2(2*(m.orientation.w*m.orientation.z+m.orientation.x*m.orientation.y),1-2*(m.orientation.y**2+m.orientation.z**2))):7.1f} gz={m.angular_velocity.z:+.3f}\"),10);rclpy.spin(n)"
```

| # | Test | Thao tác | Kỳ vọng |
|---|---|---|---|
| 9 | Đứng yên | Quan sát 60 s | Yaw drift < ~1°. NDOF: nhảy lúc đầu có thể do magnetometer chưa calibrate |
| 10 | +90° yaw | Xoay tay ngược chiều kim đồng hồ (rẽ trái, nhìn từ trên), so với vạch băng keo | Yaw tăng ~+90° (±5°), `gz` dương khi xoay |
| 11 | −90° yaw | Xoay cùng chiều kim đồng hồ | Yaw giảm ~90°, `gz` âm |
| 12 | 360° quay về | Quay đủ một vòng về vạch | Yaw về trong ±5° so với lúc đầu, có một lần wrap qua ±180° |

Ghi kết quả vào `docs/calibration.md` §8.

---

## 6. CÓ NÊN BẮT BUỘC IMU NGAY BÂY GIỜ?: **KHÔNG**

Chọn phương án B: chứng minh LiDAR + wheel odom + AMCL + Nav2 trước, validate/fuse IMU sau.

- Chuỗi này đã PASS một lần **không cần IMU**; contract Phase 0 đóng băng để wheel odom
  sở hữu `odom→base_footprint`.
- Các lỗi báo sau đó (mid-path stall, giao lộ, góc kệ) là vấn đề controller / costmap /
  motor open-loop. IMU không sửa được các lỗi đó.
- Bật EKF thêm bốn biến cùng lúc: đổi owner TF edge, yaw từ magnetometer, covariance
  chưa tune, và lệch trục ở §4.
- Debug cả hai cùng lúc thì không thể quy lỗi về một nguyên nhân. Validate IMU riêng
  (test §5), rồi fuse sau như một thay đổi tách biệt.

---

## 7. ULTRASONIC STATUS

| Check | Kết quả |
|---|---|
| 4 Range topic tồn tại | Code publish `/ultrasonic/{front,rear,left,right}`. Không có runtime evidence; `start_ultrasonic` default false |
| `sensor_msgs/Range` | Có trong code; chưa quan sát |
| Sensor TF | FRONT/REAR đã đo → URDF thêm `ultrasonic_front` / `ultrasonic_rear` (geometry check script pass). LEFT/RIGHT không có frame. Chưa check bằng `tf2_echo` |
| Nav2 RangeSensorLayer | **Chưa implement**: không có config nào trong repo |
| Collision Monitor | Có config nhưng `polygons: []`, `observation_sources: []`, không nằm trong lifecycle manager nào → không consume gì |

Né vật cản bằng ultrasonic **không PASS**: chưa có vật thấp thật nào làm đổi costmap
hay hành vi robot.

---

## 8. TOP 5 NEXT ACTIONS

Chỉ những bước tối thiểu để pipeline Nav2 trên robot thật PASS lại:

1. **Chốt revision đang test.** Commit staged changes hoặc ghi commit hash.
   `sudo systemctl stop warotrans-teleop`; launch với `enable_ekf:=false`.
2. **Bringup Nav2 với indoor map** (`map:=$HOME/maps/warotrans.yaml`) và lưu evidence
   vào log: `ros2 lifecycle get` từng node, `tf2_echo map odom` và
   `odom base_footprint`, `topic hz` cho `/scan` và `/odom`.
3. **Một goal thẳng ngắn (~1 m, sàn trống)** sau 2D Pose Estimate. Gửi bằng
   `ros2 action send_goal /navigate_to_pose ... --feedback`, đồng thời ghi
   `ros2 topic echo /cmd_vel`. Lưu text SUCCEEDED, `/cmd_vel` khác 0, và quãng đường
   đo bằng thước.
4. **Một goal cần rẽ 90°** (case từng lỗi). Ghi kết quả nguyên trạng. Nếu fail, phân
   loại lỗi (planner / controller / motor response) **trước khi** đổi bất kỳ param nào.
5. **Ghi evidence có ngày vào `STATUS.md`** kèm commit hash; tag commit
   `nav2-rviz-real-robot-pass`.
   - Đồng thời sửa doc drift: STATUS vẫn trỏ tới `warotrans-nav-start.sh`,
     `check_nav2.sh`, `navigation_doctor.sh` — đã bị xóa khỏi `tools/` trong repo
     (vẫn còn trên Pi).
   - Ghi chú launch arg `nav_start_delay_sec` bị bỏ qua vì delay hard-code 12 s.

Pi cần bật nguồn và cùng mạng trước bước 2–4.
