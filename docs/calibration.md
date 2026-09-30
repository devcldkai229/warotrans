# WaroTrans — Calibration & Measurement Log

> Đây là **source of truth cho số đo vật lý**. Ô trống = CHƯA XÁC NHẬN.
> Không điền giá trị từ ví dụ, datasheet chung, ảnh Internet hoặc "ước chừng".

Mỗi record cần:
- ngày;
- giá trị;
- phương pháp;
- điều kiện test;
- evidence/ghi chú.

### Trạng thái giá trị

| Nhãn | Ý nghĩa |
|---|---|
| **Measured / initial** | Số đo vật lý hoặc bring-up; dùng được trước Phase 7 nhưng chưa qua motion validation |
| **Phase 7A provisional** | Số đo đã xác nhận (encoder/tape/radius) dùng tạm cho runtime; chưa qua powered 360° / straight-line calibration |
| **Calibrated** | Giá trị sau test chuyển động Phase 7; đây mới là truth cho odom/SLAM |

Giá trị machine-readable tương ứng nằm trong
`ros2_ws/src/warotrans_description/config/calibration.yaml`.

Runtime odom params (tạm thời, kể cả tape separation) nằm trong
`ros2_ws/src/warotrans_hardware/config/hardware.yaml`.

---

## Phase 7 — calibrated summary (2026-09-07 evening)

| Parameter | Value | Machine file | Status |
|---|---:|---|---|
| `ticks_per_rev_left` | 2478.1 | `calibration.yaml` + `hardware.yaml` | Calibrated — 10 rev @ 15% |
| `ticks_per_rev_right` | 2478.1 | `calibration.yaml` + `hardware.yaml` | Calibrated — 10 rev @ 15% |
| `radius_left` | 0.03296 m | `calibration.yaml` + `hardware.yaml` | Calibrated — from 2.00 m + ticks |
| `radius_right` | 0.03263 m | `calibration.yaml` + `hardware.yaml` | Calibrated — from 2.00 m + ticks |
| `separation` | 0.4535 m | `calibration.yaml` + `hardware.yaml` + `teleop.yaml` | Effective — mean(0.4274, 0.4795) |

Công thức bán kính từ thẳng 2.00 m (ưu tiên tick, không dùng chord odom cũ):

```text
R = distance_real × ticks_per_rev / (Δticks × 2π)
R_L = 2.00 × 2478.1 / (23934 × 2π) ≈ 0.03296 m
R_R = 2.00 × 2478.1 / (24172 × 2π) ≈ 0.03263 m
```

Ghi chú: bài thẳng có yaw odom ~29° (không hoàn toàn thẳng); vẫn dùng Δtick + thước 2.00 m.

---

## Phase 7A — provisional summary (2026-09-02) — superseded

| Parameter | Value | Machine file | Status |
|---|---:|---|---|
| `ticks_per_rev_left` | 2468.0 | — | superseded 2026-09-07 |
| `ticks_per_rev_right` | 2241.2 | — | superseded 2026-09-07 |
| `radius_left` | 0.0325 m | — | superseded 2026-09-07 |
| `radius_right` | 0.0325 m | — | superseded 2026-09-07 |
| `separation` | 0.2183 m tape → later 0.4535 | — | see Phase 7 summary |

---

## Phase 7.5 — commissioning command scale (2026-09-03)

| Parameter | Value | Where used | Status |
|---|---:|---|---|
| `commissioning_full_scale_linear` | 0.32 m/s | firmware + teleop YAML | Phase 7.5 provisional — **NOT measured speed** |
| `commissioning_full_scale_angular` | 1.4112 rad/s | firmware + teleop YAML | derived = 2×0.32/0.4535 |
| `MOTOR_PWM_LIMIT_PERCENT` | 40 % | firmware hard ceiling | commissioning safety limit |
| default teleop speed level | 20 % | teleop YAML | commissioning default |
| `LEFT_MOTOR_INVERTED` | false | firmware | verify on robot |
| `RIGHT_MOTOR_INVERTED` | false | firmware | verify on robot |

PWM raw mapping at 8-bit resolution (255 max, 40% ceiling = 102):

| Level % | Raw PWM |
|---:|---:|
| 10 | 26 |
| 15 | 38 |
| 20 | 51 |
| 25 | 64 |
| 30 | 77 |
| 35 | 89 |
| 40 | 102 |

---

## 1. Encoder — ticks_per_rev

Định nghĩa: tick quadrature được node sử dụng trên **một vòng bánh sau hộp số**.

Phương pháp baseline:
1. kê bánh khỏi sàn;
2. đánh dấu bánh;
3. đọc tick đầu;
4. quay chậm đúng 10 vòng;
5. đọc tick cuối;
6. `(after-before)/10`;
7. lặp ≥3 lần.

| Ngày | Wheel/side | Trial 1 | Trial 2 | Trial 3 | Giá trị dùng | Ghi chú |
|---|---|---:|---:|---:|---:|---|
| 2026-09-02 | left | | | | 2468.0 | Phase 7A — superseded |
| 2026-09-02 | right | | | | 2241.2 | Phase 7A — superseded |
| 2026-09-07 | left | 24781/10 | | | 2478.1 | 10 rev @ 15%, operator |
| 2026-09-07 | right | 24781/10 | | | 2478.1 | 10 rev @ 15%, operator |

---

## 2. wheel_radius (m)

Physical measurement: đo bánh khi robot chịu tải.

Dynamic validation: chạy đoạn thẳng có khoảng cách thật đã đo.

```text
R_new = R_old × distance_real / distance_odom
```

### Per-wheel

| Ngày | Side | Giá trị | Trạng thái | Ghi chú |
|---|---|---:|---|---|
| 2026-09-02 | left | 0.0325 | superseded | nominal Ø65 |
| 2026-09-02 | right | 0.0325 | superseded | nominal Ø65 |
| 2026-09-07 | left | 0.03296 | Calibrated | 2.00 m, ΔtickL=23934, tpr=2478.1, speed 15% |
| 2026-09-07 | right | 0.03263 | Calibrated | 2.00 m, ΔtickR=24172, tpr=2478.1, speed 15% |

### Dynamic validation log

| Ngày | Side | Giá trị | Distance real | Odom/ticks | Surface | Ghi chú |
|---|---|---:|---:|---:|---|---|
| 2026-09-07 | L/R | 0.03296 / 0.03263 | 2.00 m | ticks 23934 / 24172 | | yaw drift ~29° trên /odom; R lấy từ tick+thước |
| 2026-09-07 | check | — | 1.00 m | ticks 11925 / 12038 | | cross-check cùng buổi |

---

## 3. effective wheel_separation (m)

Với skid-steer, dùng **effective separation** sau test quay Phase 7 powered 360°, không coi khoảng cách hình học là truth cuối.

```text
W_new = W_old × yaw_odom / yaw_real
```

### Calibrated (Phase 7 rotation test)

| Ngày | Giá trị | Trial inputs | Surface | Battery/load | Ghi chú |
|---|---:|---|---|---|---|
| 2026-09-07 | 0.4535 m | 0.4274 m, 0.4795 m | | | Operator: mean of two effective values `(0.4274+0.4795)/2≈0.45345` → dùng **0.4535** |

```text
W_eff = (0.4274 + 0.4795) / 2 ≈ 0.4535 m
```

### Physical tape separation (lịch sử)

```text
separation_tape = 0.2183 m   # Phase 7A, 2026-09-02 — superseded 2026-09-07
```

Phương pháp tape: đo thước dây khoảng cách trục–trục trên robot đã lắp.

### Chassis wheelbase (trục trước–sau)

Không thay `wheel_separation` (track / hiệu dụng khi xoay). Chỉ hình học khung.

| Ngày | Giá trị | Phương pháp | Trạng thái | Ghi chú |
|---|---:|---|---|---|
| 2026-09-07 | 0.146 m (146 mm) | đo trên robot đã lắp, tâm trục trước → tâm trục sau | Measured / initial | `calibration.yaml` `chassis.wheelbase` |

### Trạng thái hiện tại (2026-09-07)

```text
wheel.separation = 0.4535 m   # effective, operator mean of two rotation-derived trials
```

Đã sync: `calibration.yaml`, `hardware.yaml`, `teleop.yaml`, firmware source
`warotrans_low_level.ino` (cần reflash ESP32 mới có số trên MCU).
Triệu chứng nếu vẫn sai khi rẽ: tường cong/double — đo lại trial, **không** tune `slam_toolbox`.

---

## 4. LiDAR pose relative to `base_link`

ROS convention: +X front, +Y left, +Z up.

| Parameter | Value | Date | Method / reference | Trạng thái | Evidence |
|---|---:|---|---|---|---|
| `lidar_x` | 0.010 m | | tâm robot → tâm LiDAR theo X | measured / initial | sync `calibration.yaml`, `geometry.yaml` |
| `lidar_y` | 0.0017 m | | tâm robot → tâm LiDAR theo Y | measured / initial | sync `calibration.yaml`, `geometry.yaml` |
| `lidar_z` | 0.142 m | | sàn/base reference → scan plane | measured / initial | sync `calibration.yaml`, `geometry.yaml` |
| `lidar_roll` | | | physical alignment | CHƯA XÁC NHẬN | |
| `lidar_pitch` | | | physical alignment | CHƯA XÁC NHẬN | |
| `lidar_yaw` | +π/2 rad (1.5708) | 2026-09-07 | box-in-front Foxglove: vở trước đầu → đường song song Ox, trước mũi +X trống; operator kết luận gắn xoay trái 90° | measured / initial | sync `geometry.yaml` + `calibration.yaml`; chờ xác nhận lại sau restart |

---

## 4b. Ultrasonic pose relative to `base_link` (Phase 0.5)

ROS convention: +X front, +Y left, +Z up. Đo tới tâm transducer (hoặc mặt phát đã thống nhất).

**Nav2 Phase 0.5 chỉ cần FRONT + REAR** (vật đột ngột trước/sau).  
LEFT / RIGHT: **không** đưa vào Collision Monitor / costmap; có thể để trống hoặc ghi “không dùng Nav2”.

| Sensor | x (m) | y (m) | z (m) | yaw (rad) | Date | Method | Trạng thái | Nav2 |
|---|---:|---:|---:|---:|---|---|---|---|
| FRONT | 0.1697 | 0.0015 | 0.0045 | 0 | 2026-09-12 | thước mm: (154.2+15.5) mm tiến; y 1.5 mm trái; z 4.5 mm; yaw 0 | **measured / initial** | **bắt buộc đo** |
| REAR | −0.02892 | 0.0 | 0.0045 | π (180°) | 2026-09-12 | thước mm: (15.42+13.5) mm về đuôi → x âm; z 4.5 mm; yaw 180° (operator xác nhận toàn bộ mm) | **measured / initial** | **bắt buộc đo** |
| LEFT | | | | ~+π/2 | | | **CHƯA XÁC NHẬN** / không dùng Nav2 | không dùng Nav2 |
| RIGHT | | | | ~−π/2 | | | **CHƯA XÁC NHẬN** / không dùng Nav2 | không dùng Nav2 |

Quy đổi raw → mét (operator 2026-09-12, **toàn bộ mm**):

```text
FRONT: x = (154.2 + 15.5) mm = 169.7 mm = 0.1697 m
       y = +1.5 mm (trái) = 0.0015 m
       z = 4.5 mm = 0.0045 m
       yaw = 0 rad
REAR:  x = −(15.42 + 13.5) mm = −28.92 mm = −0.02892 m
       y = 0
       z = 4.5 mm = 0.0045 m
       yaw = π rad (180°)
```

Synced: `ros2_ws/src/warotrans_description/config/geometry.yaml`.

### Checklist đo FRONT / REAR (operator)

1. [x] Robot nằm phẳng; chọn điểm gốc `base_link` (tâm chassis / khớp URDF đã dùng).
2. [x] Đo FRONT: x (tiến +), y (trái +), z (cao), yaw (~0 nếu hướng +X).
3. [x] Đo REAR: x (thường âm), y, z, yaw (~π nếu hướng −X).
4. [x] Ghi ngày + phương pháp vào bảng trên (photo: bổ sung nếu có).
5. [x] Sync cùng số vào `ros2_ws/src/warotrans_description/config/geometry.yaml` (`ultrasonic.front` / `ultrasonic.rear`).
6. [x] `bash tools/warotrans-check-ultrasonic-front-rear-geometry.sh` → `FRONT_REAR_GEOMETRY_OK`.
7. [x] LEFT/RIGHT: bỏ qua Nav2 (không bắt buộc đo cho Phase 0.5 này).

Sau khi FRONT+REAR có số: sync URDF → `tf2_echo base_link ultrasonic_front` / `ultrasonic_rear`.  
**Không** bật Collision Monitor / RangeSensorLayer trước bước 6.  
Chi tiết enable: [ultrasonic-front-rear-nav2.md](ultrasonic-front-rear-nav2.md). Runbook Nav2: [nav2-rviz-operator.md](nav2-rviz-operator.md).

### Range validation log (STEP 6) — ưu tiên FRONT/REAR

| Sensor | expected (m) | ROS range (m) | error (m) | Date | Notes |
|---|---:|---:|---:|---|---|
| FRONT | 0.20 | | | | **DoD Phase 0.5** |
| FRONT | 0.50 | | | | **DoD Phase 0.5** |
| FRONT | 1.00 | | | | **DoD Phase 0.5** |
| REAR | 0.20 | | | | **DoD Phase 0.5** |
| REAR | 0.50 | | | | **DoD Phase 0.5** |
| REAR | 1.00 | | | | **DoD Phase 0.5** |
| LEFT | 0.20 | | | | optional / không Nav2 |
| LEFT | 0.50 | | | | optional |
| LEFT | 1.00 | | | | optional |
| RIGHT | 0.20 | | | | optional / không Nav2 |
| RIGHT | 0.50 | | | | optional |
| RIGHT | 1.00 | | | | optional |
| * | no obstacle | | | | expect `inf` / near max — never `0.0` |

---

## 5. Camera pose

**Chỉ điền sau khi exact camera model + mounting được VERIFIED.**

| Parameter | Value | Date | Method |
|---|---:|---|---|
| exact model | | | |
| interface | | | CSI / USB |
| `camera_x` | | | |
| `camera_y` | | | |
| `camera_z` | | | |
| roll/pitch/yaw | | | |

---

## 6. Encoder wiring evidence

| Motor/side | Motor + | Motor - | VCC | GND | A | B | Supply measured |
|---|---|---|---|---|---|---|---:|
| front-left / left reference | | | | | | | |
| front-right / right reference | | | | | | | |

Không suy từ màu dây của motor khác/lô khác.

---

## 7. Electrical measurements

| Date | Measurement point | Measured value | Robot state/load | Instrument | Notes |
|---|---|---:|---|---|---|
| | battery | | | | |
| | Pi input rail | | | | |
| | ESP32 supply | | | | |
| | encoder supply | | | | |
| | LiDAR USB behavior | | | | |

Không ghi "target" vào cột measured value.

---

## 8. IMU calibration/orientation

| Date | Mounting orientation | Calibration status | Static yaw behavior | Notes |
|---|---|---|---|---|
| | | | | |

---

## 9. Velocity limits (for Nav2)

| Date | Parameter | Value | PWM level | Method | Status | Notes |
|---|---|---:|---:|---|---|---|
| 2026-09-09 | max_linear | 0.128 m/s | 40% | theoretical (scale×0.4) | PROVISIONAL | firmware scale=0.32 m/s @100% |
| 2026-09-09 | max_angular | 0.56 rad/s | 40% | theoretical (scale×0.4) | PROVISIONAL | firmware scale=1.41 rad/s @100% |

**Measurement required**: run `ros2 topic echo /odom --field twist.twist.linear.x` at 40% teleop straight, and `twist.angular.z` during rotation. Update calibration.yaml after.

---

## 10. Motor velocity/PID experiment log

PID gains là **tuning values**, không phải physical truth; vẫn phải có experiment history.

| Date | Side | Kp | Ki | Kd | Target | Result | Battery/surface | Notes |
|---|---|---:|---:|---:|---:|---|---|---|
| | | | | | | | | |

---

## 10. Map quality records

| Date | Map | Area | Straight error | Rotation error | Double wall? | Loop closed? | Notes |
|---|---|---|---:|---:|---|---|---|
| | | | | | | | |
