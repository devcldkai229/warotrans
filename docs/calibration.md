# WaroTrans — Calibration & Measurement Log

> Đây là **source of truth cho số đo vật lý**. Ô trống = CHƯA XÁC NHẬN.
> Không điền giá trị từ ví dụ, datasheet chung, ảnh Internet hoặc "ước chừng".

Mỗi record cần:
- ngày;
- giá trị;
- phương pháp;
- điều kiện test;
- evidence/ghi chú.

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
| | left | | | | | |
| | right | | | | | |

---

## 2. wheel_radius (m)

Physical measurement: đo bánh khi robot chịu tải.

Dynamic validation: chạy đoạn thẳng có khoảng cách thật đã đo.

```text
R_new = R_old × distance_real / distance_odom
```

| Ngày | Giá trị | Distance real | Odom read | Surface | Ghi chú |
|---|---:|---:|---:|---|---|
| | | | | | |

---

## 3. effective wheel_separation (m)

Với skid-steer, dùng effective separation sau test quay, không coi khoảng cách hình học là truth cuối.

```text
W_new = W_old × yaw_odom / yaw_real
```

| Ngày | Giá trị | Yaw real | Yaw odom | Surface | Battery/load | Ghi chú |
|---|---:|---:|---:|---|---|---|
| | | | | | | |

---

## 4. LiDAR pose relative to `base_link`

ROS convention: +X front, +Y left, +Z up.

| Parameter | Value | Date | Method / reference | Evidence |
|---|---:|---|---|---|
| `lidar_x` | | | tâm robot → tâm LiDAR theo X | |
| `lidar_y` | | | tâm robot → tâm LiDAR theo Y | |
| `lidar_z` | | | sàn/base reference → scan plane | |
| `lidar_roll` | | | physical alignment | |
| `lidar_pitch` | | | physical alignment | |
| `lidar_yaw` | | | box-in-front + RViz validation | |

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

## 9. Motor velocity/PID experiment log

PID gains là **tuning values**, không phải physical truth; vẫn phải có experiment history.

| Date | Side | Kp | Ki | Kd | Target | Result | Battery/surface | Notes |
|---|---|---:|---:|---:|---:|---|---|---|
| | | | | | | | | |

---

## 10. Map quality records

| Date | Map | Area | Straight error | Rotation error | Double wall? | Loop closed? | Notes |
|---|---|---|---:|---:|---|---|---|
| | | | | | | | |
