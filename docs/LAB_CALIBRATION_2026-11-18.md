# RBT-001: hiệu chuẩn lại ngày 2026-11-18

Dùng cùng phương pháp đã đo RBT-002. Claude tự chạy robot trên sàn, có anh cho phép, bản đồ MAP-ONE-V014.

## Cách đo

- **Chuẩn thật:** so khớp ICP giữa hai scan trung vị chụp lúc robot đứng yên, trước và sau mỗi lần chạy. Không dùng AMCL.
- **Chặn an toàn:**
  - hành lang ±0.20 m phía trước và phía sau thân xe;
  - khi xoay, vật cản gần nhất phải cách tâm xoay ≥ 0.40 m (bán kính quét 0.335 m);
  - watchdog lệnh 300 ms trên ESP32.
- **Sao lưu cấu hình trước khi sửa:** trên Pi tại `~/warotrans_backups/20261118_pre_recalib/` (`configs.tgz`, `src_full.tgz`).

## Kết quả trước khi sửa

| Phép đo | LiDAR | BNO055 gyro z | Odom bánh |
|---|---|---|---|
| 360° CCW ở 0.6 rad/s | 6.089 rad | 6.240 (+2.5 %) | 5.014 |
| 360° CW | −6.077 | −6.238 (+2.6 %) | −5.458 |
| Tiến ~1 m ở 0.30 m/s | 0.921 m, lệch trái 2.2° | — | 0.909 (−1.3 %) |
| Lùi ở 0.18 m/s | 0.903 m, lệch 2.3° | — | 0.892 (−1.3 %) |

- **Chiều xe đúng:** LiDAR (yaw π/2), IMU và odom khớp chiều nhau.
- **Tâm xoay ≈ `base_footprint`:** sau một vòng xoay 61°, LiDAR dịch chuyển đúng theo cung tròn bán kính 0.14 m.
- **Odom bánh khi xoay hụt 10–18 %** vì bánh trượt (skid-steer), và lệch khác nhau giữa CW và CCW. EKF không dùng yaw từ odom nên không ảnh hưởng.

## Đã sửa (đã deploy lên Pi, chưa commit)

- **`warotrans_sensors/bno055_node.py`:** thêm tham số `gyro_z_scale` và `yaw_scale`. Mặc định 1.0 nên không đổi hành vi khi không cấu hình.
- **`config/bno055.yaml`:** `gyro_z_scale: 0.975`, `yaw_scale: 1.0075`.
  - Gyro thô đọc dư 2.5 %, còn yaw đã qua bộ fusion của BNO055 đọc thiếu khoảng 0.75 %.
  - Đã thử áp một hệ số 0.975 cho cả hai, nhưng EKF khi đó lại thiếu 3.3 %.
- **`warotrans_hardware/config/hardware.yaml`:** bán kính bánh nhân 1.0133, thành 0.03340 (trái) và 0.03306 (phải).

## Kiểm tra sau khi sửa

| | LiDAR | EKF (`/odometry/filtered`) |
|---|---|---|
| 360° CCW | 6.227 | 6.232 (+0.07 %) |
| 360° CW | −6.223 | −6.227 (+0.07 %) |
| Tiến | 0.919 m | 0.921 m |
| Lùi | 0.898 m | 0.898 m |

## Chạy chuỗi endpoint bằng Nav2

Cấu hình chạy:
- gửi `NavigateToPose` thẳng cho Nav2;
- tắt authority gate, không chạy gateway;
- collision monitor vẫn bật;
- planner lane-aware, costmap filters và EKF bật.

**Kết quả:** 6/6 chặng thành công, 0 lần phục hồi, 0 lần "collision ahead", 0 lần "failed to make progress".

| Chặng (bắt đầu ở INBOUND) | Thời gian | Sai số tới đích |
|---|---|---|
| → OUTBOUND | 12.3 s | 7.6 cm, −0.5° |
| → SHELF-C-1 | 25.7 s | 8.4 cm, 2.2° |
| → PARKING-B | 15.8 s | 7.3 cm, 3.5° |
| → SHELF-B-01-02 | 25.8 s | 14.4 cm, 0.6° |
| → SHELF-A-1 | 24.0 s | 10.7 cm, −2.5° |
| → PARKING-A | 16.7 s | 5.6 cm, 2.4° |

**Chưa xử lý:**
- Xe lệch trái khoảng 2.2° mỗi mét khi đi thẳng, do hai bên chạy không đều. Nav2 tự bù được.
- Sai số tới đích trên 0.12 m ở hai chặng. Khi chạy transport request, các bước docking và refine của gateway sẽ xử lý phần này.

## Thay đổi 2026-11-19 (branch `fix/rbt1-imu-drive-sync`, chưa nạp, chưa đo lại)

**Commit 1, chỉ code Pi:**
- `bno055_node` kiểm tra `OPR_MODE` mỗi 1 s. Khi chip tự reset, node cấu hình lại và nối yaw mới vào yaw đã publish (`yaw_tracker.py`), nên EKF không bị nhảy hướng.
- Bỏ mẫu có norm quaternion ngoài khoảng (0.9, 1.1).
- `Nav2SegmentExecutor`: SPIN không có yaw hiện tại thì trả `SPIN_YAW_UNAVAILABLE`, không đoán là 0.

**Commit 2, Pi và firmware ESP32 phải deploy cùng lúc:**
- Firmware không còn hardcode ticks/bán kính (cũ: 2478.1 cả hai bên, bán kính cũ, làm bánh phải nhanh hơn 0.53 %). Pi gửi `C mpt_l`, `C mpt_r`, `C sep` tính từ block `/**` trong `hardware.yaml`. Đây cũng là nguồn mà `wheel_odom_node` dùng.
- `wheel_separation` hiệu dụng đổi 0.4535 → **0.395**. Đo ICP ngày 18/11 sau khi sửa bán kính: 0.378 khi xoay CCW, 0.413 khi xoay CW. Teleop `commissioning_angular_command_max` đổi 1.4112 → 1.6203.

**Cần đo sau khi nạp:**
- Đi thẳng 2 m. Nếu còn lệch θ (rad/m) thì đặt ε = θ·T, với T là track đo bằng thước. Lệch trái thì nhân `wheel_radius_right` với (1 − ε/2) và `wheel_radius_left` với (1 + ε/2).
- Xoay 2×360° mỗi chiều. Tính `sep_mới = sep · yaw_odom / yaw_LiDAR`.
