# Navigation hardening — robot qua được ngã 3, Nav To Pose kết thúc chắc chắn

Nhánh: `feature/navigation-hardening` (từ `origin/develop` @730b4e1) · 2026-10-16

## 1. Triệu chứng

Gửi NavigateToPose trên RViz thì Nav2 vẫn plan ra đường đi và robot đi thẳng được. Nhưng ở **ngã 3**, khi phải rẽ trái hoặc phải sang lối bên kia kệ, robot không rẽ. Nó cắt góc, đâm vào tường rồi đứng im, và goal không bao giờ kết thúc (không SUCCEEDED, cũng không ABORTED).

## 2. Nguyên nhân (audit)

| # | Nguyên nhân | Tại sao gây lỗi ngã 3 |
|---|---|---|
| 1 | **Motor open-loop, không bù vùng chết.** PWM tỉ lệ với tốc độ bánh (0.32 m/s ≈ 100 %), trần 40 %. | Robot skid-steer khi quay phải kéo bánh trượt ngang nên cần mô-men lớn. Quay tại chỗ ở 0.35 rad/s thì mỗi bánh chỉ được ~25 % PWM; Spin 0.1 rad/s được ~7 %. Ở mức này bánh **kẹt**, xe không quay. Khi đi cong, bánh trong cũng kẹt, nên xe đi gần như thẳng và cắt góc vào tường. |
| 2 | `inflation_radius` 0.15 m **nhỏ hơn** bán kính ngoại tiếp của robot (~0.206 m). | NavFn coi robot là một điểm nên vẽ đường ôm sát góc kệ. Khi footprint chữ nhật quay ở ngã 3, nó đụng góc, RPP báo "collision ahead" và FollowPath fail liên tục. Lỗi này đã tái hiện được trong Gazebo. |
| 3 | BT recovery 6×3 lần kết hợp progress checker 45 s. | Robot kẹt nhiều phút, trông như bị treo. |
| 4 | EKF/IMU (đã được sửa trên develop): chỉ fuse vận tốc bánh, IMU ở chế độ IMUPLUS với `imu0_relative`. | Heading sai thì RPP lái lệch. |
| 5 | AMCL `recovery_alpha_fast` rải hạt ngẫu nhiên. | Pose nhảy sang một lối khác có hình dạng giống hệt. |
| 6 | Không chạy `velocity_smoother` và `collision_monitor`. | Không có lớp dừng khẩn cấp bằng lidar. |
| 7 | Dùng VoxelLayer cho lidar 2D, `obstacle_min_range` 0. | Tốn CPU trên Pi, có thể đánh dấu chính khung xe là vật cản. |
| 8 | Bringup bỏ qua `nav_start_delay_sec`. | — |
| 9 | Odom tính dt theo giờ ROS nhận gói. | Vận tốc odom bị giật. |

## 3. Đã sửa

### Firmware ESP32-S3 (`firmware/warotrans_low_level/warotrans_low_level.ino`)
- Trần PWM 40 % → 60 % (2026-10-16) → **100 %** (2026-10-23, duty 255/255). Mobile teleop mặc định vẫn ≤40 %; option **MAX POWER** mới mở tới 100 %.
- **Bù vùng chết:** khi bánh được lệnh chạy, `duty = min_start_duty + tỉ lệ·255`.
- **Vòng kín PI tốc độ bánh** (feedforward + PI):
  - sai số tính theo độ lớn và có xét dấu, sửa lỗi dấu khi chạy lùi;
  - chống bão hòa tích phân;
  - reset tích phân khi bánh đảo chiều hoặc khi lệnh = 0.
- **Lệnh cấu hình runtime mới:** `C <name> <value>` với `name` là `min_duty`, `closed_loop`, `kp`, `ki`. ESP32 trả về `K name value`, hoặc `K ERR name` nếu sai.
- `min_duty` / `closed_loop` / `kp` / `ki` mặc định **giống open-loop cũ** (0 / off / 0 / 0). Hành vi bù deadband + PI chỉ bật khi `esp32_bridge_node` gửi tham số từ `hardware.yaml`.
- **Ngoại lệ:** trần PWM 100 % luôn bật sau khi nạp firmware mới (không còn 40/60 %). Soft cap teleop 40 % vẫn mặc định. Watchdog 300 ms giữ nguyên.

### ROS
- `esp32_bridge_node`:
  - tham số mới `wheel_min_start_duty`, `wheel_velocity_closed_loop`, `wheel_velocity_kp`, `wheel_velocity_ki`;
  - gửi lại cấu hình mỗi khi ESP32 khởi động lại (phát hiện qua `mcu_millis` giảm).
- `motor_mixer.py` (bản Python mô phỏng firmware): trần 60 %, có `min_start_duty`.
- `wheel_odom_node`: dt lấy từ `mcu_millis`.
- `nav2_params.yaml`:
  - inflation 0.35 / 4.0 (global và local);
  - ObstacleLayer thay VoxelLayer, `obstacle_min_range` 0.20, `footprint_padding` 0.02;
  - RPP: lookahead 0.30, quay về hướng 0.5 rad/s, giảm tốc theo cost, `regulated_linear_scaling_min_radius` 0.9;
  - Spin 0.3–0.6 rad/s;
  - progress checker 0.10 m / 20 s;
  - `collision_monitor` dùng `/scan` với polygon "approach" bằng footprint (dự đoán va chạm trước 1.2 s).
- `navigation.launch.py`:
  - đường lệnh `controller/behavior → cmd_vel_nav → velocity_smoother → collision_monitor → /cmd_vel`;
  - tắt được bằng `enable_cmd_vel_pipeline:=false`.
- BT `navigate_to_pose_w_replanning_resilient.xml`:
  - outer 4 lần, FollowPath retry 2 lần;
  - recovery luân phiên: clear costmap → lùi 0.15 m → quay 90° → chờ 2 s.
- `amcl.yaml`: alpha 0.15, không rải hạt ngẫu nhiên (`recovery_alpha_*` = 0), `max_beams` 90.
- Bringup:
  - `nav_start_delay_sec` có hiệu lực;
  - cảnh báo nếu service `warotrans-teleop` đang chạy, vì nó tranh `/cmd_vel` với Nav2.
  - `enable_ekf` **vẫn mặc định false**, đến khi IMU qua DoD trên robot.

### Simulation (regression cho lỗi ngã 3)
- IMU giả lập trên `imu_link`, publish `/imu/data`; world có plugin `gz-sim-imu-system`.
- `enable_ekf:=true`:
  - `/odom` lấy từ odometry bánh của DiffDrive, có trôi như robot thật;
  - EKF chạy với `ekf.yaml` của robot cộng overlay `ekf_sim.yaml`;
  - Gazebo không phát TF `odom → base_footprint`, EKF giữ cạnh TF đó.
- `motor_deadband:=<m/s>`: node `motor_deadband_emulator` cho bánh kẹt khi tốc độ dưới ngưỡng (giả lập firmware cũ). Thêm `deadband_compensation:=true` để nâng bánh lên đúng ngưỡng (giả lập firmware mới).
- `sim_speeds:=false`: dùng đúng tốc độ của robot thật (0.10 m/s).
- `nav2_params_sim.yaml` chỉ còn phần tốc độ; costmap và BT dùng chung với robot.
- Script `nav_regression.py` chạy 6 kịch bản và in bảng kết quả.

## 4. Kết quả Gazebo

Cấu hình sát robot thật: `enable_ekf:=true sim_speeds:=false motor_deadband:=0.10 deadband_compensation:=true`.

| Kịch bản | Kết quả | Thời gian | Lần recovery |
|---|---|---|---|
| Đi thẳng 1.1 m | SUCCEEDED | 13 s | 0 |
| Rẽ **trái** ở ngã 3 vào lối A\|B | SUCCEEDED | 40 s | 0 |
| Ra khỏi lối, rẽ **phải** vào lối B\|C | SUCCEEDED | 91 s | 0 |
| Vòng qua kệ sang trạm sạc | SUCCEEDED | 48 s | 0 |
| Hộp lạ chắn đường (lidar) | SUCCEEDED | 82 s | 8 |
| Goal nằm trong kệ | **ABORTED** (không treo) | 11 s | — |

Đối chứng không bù vùng chết (`deadband_compensation:=false`):
- Ở 0.10 m/s, robot không đi nổi ngay kịch bản đầu: controller lặp "Failed to make progress", BackUp hết thời gian. Đây đúng là tình trạng bánh kẹt.
- Ở 0.085 m/s, **tái hiện đúng lỗi ngoài đời**: đi thẳng, rẽ trái, rẽ phải và sang bên kia kệ đều **treo hết 300 s** (25 lần recovery mỗi kịch bản), robot dừng cách goal 0.5–5.3 m. Đã dừng lượt chạy trước 2 kịch bản cuối vì đủ bằng chứng.

**Kết luận:** cùng cấu hình Nav2 mới, chỉ khác có hay không bù vùng chết ở motor, kết quả đi từ 0/4 lên 6/6. Vì vậy bước đo `min_start_duty` và nạp firmware trên robot thật là bắt buộc.

Topic trong sim (đo theo giờ thật):
- `/scan` 7–10 Hz;
- `/odom` 40–50 Hz;
- `/odometry/filtered` 23–30 Hz;
- `/imu/data` ~36 Hz.

```bash
ros2 launch warotrans_simulation sim_bringup.launch.py enable_ekf:=true sim_speeds:=false \
    motor_deadband:=0.10 deadband_compensation:=true gateway:=false
ros2 run warotrans_simulation nav_regression.py
```

## 5. Còn phải làm trên robot thật (cần người có mặt, khu vực trống)

1. Backup `~/ros2_ws/src` vào `~/backups/`. Stop `warotrans-teleop`. Deploy code rồi `colcon build`.
2. Kiểm tra topic và TF:
   - `/scan` ~7 Hz;
   - `/imu/data` 50 Hz với `frame_id=imu_link`;
   - `/odom` ~50 Hz;
   - `/odometry/filtered` 30 Hz;
   - mỗi cạnh TF chỉ có một publisher.
   - IMU đứng yên 60 s: yaw trôi < 1°. Quay xe ngược chiều kim đồng hồ: yaw tăng.
3. Nạp firmware:
   ```bash
   arduino-cli compile --fqbn esp32:esp32:esp32s3:CDCOnBoot=cdc warotrans_low_level
   ```
   Core `esp32:esp32@3.0.7`. Lưu bản `.bin` cũ để rollback.
4. Kê bánh lên, gửi `C min_duty N` để tìm duty nhỏ nhất mà bánh bắt đầu quay: một lần khi đi thẳng, một lần khi quay tại chỗ trên sàn.
   - Bật `closed_loop`, tune `kp`/`ki` bằng step test.
   - Ghi các giá trị vào `warotrans_hardware/config/hardware.yaml` và `docs/calibration.md`.
5. Trên RViz, chạy 6 kịch bản giống sim. Ghi rosbag gồm `/scan /imu/data /odom /odometry/filtered /cmd_vel /plan /tf`.
6. Nếu IMU qua DoD: đổi `enable_ekf` mặc định thành `true`.
