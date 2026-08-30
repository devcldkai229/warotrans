# WaroTrans — PROFESSIONAL ROBOT ENGINEERING MASTER GUIDE
## Kiến trúc mở rộng • ROS 2 Jazzy • LiDAR SLAM • Odometry • Nav2 • Camera • Fleet • Vibe Coding có kiểm soát

**Phiên bản:** 2.0  
**Mục tiêu:** biến bộ guide hiện tại thành một tài liệu kỹ thuật có thể dùng xuyên suốt từ bring-up robot thật đến navigation và fleet integration.  
**Đối tượng:** WaroTrans — robot vận chuyển hàng trong kho, Raspberry Pi 5 + ESP32-S3 + RPLIDAR + encoder + MDD10A.

> Đây là **master guide**, không phải danh sách lệnh để copy một lần. Mỗi phase có **Definition of Done (DoD)**. Chỉ sang phase tiếp theo khi phase hiện tại PASS.

---

# 0. NGUYÊN TẮC GỐC

## 0.1. Không nhảy cóc

Pipeline chuẩn:

```text
POWER / NETWORK
      ↓
Ubuntu + SSH
      ↓
ROS 2
      ↓
LiDAR /scan
      ↓
ESP32 serial + encoder
      ↓
Wheel odometry /odom
      ↓
TF + URDF
      ↓
Calibration
      ↓
SLAM + /map
      ↓
Map validation
      ↓
IMU fusion (nếu cần)
      ↓
Closed-loop motor PID
      ↓
Localization + Nav2
      ↓
Camera / perception
      ↓
Backend + Fleet
      ↓
Multi-robot
```

Nếu một tầng sai, tầng phía trên có thể vẫn "chạy" nhưng kết quả sẽ sai theo cách rất khó debug.

---

## 0.2. Ba loại thông tin phải phân biệt

### CONFIRMED
Đã kiểm tra trên robot thật hoặc đã có bằng chứng trong repo.

### MEASURED
Đã đo/hiệu chuẩn trên robot thật và ghi trong `docs/calibration.md`.

### PLANNED / NOT VERIFIED
Dự kiến dùng nhưng chưa kiểm chứng trên robot thật.

**AI, Cursor và người phát triển không được biến PLANNED thành CONFIRMED bằng cách tự đoán.**

---

## 0.3. Không tự bịa hằng số vật lý

Các giá trị sau chỉ được lấy từ đo đạc:

```text
ticks_per_rev
wheel_radius
wheel_separation
lidar_x / lidar_y / lidar_z
lidar_yaw
camera pose
encoder voltage
GPIO wiring thực tế
```

Nếu chưa có:

```text
CHƯA XÁC NHẬN
```

rồi đo.

---

# 1. KIẾN TRÚC HỆ THỐNG MỤC TIÊU

```text
                           ┌─────────────────────────────┐
                           │        LAPTOP / WEB         │
                           │ RViz/Foxglove • Dashboard   │
                           └──────────────▲──────────────┘
                                          │ Wi-Fi/Ethernet
                                          │
┌───────────────┐                   ┌──────┴───────────┐
│  RPLIDAR A1   │────── USB ───────►│ Raspberry Pi 5  │
└───────────────┘                   │ Ubuntu 24.04     │
                                    │ ROS 2 Jazzy      │
┌───────────────┐                   │                  │
│    Camera     │──────────────────►│ perception       │
└───────────────┘                   │ SLAM / Nav2      │
                                    │ fleet bridge     │
┌───────────────┐                   └──────┬───────────┘
│    BNO055     │─────────────────────────►│
└───────────────┘                          │ USB serial
                                           ▼
                                    ┌───────────────┐
                                    │   ESP32-S3    │
                                    │ encoder + PID │
                                    │ safety        │
                                    └──────┬────────┘
                                           │ PWM/DIR
                                           ▼
                                    ┌───────────────┐
                                    │    MDD10A     │
                                    └──────┬────────┘
                                           ▼
                                        Motors
```

### Phân trách nhiệm

**Raspberry Pi 5**
- ROS 2 graph.
- LiDAR.
- SLAM / localization.
- Nav2.
- Camera/perception.
- Giao tiếp backend/fleet.
- Logging/diagnostics.

**ESP32-S3**
- Đọc encoder realtime.
- Điều khiển PWM/DIR.
- Watchdog.
- PID tốc độ bánh.
- Cầu nối low-level giữa `/cmd_vel` và MDD10A.

**Backend/Fleet**
- Không điều khiển PWM.
- Không gửi lệnh từng bánh.
- Gửi **task/goal cấp cao**.
- Theo dõi robot, trạng thái, task, lỗi.

---

# 2. TF TREE — HỢP ĐỒNG BẤT BIẾN

```text
map
 └── odom
      └── base_footprint
           └── base_link
                ├── laser
                └── camera_link
```

Quyền sở hữu:

| Transform | Publisher |
|---|---|
| `map → odom` | SLAM Toolbox hoặc localization |
| `odom → base_footprint` | wheel odometry hoặc EKF |
| `base_footprint → base_link` | URDF + robot_state_publisher |
| `base_link → laser` | URDF + robot_state_publisher |
| `base_link → camera_link` | URDF + robot_state_publisher |

**Quy tắc:** một transform chỉ có **một publisher**.

Nếu thêm `robot_localization` EKF:
- wheel odometry vẫn publish `/odom_raw` hoặc dữ liệu odometry.
- wheel node **không** publish `odom → base_footprint`.
- EKF là node duy nhất publish transform đó.

---

# 3. CẤU TRÚC REPO CHUYÊN NGHIỆP

Khuyến nghị dùng **monorepo**:

```text
warotrans/
├── .cursor/
│   └── rules/
│
├── firmware/
│   └── esp32_low_level/
│       ├── platformio.ini
│       ├── src/
│       │   └── main.cpp
│       ├── lib/
│       │   ├── MotorDriver/
│       │   ├── QuadEncoder/
│       │   ├── SerialProtocol/
│       │   ├── VelocityController/
│       │   └── WebControl/
│       └── test/
│
├── ros2_ws/
│   └── src/
│       ├── warotrans_msgs/
│       ├── warotrans_description/
│       │   ├── urdf/
│       │   ├── meshes/
│       │   └── launch/
│       │
│       ├── warotrans_hardware/
│       │   ├── warotrans_hardware/
│       │   │   ├── wheel_odom_node.py
│       │   │   ├── esp32_bridge_node.py
│       │   │   └── diagnostics_node.py
│       │   ├── config/
│       │   └── launch/
│       │
│       ├── warotrans_localization/
│       │   ├── config/
│       │   │   └── ekf.yaml
│       │   └── launch/
│       │
│       ├── warotrans_slam/
│       │   ├── config/
│       │   │   └── slam_toolbox.yaml
│       │   └── launch/
│       │
│       ├── warotrans_navigation/
│       │   ├── config/
│       │   │   └── nav2_params.yaml
│       │   ├── maps/
│       │   ├── behavior_trees/
│       │   └── launch/
│       │
│       ├── warotrans_perception/
│       │   ├── config/
│       │   ├── launch/
│       │   └── warotrans_perception/
│       │
│       ├── warotrans_fleet/
│       │   ├── config/
│       │   ├── launch/
│       │   └── warotrans_fleet/
│       │
│       ├── warotrans_simulation/
│       │
│       └── warotrans_bringup/
│           ├── config/
│           └── launch/
│               ├── robot.launch.py
│               ├── mapping.launch.py
│               └── navigation.launch.py
│
├── backend/
├── web/
├── mobile/
│
├── docs/
│   ├── STATUS.md
│   ├── interfaces.md
│   ├── calibration.md
│   ├── architecture.md
│   ├── troubleshooting.md
│   └── adr/
│
├── tools/
│   ├── doctor.sh
│   ├── check.sh
│   ├── deploy.sh
│   └── deploy.ps1
│
└── README.md
```

## 3.1. Một package = một lý do để thay đổi

- `description`: hình học robot.
- `hardware`: serial, encoder, odometry, hardware health.
- `localization`: EKF/AMCL.
- `slam`: mapping.
- `navigation`: Nav2.
- `perception`: camera/vision.
- `fleet`: bridge tới backend.
- `bringup`: **chỉ orchestration**, không chứa business logic.

## 3.2. Migration từ package prototype hiện tại

Không cần refactor cực lớn ngay lập tức.

**Stage 1**
- Di chuyển `wheel_odom_node.py` từ `warotrans_bringup` → `warotrans_hardware`.
- `warotrans_bringup` chỉ giữ launch/config.
- Giữ behavior hiện tại để đạt map đầu tiên.

**Stage 2 — sau map đầu tiên**
- Tách serial transport khỏi odometry.
- `esp32_bridge_node` chịu serial.
- `wheel_odom_node` chỉ tính kinematics.
- Thêm diagnostics.
- Sau đó mới thêm PID/IMU/Nav2.

---

# 4. SOURCE OF TRUTH CHO VIBE CODING

AI phải đọc ít nhất:

```text
.cursor/rules/*
docs/STATUS.md
docs/interfaces.md
docs/calibration.md
```

## 4.1. `docs/STATUS.md`

Ví dụ:

```markdown
# Current Robot Status

## Verified
- [x] Ubuntu Server boot
- [x] SSH `waro@warotrans.local`
- [x] MDD10A manual board-button test
- [x] Motors rotate

## In progress
- [ ] ROS 2 Jazzy
- [ ] RPLIDAR `/scan`

## Not started
- [ ] Encoder
- [ ] `/odom`
- [ ] SLAM
- [ ] BNO055
- [ ] PID
- [ ] Nav2
- [ ] Camera
- [ ] Fleet

## Known issues
- none
```

Mỗi khi hardware/software đạt checkpoint mới → cập nhật file này.

---

## 4.2. `docs/interfaces.md`

ROS interface phải được xem là **contract**:

```text
/scan              sensor_msgs/msg/LaserScan
/odom              nav_msgs/msg/Odometry
/cmd_vel           geometry_msgs/msg/Twist
/imu/data           sensor_msgs/msg/Imu
/map                nav_msgs/msg/OccupancyGrid
/camera/image_raw   sensor_msgs/msg/Image
```

Serial contract hiện tại:

```text
ESP32 → Pi
O <tickL> <tickR> <millis>\n

Pi → ESP32
V <linear_mps> <angular_rps>\n
R\n
```

Không tự đổi protocol mà không sửa cả hai đầu và tài liệu.

---

# 5. QUY TRÌNH VIBE CODING CHUẨN

```text
1. git status
2. commit trạng thái đang chạy
3. AI đọc rules + STATUS + interfaces
4. giao MỘT task
5. AI lập plan
6. review plan
7. AI sửa
8. git diff
9. tools/check.sh
10. deploy
11. hardware test
12. cập nhật STATUS/calibration
13. commit
```

## Prompt chuẩn

```text
Context:
@docs/STATUS.md
@docs/interfaces.md
@file-cần-sửa

Goal:
Publish ổn định `/scan` từ RPLIDAR.

Constraints:
- Chưa sửa firmware.
- Chưa sửa TF.
- Không hardcode physical constant mới.
- Không thêm dependency nếu chưa giải thích.

Before editing:
Liệt kê file sẽ sửa và lý do.

Definition of Done:
- `ros2 topic hz /scan` ổn định.
- `ros2 topic echo /scan --once` có ranges hợp lệ.
- Node không crash nếu thiết bị tạm thời mất.
```

## Debug prompt chuẩn

```text
Triệu chứng:
...

Evidence:
...

Expected:
...

Do not edit code yet.
Xếp nguyên nhân theo xác suất và cho lệnh kiểm tra từng nguyên nhân.
```

---

# 6. PHASE 0 — POWER, NETWORK, UBUNTU

Trạng thái đầu vào hiện tại:

```text
Ubuntu Server 24.04
user: waro
hostname: warotrans
SSH qua warotrans.local
```

Kiểm tra:

```bash
hostname
hostname -I
uname -a
lsblk
df -h
free -h
```

Health:

```bash
vcgencmd get_throttled
cat /sys/class/thermal/thermal_zone0/temp
```

## 6.1. USB current budget — bắt buộc khi Pi không dùng nguồn USB-C chính hãng

Pi 5 xác định ngân sách dòng cho cổng USB bằng cách đàm phán USB-PD trên cổng USB-C.
Nếu Pi được cấp nguồn qua **chân GPIO 5V** hoặc qua một nguồn không đàm phán được PD,
Pi giả định nguồn yếu và **giới hạn tổng dòng cho toàn bộ cổng USB xuống 600 mA**
thay vì 1.6 A.

RPLIDAR A1 lúc khởi động (motor quay + laser bật) cộng với ESP32 có thể vượt ngưỡng đó.
Triệu chứng: LiDAR quay được vài giây rồi dừng, hoặc `/scan` chập chờn, hoặc thiết bị
tự enumerate lại — và bạn sẽ đi tìm lỗi ở tầng ROS trong khi lỗi nằm ở nguồn.

```bash
sudo nano /boot/firmware/config.txt
```

Thêm:

```text
[all]
usb_max_current_enable=1
```

```bash
sudo reboot
```

Kiểm chứng sau reboot:

```bash
vcgencmd get_throttled            # kỳ vọng: throttled=0x0
vcgencmd pmic_read_adc EXT5V_V    # kỳ vọng: ~5.0-5.1 V
dmesg | grep -i "over-current\|usb.*disconnect"
```

`get_throttled` khác `0x0` nghĩa là đang sụt áp: dây cấp nguồn quá mảnh hoặc quá dài.

> Nếu Pi đang chạy bằng nguồn USB-C chính hãng 27 W thì bước này chưa bắt buộc,
> nhưng vẫn nên thêm vì robot sẽ chuyển sang nguồn onboard.

### DoD Phase 0

- [ ] Pi boot ổn.
- [ ] SSH bằng `ssh waro@warotrans.local`.
- [ ] Không cần dò IP thủ công.
- [ ] Không undervoltage/throttling bất thường.
- [ ] Internet hoạt động.
- [ ] `usb_max_current_enable=1` đã có trong `/boot/firmware/config.txt` (nếu Pi không dùng nguồn USB-C PD chính hãng).

---

# 7. PHASE 1 — CÀI ROS 2 JAZZY

```bash
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe -y

export ROS_APT_SOURCE_VERSION=$(curl -s \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F "tag_name" | awk -F\" '{print $4}')

curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"

sudo apt install -y /tmp/ros2-apt-source.deb
sudo apt update
```

Cài:

```bash
sudo apt install -y \
  ros-jazzy-ros-base \
  ros-dev-tools \
  ros-jazzy-rplidar-ros \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-map-server \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-tf2-tools \
  ros-jazzy-teleop-twist-keyboard \
  python3-serial
```

Environment:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
ros2 --help
```

Workspace:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

sudo rosdep init
# Nếu báo đã initialized thì bỏ qua.
rosdep update
```

### DoD Phase 1

```bash
ros2 --help
ros2 pkg list | grep slam_toolbox
ros2 pkg list | grep rplidar_ros
```

đều PASS.

---

# 8. PHASE 2 — RPLIDAR A1 → `/scan`

## 8.1. Cắm thiết bị

```bash
sudo usermod -aG dialout $USER
```

Logout SSH rồi login lại.

```bash
lsusb
ls -l /dev/ttyUSB*
```

Không đoán vendor/product. Lấy từ máy thật:

```bash
udevadm info -a -n /dev/ttyUSB0 \
  | grep -E "idVendor|idProduct|serial" | head -10
```

## 8.2. Udev stable name

Tạo:

```bash
sudo nano /etc/udev/rules.d/99-warotrans.rules
```

Chỉ điền `idVendor/idProduct/serial` sau khi đã đọc từ thiết bị thật.

Mục tiêu cuối:

```text
/dev/rplidar
/dev/warotrans
```

Sau khi sửa:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -l /dev/rplidar
```

## 8.3. Chạy driver độc lập

```bash
ros2 run rplidar_ros rplidar_composition --ros-args \
  -p serial_port:=/dev/rplidar \
  -p serial_baudrate:=115200 \
  -p frame_id:=laser \
  -p angle_compensate:=true
```

Hai tham số dễ sai nhất:

| Tham số | Lưu ý |
|---|---|
| `serial_baudrate` | A1 dùng 115200. Các model khác trong cùng họ dùng baud khác. Sai baud → `operation timed out`. |
| `scan_mode` | **Cố tình không đặt ở đây.** Không phải mọi scan mode đều tồn tại trên mọi model; đặt một mode mà firmware LiDAR không hỗ trợ sẽ làm node không khởi động. Chỉ đặt sau khi liệt kê được mode thật mà thiết bị báo lúc chạy. |

Nếu bạn hoặc AI copy một `scan_mode` từ tutorial và node chết ngay lúc start, đây là
nguyên nhân đầu tiên cần loại trừ.

Terminal khác:

```bash
ros2 topic hz /scan
ros2 topic echo /scan --once
```

### DoD Phase 2

- [ ] LiDAR quay.
- [ ] `/scan` tồn tại.
- [ ] Tần số scan ổn định.
- [ ] `ranges` có dữ liệu hợp lệ.
- [ ] Rút LiDAR → node báo lỗi có ý nghĩa, không làm hỏng toàn hệ thống.

> Chưa SLAM ở đây.

---

# 9. PHASE 3 — VISUALIZATION

Pi chạy Ubuntu Server → tránh ép Pi làm desktop đầy đủ nếu không cần.

Có hai lựa chọn:

### Lựa chọn A — RViz qua remote desktop
Tốt khi cần TF/RobotModel/debug ROS trực tiếp.

### Lựa chọn B — Foxglove trên laptop
Tốt cho dashboard, topic visualization và camera.

**Khuyến nghị:** dùng RViz cho TF/Nav2 tuning; Foxglove cho dashboard/telemetry.

Trước SLAM:
- Fixed Frame = `laser`.
- Hiển thị `/scan`.

Sau SLAM:
- Fixed Frame = `map`.

### DoD Phase 3

- [ ] Laptop nhìn thấy scan 360° realtime.
- [ ] Đặt một hộp trước LiDAR → điểm xuất hiện đúng hướng.

---

# 10. PHASE 4 — ESP32 LOW-LEVEL

Firmware phải giữ các bất biến an toàn:

1. PWM = 0 khi boot.
2. Watchdog lệnh luôn hoạt động.
3. Mất command → STOP.
4. ISR không làm việc nặng.
5. Không `delay()` trong main control loop.
6. Không đổi GPIO nếu chưa xác nhận wiring.
7. Serial protocol tương thích ngược.

## 10.1. Serial contract

```text
O <tickL> <tickR> <millis>
```

50 Hz là target hợp lý cho prototype hiện tại.

Pi gửi:

```text
V <linear_mps> <angular_rps>
R
```

## 10.2. Encoder

Không đoán màu dây.

Quy trình:
- xác định motor pair;
- xác định VCC/GND;
- xác định A/B;
- kiểm tra logic voltage;
- chỉ sau đó nối ESP32.

### DoD Phase 4

- [ ] ESP32 hiện `/dev/warotrans`.
- [ ] `cat /dev/warotrans` hoặc tool test thấy tick.
- [ ] quay bánh trái → tick trái đổi đúng dấu.
- [ ] quay bánh phải → tick phải đổi đúng dấu.
- [ ] mất command → motor dừng theo watchdog.

---

# 11. PHASE 5 — WHEEL ODOMETRY

Mục tiêu:

```text
encoder ticks
    ↓
wheel odometry
    ↓
/odom
    ↓
TF odom → base_footprint
```

Kinematics differential/skid-steer:

```text
dL = quãng đường bánh trái
dR = quãng đường bánh phải

dS     = (dR + dL) / 2
dTheta = (dR - dL) / wheel_separation
```

Pose cập nhật:

```text
x     += dS * cos(theta + dTheta/2)
y     += dS * sin(theta + dTheta/2)
theta += dTheta
```

Các physical parameters phải là ROS parameters, không hardcode sâu trong code.

Ví dụ:

```text
ticks_per_rev
wheel_radius
wheel_separation
base_frame
odom_frame
serial_port
```

### DoD Phase 5

```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
ros2 run tf2_tools view_frames
```

TF phải là:

```text
odom → base_footprint → base_link → laser
```

**Không phải:**

```text
odom → base_link
```

nếu URDF đã có `base_footprint → base_link`.

---

# 12. PHASE 6 — URDF / ROBOT DESCRIPTION

URDF chứa:
- footprint/base.
- wheel links.
- laser.
- camera khi đã xác nhận vị trí.

Không dùng URDF để "ước lượng đẹp mắt". Pose sensor phải đo trên robot thật.

LiDAR:
- `lidar_x`
- `lidar_y`
- `lidar_z`
- `lidar_yaw`

Test:

```bash
ros2 run tf2_ros tf2_echo base_link laser
```

Đặt hộp trước mũi xe.

Nếu scan không nằm +X:
- kiểm tra pose thật;
- sửa `lidar_yaw` dựa trên measurement;
- không sửa scan data để "xoay cho đúng".

### DoD Phase 6

- [ ] RobotModel đúng hướng.
- [ ] LaserScan trùng với môi trường vật lý.
- [ ] TF tree không loop/duplicate.
- [ ] `view_frames` không có frame orphan quan trọng.

---

# 13. PHASE 7 — CALIBRATION

Đây là phase quyết định chất lượng SLAM.

## 13.1. ticks_per_rev

```text
ticks_per_rev = (tick_after - tick_before) / số_vòng
```

Quay 10 vòng, làm ít nhất 3 lần.

Điền vào:

```text
docs/calibration.md
```

## 13.2. wheel_radius

Đo khi xe chịu tải.

Kiểm tra thực nghiệm bằng đoạn thẳng 2.00 m.

```text
wheel_radius_new
  = wheel_radius_old × (distance_real / distance_odom)
```

Mục tiêu ban đầu:

```text
2.00 m thật → odom 1.98–2.02 m
```

## 13.3. effective wheel_separation

Skid-steer có trượt → không lấy số hình học làm truth cuối.

Xoay đúng 360°.

```text
wheel_separation_new
  = wheel_separation_old × (yaw_odom / yaw_real)
```

Lặp tới khi sai số góc đạt tiêu chí.

Không dự đoán trước parameter sẽ tăng hay giảm. Tin measurement.

### DoD Phase 7

- [ ] ticks/rev lặp lại ổn định.
- [ ] straight-line test đạt.
- [ ] rotation test đạt.
- [ ] mọi kết quả có record ngày/phương pháp.

---

# 14. PHASE 8 — SLAM TOOLBOX

Input:

```text
/scan
/odom
TF odom → base_footprint → base_link → laser
```

Output:

```text
/map
TF map → odom
```

Kiến trúc:

```text
RPLIDAR ──► /scan ──────┐
                        │
Encoder ───► /odom ─────┼──► slam_toolbox ─► /map
                        │
URDF ──────► TF ────────┘
```

Chạy launch của project:

```bash
ros2 launch warotrans_bringup mapping.launch.py
```

hoặc launch riêng:

```bash
ros2 launch warotrans_slam slam.launch.py
```

Tên launch cuối cùng phải thống nhất trong repo.

## 14.1. Mapping procedure

1. Khu vực ít người chuyển động.
2. Chạy chậm.
3. Hạn chế xoay gắt tại chỗ.
4. Ưu tiên vòng cung rộng.
5. Quét toàn khu vực.
6. Quay lại khu vực xuất phát để tạo loop closure.
7. Nếu map mờ/double wall → debug odom/TF trước, không tune SLAM ngẫu nhiên.

## 14.2. Lưu map

```bash
mkdir -p ~/maps

ros2 run nav2_map_server map_saver_cli \
  -f ~/maps/warotrans \
  --ros-args -p save_map_timeout:=10000.0
```

Kết quả:

```text
warotrans.pgm
warotrans.yaml
```

### DoD Phase 8

- [ ] `/map` publish.
- [ ] TF `map → odom` tồn tại.
- [ ] map loop closure hợp lý.
- [ ] tường không double rõ rệt.
- [ ] kích thước map gần kích thước thật.
- [ ] map lưu được và load lại được.

---

# 15. MAP QUALITY GATE

Map chưa đạt → **không sang Nav2**.

Kiểm tra:

| Test | PASS |
|---|---|
| Tường chính | một lớp, không ghost/double rõ |
| Kích thước | gần số đo thật |
| Góc | hợp lý với môi trường |
| Loop closure | đường về gần khép kín |
| Robot pose | không nhảy vô lý |
| Scan overlay | bám vào tường map |

Thứ tự debug:

```text
1. TF
2. timestamp
3. encoder
4. wheel calibration
5. LiDAR pose
6. tốc độ chạy
7. SLAM tuning
```

**Không tune slam_toolbox trước khi 1–5 đúng.**

---

# 16. PHASE 9 — BNO055 + ROBOT_LOCALIZATION

Chỉ thêm sau khi wheel odometry baseline đã chạy.

Pipeline:

```text
wheel odom ─────┐
                ├──► EKF ──► /odometry/filtered
BNO055 /imu ────┘
                        │
                        └──► TF odom → base_footprint
```

Mục tiêu:
- encoder tốt cho translation;
- IMU hỗ trợ orientation/yaw;
- giảm drift khi skid-steer trượt.

Khi bật EKF:
- wheel node dừng publish TF `odom → base_footprint`;
- EKF là publisher duy nhất.

### DoD Phase 9

- [ ] IMU frame đúng.
- [ ] orientation không đảo trục.
- [ ] filtered odometry không nhảy.
- [ ] chỉ một publisher cho `odom → base_footprint`.

---

# 17. PHASE 10 — CLOSED-LOOP MOTOR PID

Trước Nav2 production-quality, motor nên có closed-loop wheel speed.

Open loop:

```text
/cmd_vel → PWM
```

Closed loop:

```text
/cmd_vel
   ↓
target wheel velocity
   ↓
PID ◄── encoder velocity
   ↓
PWM
```

Mục tiêu:
- pin yếu dần nhưng tốc độ vẫn gần command;
- hai bên cân hơn;
- controller của Nav2 nhận robot response ổn định.

Không tune PID bằng cảm giác.

Test riêng:
- step response;
- steady-state error;
- overshoot;
- command zero;
- reverse direction;
- watchdog.

### DoD Phase 10

- [ ] command tốc độ → tốc độ bánh bám ổn.
- [ ] STOP an toàn.
- [ ] không rung mạnh quanh zero.
- [ ] mất serial/Wi-Fi → stop.
- [ ] left/right response tương đối cân.

---

# 18. PHASE 11 — LOCALIZATION TRÊN MAP ĐÃ LƯU

Mapping mode và navigation mode là hai trạng thái khác nhau.

### Mapping
```text
SLAM Toolbox online mapping
→ tạo /map
```

### Normal navigation
```text
Map Server
+
AMCL hoặc SLAM localization mode
→ robot biết pose trên map tĩnh
```

Khuyến nghị ban đầu:
- map tĩnh → Map Server.
- AMCL cho localization 2D LiDAR.
- Nav2 dùng pose từ localization.

### DoD Phase 11

- [ ] map load được.
- [ ] set initial pose.
- [ ] robot di chuyển → pose trên map theo đúng.
- [ ] quay về điểm cũ → localization không drift lớn.

---

# 19. PHASE 12 — NAV2

Nav2 không phải "một node".

Khái niệm:

```text
Goal
 ↓
Behavior Tree Navigator
 ↓
Planner Server ──► Global path
 ↓
Controller Server ──► local velocity commands
 ↓
/cmd_vel
 ↓
ESP32 PID
 ↓
motors
```

Song song:

```text
Global Costmap
Local Costmap
Obstacle data từ LiDAR
```

## 19.1. Navigation gate

Trước khi bật Nav2:

- [ ] `/scan` ổn.
- [ ] `/odom` ổn.
- [ ] localization ổn.
- [ ] map tốt.
- [ ] TF tree đúng.
- [ ] `/cmd_vel` tới được ESP32.
- [ ] closed-loop motor đủ ổn.
- [ ] footprint đúng kích thước robot.
- [ ] stop/watchdog hoạt động.

## 19.2. Không hardcode thông số Nav2 từ ví dụ

Các số như:
- inflation radius;
- footprint;
- max velocity;
- acceleration;
- controller frequency;
- costmap resolution;

phải:
1. bắt đầu từ giá trị có lý do;
2. test trên robot thật;
3. record quyết định.

### DoD Phase 12

- [ ] click goal trong RViz → robot tới goal.
- [ ] tránh obstacle tĩnh.
- [ ] gặp obstacle động → giảm tốc/dừng/replan phù hợp.
- [ ] không va chạm ở tốc độ demo.
- [ ] abort/recovery có log rõ.

---

# 20. PHASE 13 — CAMERA

Camera là subsystem riêng, **không phải điều kiện bắt buộc cho 2D LiDAR SLAM**.

Trước khi code:
- xác nhận model camera;
- xác nhận interface: CSI hay USB;
- xác nhận driver tương thích Ubuntu hiện tại.

Pipeline:

```text
Camera
  ↓
camera driver
  ↓
/camera/image_raw
  ↓
Foxglove / Web
  ↓
(optional)
AI perception
```

Không ghép vision vào navigation ngay.

Thứ tự:
1. camera stream.
2. latency.
3. frame rate.
4. remote viewing.
5. record rosbag.
6. sau đó mới object/QR detection.

### DoD Phase 13

- [ ] image topic ổn định.
- [ ] laptop xem realtime.
- [ ] camera disconnect không làm crash robot.
- [ ] bandwidth không phá ROS traffic quan trọng.

---

# 21. PHASE 14 — FLEET INTEGRATION

Mục tiêu capstone không phải remote-control đơn thuần.

Sai:

```text
Backend → PWM
```

Đúng:

```text
Warehouse request
      ↓
Fleet Manager
      ↓
Assign Robot
      ↓
Goal / Task
      ↓
Robot Fleet Bridge
      ↓
Nav2 NavigateToPose
      ↓
Robot tự đi
      ↓
Robot reports status/result
```

## 21.1. Fleet bridge responsibilities

```text
Backend → Robot
- task_id
- pickup / destination
- cancel/pause/resume
- configuration phù hợp

Robot → Backend
- robot_id
- online/offline
- pose
- battery
- current task
- navigation state
- error
- completed/failed
```

**Không khóa kiến trúc vào VDA5050 quá sớm.**

Thiết kế:

```text
warotrans_fleet/
    domain model
    task state machine
    fleet bridge
    protocol adapter
```

Nếu sau này thật sự cần VDA5050:
- implement adapter;
- giữ core robot contract không phụ thuộc trực tiếp standard.

---

# 22. PHASE 15 — MULTI-ROBOT

Chỉ làm khi **một robot chạy end-to-end ổn**.

Mỗi robot cần:
- unique `robot_id`.
- unique hostname.
- unique backend identity.
- namespace hoặc domain strategy rõ ràng.
- task ownership rõ ràng.

Ví dụ:

```text
robot_01
robot_02
robot_03
```

Fleet manager quyết định:
- robot nào nhận task;
- tránh assign robot lỗi;
- workload;
- queue;
- conflict ở tầng hệ thống.

Nav2 của từng robot vẫn chịu trách nhiệm navigation cục bộ.

---

# 23. RELIABILITY / PRODUCTION-STYLE BRINGUP

Khi prototype ổn, chuyển từ:

```text
SSH vào Pi
→ chạy 6 terminal
```

sang:

```text
Pi boot
→ services start
→ ROS bringup
→ hardware connected
→ diagnostics healthy
→ robot READY
```

Cần:
- systemd cho bringup phù hợp;
- restart policy có kiểm soát;
- log rotation;
- health check;
- diagnostics;
- không restart motor controller một cách nguy hiểm.

Robot state machine tối thiểu:

```text
BOOTING
CONNECTING_HARDWARE
READY
EXECUTING
PAUSED
ERROR
ESTOP
OFFLINE
```

---

# 24. OBSERVABILITY

Không debug robot bằng "nhìn nó chạy".

Thu thập:

```text
/diagnostics
/scan rate
/odom rate
/cmd_vel
battery voltage
ESP32 heartbeat
serial reconnect count
CPU temperature
Pi throttling
navigation result
task state
```

Rosbag khi debug khó:

```bash
ros2 bag record \
  /scan \
  /odom \
  /tf \
  /tf_static \
  /cmd_vel
```

Sau này thêm:
- IMU.
- camera theo nhu cầu.
- Nav2 status.

Rosbag giúp replay lỗi mà không cần chạy robot lại.

---

# 25. TEST PYRAMID

```text
                 Hardware-in-loop
                      /\
                     /  \
              Integration ROS tests
                  /        \
             Unit tests    Static checks
```

## Firmware
- serial parser.
- quadrature decoder.
- velocity conversion.
- PID math.
- watchdog logic.

## ROS
- serial line parsing.
- tick → distance.
- kinematics.
- parameter validation.
- launch tests nếu cần.

## Hardware-in-loop
- encoder spin.
- straight 2m.
- rotate 360°.
- stop/watchdog.
- LiDAR scan.
- Nav2 goal.

---

# 26. `tools/check.sh` — QUALITY GATE

Trước deploy:

```bash
#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/jazzy/setup.bash 2>/dev/null || true

cd ros2_ws
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

Khi repo lớn hơn có thể thêm:
- Python lint.
- type checks.
- PlatformIO build/test.
- YAML validation.

Nguyên tắc:

```text
check FAIL
→ không deploy lên robot thật
```

---

# 27. DEPLOYMENT AN TOÀN

Laptop Git repo = source of truth.

Không để:
- file code quan trọng chỉ tồn tại trên Pi;
- deploy script xóa dữ liệu calibration/map;
- source cũ "ghost" trên Pi.

Các thư mục runtime không được sync-delete chung với source:

```text
~/maps
~/bags
~/logs
calibration records
```

Trước `rsync --delete`, nên có dry-run hoặc cảnh báo rõ phạm vi.

---

# 28. GIT WORKFLOW

Không cần GitFlow phức tạp.

Đủ dùng:

```text
main
feature/lidar
feature/odom
feature/slam
feature/nav2
fix/serial-reconnect
```

Commit nhỏ:

```text
feat(lidar): publish stable /scan
fix(tf): make base_footprint child of odom
feat(odom): add reset odometry service
docs(calibration): record wheel radius test
```

Không commit kiểu:

```text
fix everything
final final v3
code new
```

---

# 29. ADR — ARCHITECTURE DECISION RECORD

Những quyết định lớn nên ghi ở:

```text
docs/adr/
```

Ví dụ:

```text
0001-use-ros2-jazzy.md
0002-use-esp32-low-level-controller.md
0003-use-slam-toolbox.md
0004-use-amcl-for-static-map-localization.md
0005-fleet-protocol-strategy.md
```

ADR ghi:
- context;
- decision;
- alternatives;
- consequence.

Điều này cực hữu ích cho báo cáo capstone.

---

# 30. TROUBLESHOOTING TREE

## LiDAR không có `/scan`

```text
lsusb?
 ├─ NO → cable/power/device
 └─ YES
     ↓
/dev/rplidar?
 ├─ NO → udev/permission
 └─ YES
     ↓
driver running?
     ↓
baud/port/frame
```

## `/scan` có nhưng SLAM không map

```text
/scan valid?
 ↓
TF laser connected?
 ↓
/odom valid?
 ↓
timestamps?
 ↓
calibration?
 ↓
slam config
```

## map double wall

```text
wheel separation
wheel slip
timestamp
TF duplicate
LiDAR pose
robot speed
```

## Nav2 ra `/cmd_vel` nhưng xe không chạy

```text
/cmd_vel exists?
 ↓
bridge subscribed?
 ↓
serial command sent?
 ↓
ESP32 watchdog?
 ↓
PWM?
 ↓
MDD10A?
```

Debug theo pipeline, không sửa ngẫu nhiên.

---

# 31. CHECKLIST MASTER

## Foundation
- [ ] Ubuntu ổn.
- [ ] SSH ổn.
- [ ] Git repo chuẩn.
- [ ] Cursor rules.
- [ ] STATUS.md.
- [ ] interfaces.md.
- [ ] calibration.md.
- [ ] check/deploy/doctor tools.

## Sensors
- [ ] RPLIDAR stable udev name.
- [ ] `/scan`.
- [ ] visual scan.
- [ ] camera exact model verified.
- [ ] camera stream.
- [ ] BNO055.

## Base
- [ ] ESP32 stable serial.
- [ ] encoder signs.
- [ ] `/odom`.
- [ ] watchdog.
- [ ] PID.

## Geometry
- [ ] URDF.
- [ ] TF.
- [ ] LiDAR pose measured.
- [ ] footprint measured.

## Calibration
- [ ] ticks/rev.
- [ ] straight distance.
- [ ] rotation.
- [ ] IMU orientation.

## Mapping
- [ ] SLAM.
- [ ] loop closure.
- [ ] map quality gate.
- [ ] map save/load.

## Navigation
- [ ] localization.
- [ ] global planner.
- [ ] controller.
- [ ] local/global costmap.
- [ ] obstacle test.
- [ ] goal navigation.

## Fleet
- [ ] robot ID.
- [ ] task state machine.
- [ ] status reporting.
- [ ] goal dispatch.
- [ ] cancel/pause.
- [ ] failure reporting.
- [ ] multi-robot only after single robot passes.

---

# 32. DEFINITION OF DONE CHO MỘT ROBOT WAROTRANS

Một robot được coi là **autonomous prototype hoàn chỉnh** khi:

1. Boot độc lập.
2. Kết nối network.
3. Hardware nodes tự khởi động.
4. LiDAR publish ổn định.
5. Odometry đúng trong tolerance đã ghi.
6. Localization ổn định.
7. Map kho load thành công.
8. Nhận goal.
9. Nav2 lập đường.
10. Robot đi tới goal.
11. Obstacle xuất hiện → phản ứng an toàn.
12. Camera xem được từ operator UI.
13. Backend nhận trạng thái realtime.
14. Task complete/fail được báo lại.
15. Mất network/control command → robot không chạy mất kiểm soát.
16. Logs đủ để điều tra lỗi.

---

# 33. THỨ TỰ THỰC HIỆN THỰC TẾ TỪ TRẠNG THÁI HIỆN TẠI

Bạn **không cần làm toàn bộ tài liệu ngay**.

Bắt đầu:

```text
NEXT 1
Cài ROS 2 Jazzy
   ↓
NEXT 2
RPLIDAR → /scan
   ↓
NEXT 3
visualize scan
   ↓
NEXT 4
ESP32 encoder
   ↓
NEXT 5
/odom + TF
   ↓
NEXT 6
calibration
   ↓
NEXT 7
SLAM map đầu tiên
```

Sau khi có **map đầu tiên đạt quality gate**:

```text
refactor hardware package
→ BNO055/EKF
→ PID
→ Localization
→ Nav2
→ Camera
→ Fleet
```

Đây là cách cân bằng giữa:
- kiến trúc chuyên nghiệp;
- không over-engineer quá sớm;
- vẫn đạt milestone robot thật nhanh.

---

# 34. QUY TẮC CUỐI CÙNG CHO AI / VIBE CODING

AI được phép sáng tạo ở:
- implementation;
- test;
- refactor cục bộ;
- logging;
- documentation;
- tooling.

AI **không được tự quyết**:
- physical constants;
- GPIO;
- TF contract;
- voltage/current;
- watchdog removal;
- serial protocol breaking change;
- major architecture migration;
- safety behavior.

Khi gặp một trong các mục trên:

```text
STOP
→ explain
→ show impacted files
→ propose measurement/test
→ wait for approval
```

---


---

# APPENDIX A — CURRENT HARDWARE CONTRACT

Các mục dưới đây lấy từ bộ guide/devkit hiện tại. Chỉ coi là **CONFIRMED** sau khi đối chiếu đúng phần cứng đang cầm trên tay.

| Thành phần | Trạng thái hiện tại |
|---|---|
| Raspberry Pi | Raspberry Pi 5, Ubuntu Server 24.04 |
| User / hostname | `waro` / `warotrans` |
| ESP32 | ESP32-S3 MKE-K01 N16R8 |
| Motor driver | Cytron MDD10A |
| Drive | skid-steer 4 bánh |
| Motor | JGB37-520 có encoder |
| LiDAR | RPLIDAR A1M8-R6 |
| IMU | GY-BNO055, chưa tích hợp |
| Chassis | 320 mm × 260 mm, mica 3 mm |
| Wheel | Ø65 mm |
| Camera | **CẦN XÁC NHẬN model/interface chính xác trước khi code driver** |

## GPIO contract hiện tại trong devkit

```text
DIR1        GPIO 4
PWM1        GPIO 5
DIR2        GPIO 6
PWM2        GPIO 7

Encoder L A GPIO 8
Encoder L B GPIO 9
Encoder R A GPIO 10
Encoder R B GPIO 11
```

Trước khi đấu:
1. mở firmware hiện tại;
2. đối chiếu đúng board;
3. đối chiếu dây thật;
4. chỉ sau đó mới cấp nguồn.

Không đổi pinout để "code đẹp hơn".

---

# APPENDIX B — NHẬN DIỆN ENCODER AN TOÀN

Motor JGB37-520 encoder thường có nhiều dây, nhưng **không dùng màu dây từ Internet làm truth**.

## B1. Tìm cặp motor

Dùng đồng hồ đo điện trở.

Cặp dây motor thường:
- có điện trở thấp;
- phản ứng khi xoay trục.

Có thể xác nhận bằng nguồn điện áp thấp thích hợp trong thời gian rất ngắn, nhưng chỉ thực hiện khi đã hiểu rõ wiring.

## B2. Tìm encoder power và channels

Sau khi loại cặp motor:
- xác định VCC/GND theo module thực tế;
- xác nhận điện áp encoder;
- đo A/B khi xoay thật chậm.

Mục tiêu:

```text
Channel A: chuyển mức logic
Channel B: chuyển mức logic
A/B lệch pha
```

**Không đưa điện áp motor vào dây encoder.**

## B3. Test sau khi nối ESP32

Kê bánh khỏi mặt đất.

Serial output target:

```text
O <tickL> <tickR> <millis>
```

Quay bánh trái tiến:
- tick trái phải đổi;
- tick phải gần như không đổi.

Quay bánh phải tiến:
- tick phải phải đổi.

Nếu dấu ngược:
- sửa direction/invert ở đúng một tầng;
- ghi lại convention trong `docs/interfaces.md`.

---

# APPENDIX C — UDEV RULE MẪU

Không copy vendor/product một cách mù quáng.

Đầu tiên:

```bash
udevadm info -a -n /dev/ttyUSB0 \
  | grep -E "idVendor|idProduct|serial" | head -10
```

Nếu **thiết bị thật** báo đúng CP2102 với:

```text
idVendor  = 10c4
idProduct = ea60
```

thì rule có thể là:

```udev
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0660", GROUP:="dialout", SYMLINK+="rplidar"
```

ESP32 cũng phải lấy vendor/product/serial từ thiết bị thật.

Ưu tiên:
- `GROUP="dialout"` + permission phù hợp;
- không dựa lâu dài vào `chmod 666`.

Reload:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Kiểm tra:

```bash
ls -l /dev/rplidar
ls -l /dev/warotrans
```

---

# APPENDIX D — RVIZ QUA VNC TRÊN UBUNTU SERVER

Nếu muốn chạy RViz trên môi trường GUI tối thiểu ở Pi:

```bash
sudo apt install -y \
  ros-jazzy-rviz2 \
  xfce4 \
  xfce4-goodies \
  tigervnc-standalone-server \
  dbus-x11
```

Đặt password:

```bash
vncpasswd
mkdir -p ~/.vnc
nano ~/.vnc/xstartup
```

Nội dung:

```sh
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
exec startxfce4
```

Sau đó:

```bash
chmod +x ~/.vnc/xstartup
vncserver :1 -geometry 1600x900 -depth 24 -localhost no
```

Laptop kết nối tới:

```text
warotrans.local:5901
```

Trong terminal của VNC:

```bash
rviz2
```

Khi chỉ test LiDAR:
- Fixed Frame = `laser`.

Khi mapping:
- Fixed Frame = `map`.

> VNC là công cụ debug. Không cần biến Pi thành desktop đầy đủ cho runtime production.

---

# APPENDIX E — SLAM DRIVING PROTOCOL

Chất lượng map phụ thuộc cách lái rất nhiều.

## Trước khi scan

- dọn người/vật di chuyển không cần thiết;
- kiểm tra `/scan`;
- kiểm tra `/odom`;
- kiểm tra TF;
- kiểm tra calibration;
- đảm bảo robot stop command hoạt động.

## Khi scan

1. chạy chậm;
2. tránh acceleration giật;
3. hạn chế spin tại chỗ;
4. cua vòng cung;
5. giữ LiDAR có feature để match;
6. quay lại khu vực đã scan để tạo loop closure;
7. nếu map bắt đầu double wall mạnh → dừng và debug, không cố quét tiếp.

## Đánh giá

| Hiện tượng | Kiểm tra đầu tiên |
|---|---|
| Tường nhân đôi khi quay | wheel separation / wheel slip |
| Map co/giãn theo quãng đường | wheel radius |
| Scan xoay sai hướng | LiDAR yaw / TF |
| Robot nhảy pose | timestamp / duplicate TF |
| Loop không khép | odom drift / scan matching / cách lái |
| Map rung liên tục | TF publishers / timing |

---

# APPENDIX F — NAV2 PROFESSIONAL BASELINE

Không xem Nav2 là hộp đen.

```text
NavigateToPose
      ↓
BT Navigator
      ↓
Planner Server
      ↓
Global Costmap
      ↓
Controller Server
      ↓
Local Costmap
      ↓
/cmd_vel
```

## Planner

Trả lời:

> Từ pose hiện tại tới goal, đường toàn cục nào hợp lệ?

## Controller

Trả lời:

> Trong vài giây tiếp theo robot phải chạy vận tốc nào để bám path và tránh vật cản?

## Costmap

Biểu diễn:
- obstacle;
- inflation;
- footprint;
- vùng free/occupied.

## Behavior Tree

Điều phối:
- plan;
- follow;
- recovery;
- replan;
- clear costmap;
- retry/abort.

## WaroTrans contract với Nav2

Nav2 chỉ nên cần biết:

```text
/map
/scan
/odom
/tf
/cmd_vel
```

Nav2 **không cần biết**:
- MDD10A PWM pin;
- encoder GPIO;
- serial packet chi tiết.

Đó là dấu hiệu abstraction đúng.

---

# APPENDIX G — FLEET TASK STATE MACHINE

Robot task state nên rõ:

```text
IDLE
  ↓
ASSIGNED
  ↓
NAVIGATING_TO_PICKUP
  ↓
WAITING_PICKUP_CONFIRMATION
  ↓
NAVIGATING_TO_DROPOFF
  ↓
WAITING_DROPOFF_CONFIRMATION
  ↓
COMPLETED
```

Error path:

```text
ANY STATE
   ↓
PAUSED / BLOCKED / FAILED / CANCELLED
```

Fleet backend không nên suy luận trạng thái từ topic motor.

Robot gửi event rõ:

```text
TASK_ACCEPTED
PICKUP_REACHED
DROPOFF_REACHED
TASK_COMPLETED
TASK_FAILED
ROBOT_BLOCKED
ROBOT_OFFLINE
```

---

# APPENDIX H — CAMERA INTEGRATION DECISION GATE

Trước camera implementation, điền:

```text
Exact camera model:
Interface: CSI / USB
Driver verified on current Ubuntu:
Expected resolution:
Expected FPS:
Expected operator use:
Expected AI use:
```

Nếu CSI:
- xác minh camera stack với **đúng Ubuntu release hiện tại** trước.

Nếu USB:
- xác minh V4L2 device trước.

Không đổi OS chỉ vì camera cho tới khi đánh giá ảnh hưởng:
- ROS 2 distro;
- packages;
- LiDAR;
- Nav2;
- deployment.

---

# APPENDIX I — `doctor.sh` CHECKLIST

Tool doctor nên kiểm tra tối thiểu:

```text
[POWER]
throttling
CPU temperature

[DEVICES]
/dev/rplidar
/dev/warotrans
dialout membership

[ROS]
rplidar package
slam_toolbox
warotrans packages

[RUNTIME]
topics
TF
node health
```

Chạy:

```bash
./tools/doctor.sh
```

**Doctor output là evidence**, không phải diagnosis cuối cùng.

---

# APPENDIX J — RELEASE GATES

## Gate M0 — Hardware base
- Motor board manual test PASS.
- Emergency stop/power cutoff procedure hiểu rõ.

## Gate M1 — Compute
- Pi + SSH + ROS PASS.

## Gate M2 — Perception
- LiDAR `/scan` PASS.

## Gate M3 — Motion estimate
- Encoder + `/odom` + TF PASS.

## Gate M4 — Mapping
- Valid map saved.

## Gate M5 — Control
- PID/velocity control PASS.

## Gate M6 — Localization
- Static map localization PASS.

## Gate M7 — Autonomous navigation
- Nav2 goal + obstacle test PASS.

## Gate M8 — Operator perception
- Camera live PASS.

## Gate M9 — System integration
- Fleet task end-to-end PASS.

## Gate M10 — Multi-robot
- Chỉ bắt đầu khi M9 của một robot đã ổn định.

---

# APPENDIX K — CÁC ANTI-PATTERN CẦN TRÁNH

### 1. "Fix SLAM" bằng cách tune 20 parameter
Sai nếu chưa xác nhận odometry/TF.

### 2. Nhét mọi node vào `warotrans_bringup`
Sẽ làm package không còn một trách nhiệm rõ.

### 3. Backend gửi PWM
Phá abstraction và làm hệ thống khó mở rộng.

### 4. Hai node cùng publish một TF
Map sẽ rung/nhảy khó chẩn đoán.

### 5. Hardcode `/home/waro/...`
Launch phải dùng package share path.

### 6. Sửa trực tiếp code trên Pi mà không commit laptop
Sinh source drift.

### 7. `rsync --delete` cả thư mục runtime
Có nguy cơ xóa map/log/calibration.

### 8. AI tự đổi GPIO hoặc constants
Nguy hiểm với robot thật.

### 9. Camera + SLAM + Nav2 + fleet cùng một lần
Khi lỗi sẽ không biết subsystem nào sai.

### 10. Không ghi calibration history
Sau một thời gian không thể giải thích tại sao parameter có giá trị đó.

# 35. KẾT LUẬN KIẾN TRÚC

Mục tiêu cuối của WaroTrans không phải:

```text
Phone → motor
```

mà là:

```text
Warehouse Staff
     ↓
Transportation Request
     ↓
Fleet Manager
     ↓
Assign Robot
     ↓
Robot receives Goal
     ↓
Localization + Nav2
     ↓
LiDAR obstacle avoidance
     ↓
ESP32 closed-loop motor control
     ↓
Robot arrives
     ↓
Task status returned to Fleet
```

LiDAR SLAM chỉ là **một tầng** của kiến trúc đó.

Cấu trúc repo, TF contract, calibration log, interface contract, test gates và workflow trong guide này được thiết kế để WaroTrans có thể đi từ **một robot prototype** tới **nhiều robot được fleet manager điều phối** mà không phải viết lại toàn bộ nền tảng.
