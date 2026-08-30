# WaroTrans — Troubleshooting by Pipeline

Nguyên tắc: **debug từ tầng thấp lên**, không tune tầng cao để che lỗi tầng thấp.

## 1. Không SSH được

```text
Pi boot?
 ↓
Ethernet/Wi-Fi link?
 ↓
IP?
 ↓
mDNS?
 ↓
port 22?
 ↓
authentication?
```

## 2. LiDAR không có `/scan`

```text
lsusb thấy thiết bị?
 ├─ no  → cable / power / adapter / LiDAR
 └─ yes
      ↓
/dev/rplidar tồn tại?
 ├─ no  → udev / permission / device identity
 └─ yes
      ↓
driver chạy?
      ↓
serial port / baud / mode
      ↓
ros2 topic echo /scan --once
```

### 2b. LiDAR quay rồi dừng, hoặc `/scan` chập chờn

Đây thường **không phải lỗi ROS**. Kiểm tra ngân sách dòng USB trước khi đụng tới driver:

```bash
grep usb_max_current_enable /boot/firmware/config.txt
vcgencmd get_throttled
dmesg | grep -i "over-current\|usb.*disconnect\|reset high-speed"
```

Khi Pi 5 được cấp nguồn qua chân GPIO hoặc qua nguồn không đàm phán được USB-PD,
tổng dòng cho các cổng USB bị giới hạn 600 mA. Xem `MASTER_ENGINEERING_GUIDE.md`
mục 6.1.

Loại trừ nhanh: cắm LiDAR qua hub USB có nguồn ngoài, hoặc tạm cấp Pi bằng nguồn
USB-C chính hãng. Nếu triệu chứng biến mất thì nguyên nhân là nguồn, không phải driver.

### 2c. Node chết ngay khi start

Nếu log báo lỗi liên quan scan mode: một số scan mode chỉ có trên vài model trong họ
RPLIDAR. Bỏ tham số `scan_mode` để dùng mặc định của thiết bị, rồi mới thêm lại sau khi
xác nhận mode nào thật sự tồn tại.

## 3. `/scan` có nhưng RViz không thấy

```text
Fixed Frame đúng?
 ↓
TF tới laser tồn tại?
 ↓
QoS/display config?
 ↓
scan ranges hợp lệ?
```

## 4. `/odom` sai

```text
encoder tick ổn?
 ↓
dấu trái/phải?
 ↓
ticks_per_rev?
 ↓
wheel_radius?
 ↓
wheel_separation?
 ↓
timestamp / dt?
```

## 5. Map double wall / méo

Kiểm tra theo thứ tự:

1. duplicate/broken TF;
2. timestamp;
3. encoder mất tick;
4. wheel calibration;
5. LiDAR pose/yaw;
6. wheel slip và tốc độ lái;
7. cuối cùng mới tune SLAM.

## 6. Nav2 có `/cmd_vel` nhưng xe không chạy

```text
/cmd_vel publish?
 ↓
base/bridge subscribe?
 ↓
serial packet gửi?
 ↓
ESP32 parser nhận?
 ↓
watchdog state?
 ↓
PWM/DIR output?
 ↓
MDD10A / motor power?
```

## 7. Robot chạy nhưng bám path kém

Không tăng gain ngẫu nhiên. Kiểm tra:

```text
velocity command
vs
wheel measured velocity
```

Nếu open-loop → hoàn thiện PID trước khi tune Nav2 controller quá sâu.

## 8. Evidence nên gửi khi nhờ AI debug

```text
- triệu chứng chính xác
- command đã chạy
- raw output/error
- `ros2 topic list`
- `ros2 node list`
- `ros2 topic hz` topic liên quan
- `tf2_echo`/view_frames nếu liên quan TF
- hardware wiring/photo/measurement nếu liên quan điện
```

Prompt: **"Đừng sửa code vội; xếp nguyên nhân theo xác suất và cho lệnh loại trừ từng nguyên nhân."**
