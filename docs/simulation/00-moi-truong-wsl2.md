# GĐ 0 — Môi trường mô phỏng trên WSL2

> Trạng thái: **XONG** (2026-10-15). Kiểm chứng bằng `scripts/check-env.sh` (kết quả cuối file).

## 1. Vì sao cần WSL2

ROS 2 Jazzy, Nav2 và Gazebo chỉ chạy tốt trên **Linux (Ubuntu 24.04)**. Máy dev là Windows 11, nên ta chạy
một Ubuntu thật bên trong **WSL2** (máy ảo nhẹ của Windows). **WSLg** cho phép cửa sổ Linux (Gazebo, RViz)
hiện thẳng lên màn hình Windows.

Backend .NET, Docker (PostgreSQL, Mongo, **Mosquitto**) vẫn chạy trên Windows như cũ.

```
┌──────────────────────── Windows 11 ─────────────────────────┐
│  Backend .NET (:5090)      Docker: Mosquitto :1883,          │
│  Web admin                         PostgreSQL :5433, Mongo    │
│        ▲  MQTT localhost:1883                                │
│        │                                                     │
│  ┌─────┴──────────── WSL2: Ubuntu 24.04 ──────────────────┐  │
│  │ ROS 2 Jazzy · Nav2 · Gazebo Harmonic · gateway MQTT     │  │
│  │ (ổ đĩa ảo: D:\WaroTransSim\wsl\Ubuntu-24.04\ext4.vhdx)  │  │
│  └─────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────┘
```

## 2. Mạng: vì sao gateway trong Ubuntu gọi được `localhost:1883`

Mặc định WSL2 có địa chỉ IP riêng (NAT), nên `localhost` trong Ubuntu **không phải** Windows.
File `C:\Users\pc\.wslconfig` bật chế độ **mirrored**:

```ini
[wsl2]
networkingMode=mirrored
```

Khi đó Ubuntu dùng chung card mạng với Windows. `localhost:1883` trong Ubuntu chính là cổng 1883 mà Docker
(Mosquitto) mở trên Windows. Nhờ vậy cấu hình gateway giữ `mqtt_host: localhost`.

> Thay đổi `.wslconfig` áp dụng cho mọi distro WSL (kể cả Docker Desktop), và cần `wsl --shutdown` một lần.

## 3. Mọi thứ nằm ở đâu (ổ D)

| Đường dẫn | Nội dung |
|---|---|
| `D:\WaroTransSim\wsl\Ubuntu-24.04\ext4.vhdx` | Toàn bộ Ubuntu: ROS, Gazebo, Nav2, workspace build |
| `D:\WaroTransSim\scripts\` | Script cài đặt / build / chạy / smoke test |
| `D:\WaroTransSim\logs\` | Log cài đặt và smoke test |
| `D:\FPT_edu\...\robotSource\warotrans` | **Code robot** (repo này), sửa trên Windows như thường |
| `~/warotrans_ws` (trong Ubuntu) | Workspace colcon. `src` là **symlink** tới `ros2_ws/src` của repo |

Workspace dùng symlink: bạn sửa code trên Windows (VS Code), rồi build trong Linux (`colcon build`), mà không
phải copy qua lại. Thư mục `build/` và `install/` nằm trong Linux nên build nhanh.

## 4. Các bước đã chạy (có thể chạy lại)

| Script | Chạy với | Làm gì |
|---|---|---|
| `wsl --install Ubuntu-24.04 --location D:\WaroTransSim\wsl\Ubuntu-24.04` | PowerShell | Cài Ubuntu vào ổ D |
| `01-setup-ubuntu-user.sh` | root | Tạo user `waro` (sudo không mật khẩu), bật systemd, đặt user mặc định |
| tạo `.wslconfig` + `wsl --shutdown` | PowerShell | Bật mạng mirrored |
| `02-install-ros-jazzy-gazebo.sh` | root | ROS 2 Jazzy desktop, Nav2, `ros-gz` (Gazebo Harmonic), slam_toolbox, xacro, paho-mqtt, mosquitto-clients |
| `03-user-env.sh` | waro | `~/.bashrc` (source ROS, `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`), tạo `~/warotrans_ws` |
| `04-build-workspace.sh` | waro | Sinh world/map từ layout, rồi `colcon build --packages-up-to warotrans_simulation` |

Lệnh mẫu (PowerShell):

```powershell
wsl -d Ubuntu-24.04 -u root -- bash /mnt/d/WaroTransSim/scripts/02-install-ros-jazzy-gazebo.sh
wsl -d Ubuntu-24.04 -- bash /mnt/d/WaroTransSim/scripts/04-build-workspace.sh
```

> Từ Git Bash phải gọi qua PowerShell, vì Git Bash tự đổi đường dẫn `/mnt/d/...`.

## 5. Kết quả kiểm chứng

```
$ wsl -d Ubuntu-24.04 -- bash /mnt/d/WaroTransSim/scripts/check-env.sh
ROS_DISTRO=jazzy
gz: Gazebo Sim, version 8.15.0          ← Harmonic
nav2 packages: 34
ros_gz: ros_gz_bridge ros_gz_sim
paho-mqtt: 1.6.1
workspace src -> /mnt/d/FPT_edu/CaptoneProject/sourceCode/robotSource/warotrans/ros2_ws/src
WSLg DISPLAY=:0 WAYLAND_DISPLAY=wayland-0
```

## 6. Gỡ bỏ

```powershell
wsl --unregister Ubuntu-24.04
Remove-Item -Recurse D:\WaroTransSim
Remove-Item C:\Users\pc\.wslconfig   # nếu muốn quay về mạng NAT mặc định
```

Tiếp theo: [01-robot-ao-gazebo.md](01-robot-ao-gazebo.md).
