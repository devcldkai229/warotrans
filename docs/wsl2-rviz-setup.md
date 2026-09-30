# WSL2 RViz2 Setup for WaroTrans Navigation

Hướng dẫn cài đặt RViz2 trên WSL2 (Windows 11) để visualize và điều khiển
WaroTrans Nav2 stack.

## Prerequisites

- Windows 11 build 26200+ (có WSL2 mirrored networking)
- WSL2 với Ubuntu 24.04
- ROS 2 Jazzy desktop trên WSL2

## Bước 1 — Cấu hình WSL2 mirrored networking

Tạo hoặc sửa file `%USERPROFILE%\.wslconfig` (ví dụ: `C:\Users\User\.wslconfig`):

```ini
[wsl2]
networkingMode=mirrored
```

Sau đó shutdown WSL và khởi động lại:

```powershell
wsl --shutdown
```

## Bước 2 — Cài ROS 2 Jazzy trong WSL2

```bash
# Cài ROS 2 Jazzy desktop (nếu chưa có)
sudo apt update
sudo apt install -y ros-jazzy-desktop

# Cài thêm nav2 messages để có thể dùng action client
sudo apt install -y ros-jazzy-nav2-msgs
```

## Bước 3 — Cấu hình ROS environment

Thêm vào `~/.bashrc` trong WSL:

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Match Pi's DDS settings
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

Reload:

```bash
source ~/.bashrc
```

## Bước 4 — Mở firewall (nếu cần)

Nếu ROS discovery không hoạt động, mở Hyper-V firewall (PowerShell Admin):

```powershell
# Lấy WSL VM setting ID
Get-NetFirewallHyperVVMSetting

# Cho phép inbound cho WSL (thay ID nếu khác)
Set-NetFirewallHyperVVMSetting -Name '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -DefaultInboundAction Allow
```

Hoặc tạm thời tắt Windows Firewall cho profile Domain/Private.

## Commands nhanh (Pi + WSL)

**Không chạy mapping + Nav2 cùng lúc.** Teleop phone (`:8080`) tắt khi Nav2 (xung đột `/cmd_vel`).

### Mapping (SLAM)

```bash
# Pi
bash ~/tools/warotrans-mapping-restart.sh   # hoặc slam-fresh / mapping launch

# WSL (bash, không zsh + setup.bash)
bash /mnt/e/programming/my_project/warotrans/tools/start-rviz-mapping-wsl.sh
```

### Save map

```bash
# Pi
bash ~/tools/save_map.sh ~/maps/warotrans_outdoor
```

### Navigation + RViz (map outdoor + Zenoh TCP)

Windows thường **chặn UDP inbound** từ Pi → FastDDS `ROS_STATIC_PEERS` fail dù ping OK.
Cách đã smoke: **rmw_zenoh_cpp** — router trên Pi `:7447`, WSL client TCP outbound.

```bash
# Pi
bash ~/tools/warotrans-stop-all.sh
SUDO_PASS=12 SKIP_LIDAR_RESET=1 bash ~/tools/warotrans-nav-start-zenoh.sh ~/maps/warotrans_outdoor.yaml

# WSL (bash)
cp /mnt/e/programming/my_project/warotrans/tools/start-rviz-nav-wsl.sh ~/start-rviz-nav-wsl.sh
bash ~/start-rviz-nav-wsl.sh
# RViz config: /mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_navigation/rviz/nav2_warotrans.rviz
```

Acceptance: script in `CONNECT_OK` khi thấy `/map` `/scan` `/tf` (probe dùng `ros2 topic list --no-daemon`).

**Runbook đầy đủ (Pose Estimate, Goal, checklist topic, ultrasonic FRONT/REAR):**  
[nav2-rviz-operator.md](nav2-rviz-operator.md)

Sau smoke / trước khi để máy idle:

```bash
bash ~/tools/warotrans-stop-all.sh
```

### Bật lại khi test ngoài đời

```text
Pi:  SUDO_PASS=12 SKIP_LIDAR_RESET=1 bash ~/tools/warotrans-nav-start-zenoh.sh ~/maps/warotrans_outdoor.yaml
WSL: bash ~/start-rviz-nav-wsl.sh
RViz: Fixed Frame=map → 2D Pose Estimate → Nav2 Goal
```

Laptop LAN IP thường `192.168.100.22` (không dùng `192.168.137.1` ICS). Pi `192.168.100.173`.

Optional FastDDS peers (chỉ khi Windows Firewall Allow UDP từ Pi):

```bash
# Pi
LAPTOP_IP=192.168.100.22 bash ~/tools/warotrans-nav-start-wsl-peers.sh ~/maps/warotrans_outdoor.yaml
# WSL
USE_STATIC_PEERS=1 bash ~/start-rviz-nav-wsl.sh
```

## Bước 5 — Verify kết nối

Luôn dùng **bash** + `--no-daemon` (daemon WSL hay TimeoutError):

```bash
# sau khi start-rviz-nav-wsl.sh in CONNECT_OK, hoặc:
timeout 20 ros2 topic list --no-daemon | grep -E '^/(map|scan|tf)$'
```

Phải thấy ít nhất `/map`, `/scan`, `/tf` (và thường `/amcl_pose`).

Nếu không thấy topic:

1. `ping 192.168.100.173` / cùng Wi‑Fi
2. Pi có `rmw_zenohd` listen `:7447` (`ss -tlnp | grep 7447`)
3. WSL: `ZENOH_SESSION_CONFIG_URI` trỏ path thật (không dùng prefix `file://`)
4. Firewall: UDP peers cần Admin Allow; Zenoh TCP outbound thường OK

## Bước 6 — Chạy RViz2 (navigation)

```bash
bash ~/start-rviz-nav-wsl.sh
# path mặc định:
# /mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_navigation/rviz/nav2_warotrans.rviz
```

Global path topic: `/plan`.

## Bước 7 — Sử dụng RViz cho Navigation

1. **2D Pose Estimate** (toolbar): Click và kéo trên map để set initial pose
   cho AMCL. Particle cloud sẽ hội tụ sau vài giây.

2. **2D Goal Pose / Nav2 Goal** (toolbar → `/goal_pose`): Click điểm đích gần
   (0.5–1 m). Robot plan path và di chuyển.

3. **Quan sát**:
   - Path (màu xanh lá): global plan `/plan`
   - Local Plan (màu cam): `/local_plan`
   - LaserScan (đỏ): `/scan`
   - Costmaps: local và global obstacle layers

4. **Cancel**: Cancel goal trong RViz/action; `/cmd_vel` phải về 0.

## Troubleshooting

### Không thấy topic từ Pi

```bash
ping 192.168.100.173
# Zenoh
ss -tlnp | grep 7447   # trên Pi
bash ~/start-rviz-nav-wsl.sh   # phải in CONNECT_OK trước khi mở RViz
# FastDDS (nếu dùng peers): luôn --no-daemon
pkill -9 -f ros2cli.daemon || true
ros2 topic list --no-daemon
```

### bash vs zsh

`source /opt/ros/jazzy/setup.bash` trong **zsh** dễ lỗi `BASH_SOURCE`. Chạy script bằng `bash`, hoặc `source /opt/ros/jazzy/setup.zsh`.

### TF lookup failed

Đợi vài giây để TF buffer fill. Nếu vẫn fail, check:

```bash
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_footprint
```

### AMCL không localize

- Đặt 2D Pose Estimate gần đúng vị trí thật
- Di chuyển robot (manual push hoặc keyboard teleop) để AMCL update
- Kiểm tra particle cloud convergence

### Nav2 action failed

```bash
# Check lifecycle state
ros2 lifecycle get /amcl
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

Tất cả phải ở state `active`.

## RViz Config Location

Path WSL (ổ E:):

```
/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_navigation/rviz/nav2_warotrans.rviz
```

Hoặc sau build:

```
~/ros2_ws/install/warotrans_navigation/share/warotrans_navigation/rviz/nav2_warotrans.rviz
```
