# WaroTrans — Nav2 + RViz operator runbook

> Vận hành navigation trên map outdoor + WSL RViz (Zenoh).  
> Né vật cản **hôm nay = LiDAR `/scan` → costmap → planner**.  
> Ultrasonic **FRONT/REAR** → Nav2 safety: **chưa bật** đến khi đo mount (xem §5).

**Không** chạy mapping / teleop phone (`:8080`) cùng lúc với Nav2 (`/cmd_vel`).

---

## 1. Bật stack

### 1.1 Pi — Nav2 + Zenoh router

```bash
bash ~/tools/warotrans-stop-all.sh
SUDO_PASS=12 SKIP_LIDAR_RESET=1 \
  bash ~/tools/warotrans-nav-start-zenoh.sh ~/maps/warotrans_outdoor.yaml
```

Outdoor mặc định **tắt** keepout/speed filter (tránh mask Phase 3 lệch map outdoor).  
Phase 3 demo: `ENABLE_COSTMAP_FILTERS=1 bash ~/tools/warotrans-nav-start.sh ~/maps/warotrans.yaml`.

Kiểm router: `ss -tlnp | grep 7447` → `rmw_zenohd` listen.

### 1.2 WSL — RViz (bash, không zsh + `setup.bash`)

```bash
bash ~/start-rviz-nav-wsl.sh
```

Chờ dòng **`CONNECT_OK`** (`/map` `/scan` `/tf`) rồi cửa sổ RViz mở.  
Config: `/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_navigation/rviz/nav2_warotrans.rviz`

Chi tiết Zenoh / firewall: [wsl2-rviz-setup.md](wsl2-rviz-setup.md).

---

## 2. Checklist topic / TF (trước khi Goal)

Chạy trên **Pi** (cùng env Zenoh với stack) hoặc WSL sau `CONNECT_OK`:

| Check | Lệnh / kỳ vọng | OK? |
|---|---|---|
| `/scan` | `ros2 topic hz /scan` → ~7 Hz | |
| `/odom` | `ros2 topic hz /odom` → ~50 Hz | |
| `/map` | `ros2 topic echo /map --once` → width/height > 0 | |
| `/tf` | có trong `ros2 topic list --no-daemon` | |
| CONNECT_OK | script WSL in `CONNECT_OK` | |
| `odom → base_footprint` | `ros2 run tf2_ros tf2_echo odom base_footprint` | |
| `map → odom` | chỉ **sau** 2D Pose Estimate | |

Topic tối thiểu Nav2: `/scan`, `/odom`, `/tf`, `/tf_static`, `/map`, `/amcl_pose`,  
`/cmd_vel`, `/plan`, `/goal_pose`, `/initialpose`.

**Chỉ gửi Nav2 Goal khi** scan + odom + map OK và đã Pose Estimate (map→odom có).

---

## 3. Thao tác RViz

1. **Global Options → Fixed Frame** = `map`.
2. Display **Map** topic `/map` — bản đồ outdoor hiện ra.
3. **LaserScan** `/scan` — điểm laser phải khớp tường/vật trên map; lệch nhiều → chưa đúng pose.
4. Toolbar **2D Pose Estimate**:
   - Click gần vị trí thật robot trên map.
   - Kéo mũi tên theo hướng mũi robot.
   - **ParticleCloud** co cụm; `/amcl_pose` ổn định.
5. Toolbar **Nav2 Goal** / **2D Goal Pose** (`/goal_pose`):
   - Đích gần trước (0.5–1 m) lần đầu outdoor.
   - Thấy **Path** `/plan`; robot đi; `/cmd_vel` ≠ 0.
   - Smoke stall-fix: Goal 1 hết đường → Goal 2 ngay sau (Pose lại nếu laser lệch) — robot phải còn `linear.x`, không chỉ vẽ path.
6. **Né vật (LiDAR — hiện tại):** đặt vật trong tầm LiDAR trên đường đi → local/global costmap cập nhật → path đổi hoặc robot dừng/tránh theo controller. Đây **không** phải ultrasonic.
7. **Goal lần 2+ mà chỉ thấy path, robot đứng:** trên Pi
   `bash ~/tools/warotrans-nav-cancel-clear.sh` → RViz **2D Pose Estimate** lại
   (laser khớp tường) → **2D Goal Pose** ngắn mới. Không bê robot tay rồi Goal ngay.
   (Safety net — stack đã dùng BT resilient + filters off outdoor; vẫn dùng khi costmap bẩn.)
8. Cancel goal nếu cần.

### Sau test

```bash
# Pi
bash ~/tools/warotrans-stop-all.sh
```

---

## 4. Bật lại nhanh (test ngoài đời)

```text
Pi:  SUDO_PASS=12 SKIP_LIDAR_RESET=1 bash ~/tools/warotrans-nav-start-zenoh.sh ~/maps/warotrans_outdoor.yaml
WSL: bash ~/start-rviz-nav-wsl.sh
RViz: Fixed Frame=map → 2D Pose Estimate → Nav2 Goal
Sau: bash ~/tools/warotrans-stop-all.sh
```

---

## 5. Ultrasonic FRONT / REAR (Phase 0.5) — trạng thái & checklist đo

| Sensor | Topic | Nav2 Phase 0.5 |
|---|---|---|
| **FRONT** | `/ultrasonic/front` | **Cần** — vật đột ngột phía mũi |
| **REAR** | `/ultrasonic/rear` | **Cần** — phía đuôi / lùi |
| LEFT | `/ultrasonic/left` | **Không** wire Nav2 |
| RIGHT | `/ultrasonic/right` | **Không** wire Nav2 |

Hiện: topic = monitoring only; không TF mount; Collision Monitor tắt  
(`observation_sources: []` trong `nav2_params.yaml`).

### 5.1 Bạn đo tay (blocker — không bịa số)

1. Thước + photo; ngày đo.
2. Điền **chỉ hàng FRONT và REAR** trong [calibration.md](calibration.md) §4b: `x y z yaw` relative `base_link` (REP-103: +X trước, +Y trái).
3. LEFT/RIGHT: ghi “không dùng Nav2” hoặc để trống.
4. Copy số đo vào `ros2_ws/src/warotrans_description/config/geometry.yaml` → `ultrasonic.front` / `ultrasonic.rear` (không còn `null`).
5. Kiểm: `bash tools/warotrans-check-ultrasonic-front-rear-geometry.sh` → `FRONT_REAR_GEOMETRY_OK`.

### 5.2 Sau khi có số đo (bật an toàn)

Scaffold đã có, **mặc định tắt**:

| Thành phần | Cách bật |
|---|---|
| TF `ultrasonic_front` / `ultrasonic_rear` | Tự thêm trong URDF khi `geometry.yaml` front+rear đủ số (LEFT/RIGHT không thêm) |
| Node GPIO | `start_ultrasonic:=true` trên `navigation.launch.py` / nav-start |
| Collision Monitor FRONT+REAR | Chỉ sau TF OK — xem [ultrasonic-front-rear-nav2.md](ultrasonic-front-rear-nav2.md) (đổi `/cmd_vel` path — safety gate) |

DoD smoke (khi đã enable): vật thấp trước mũi → FRONT range giảm → stop hoặc replan; LEFT/RIGHT không nằm acceptance.

---

## 6. Không claim PASS khi

- Chưa Pose Estimate → Goal thật ngoài đời với robot chuyển động.
- Chưa đo FRONT/REAR và chưa smoke ultrasonic safety.
