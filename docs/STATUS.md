# WaroTrans — Current Robot Status

> Đây là **trạng thái thực tế**, không phải roadmap. Chỉ tick `[x]` sau khi có evidence trên robot thật.
> Mỗi session vibe coding nên đọc file này trước.

**Last reviewed:** 2026-09-12  
**Operator:** waro  
**Robot hostname:** `warotrans`

## VERIFIED

- [x] Raspberry Pi 5 boot được Ubuntu Server 24.04.
- [x] SSH vào Pi bằng user `waro`.
- [x] mDNS/hostname dùng được qua `warotrans.local`.
- [x] MDD10A đã được cấp nguồn và test bằng button trên board.
- [x] Các bánh/motor có thể quay trong bài test thủ công hiện tại.
- [x] Chassis hiện tại: 320 mm × 260 mm, mica 3 mm.
- [x] Wheel nominal diameter: Ø65 mm.
- [x] Wiring encoder front và encoder supply 3.3V đã được operator xác nhận;
      runtime signal/count vẫn chưa kiểm tra.
- [x] ROS 2 Jazzy trên Pi; RPLIDAR `/dev/rplidar`; `/scan` ổn định (~7 Hz).
- [x] `/odom` ổn định (~50 Hz); TF `odom → base_footprint → base_link → laser`.
- [x] Saved map load + AMCL `map → odom` (navigation mode).
- [x] **Phase 0 Nav2 RViz real-robot PASS** (2026-09-11): NavigateToPose →
      path → `/cmd_vel` → ESP32 → robot tới goal. Contract đóng băng — xem
      mục **PHASE 0 — STABLE ROS INTERFACE** bên dưới.
- [x] `ticks_per_rev` calibrated: L/R **2478.1** (10 rev @ 15%, 2026-09-07).
- [x] `wheel_radius` calibrated: L **0.03296** / R **0.03263** m (2.00 m + ticks, 2026-09-07).
- [x] `wheel_separation` calibrated (effective): **0.4535 m** = mean(0.4274, 0.4795), operator 2026-09-07.

## IN PROGRESS / NEXT CHECKPOINT

- [ ] Product features after Phase 3 (Workflow / DB / MapNode / multi-robot) — **chưa mở**.
- [x] Phase 2 Semantic Map Editor — **PASS**.
- [x] Phase 3 Semantic Navigation Integration — xem **PHASE Demo 3** (stack/filter smoke PASS; motion `/plan` DoD cần 2D Pose Estimate).
- [ ] Outdoor Nav2 mid-path stall fix (2026-09-12): params + resilient BT + filters
      off outdoor — **đã sync+colcon trên Pi**; hotfix: **không** dùng empty
      `filters` list (Jazzy abort planner/controller) — omit key thay vì `[]`.
      Lifecycle ACTIVE sau restart; chờ smoke 2–3 goal (chưa claim motion PASS).
- [ ] Phase 7.5 mobile web teleop commissioning (wheels lifted first).
- [ ] Reflash ESP32 with updated `warotrans_low_level.ino` commissioning firmware.
- [ ] Phase 8 SLAM quality gate (map đã dùng được cho Nav2; quality gate riêng nếu cần).

## NOT YET VERIFIED

- [ ] ESP32 serial device `/dev/warotrans` ổn định lâu dài (bring-up OK; endurance chưa).
- [ ] Encoder signal/count runtime đo điện đầy đủ trên board.
- [ ] Tick encoder đúng dấu (đã chạy odom/nav; chưa bài sign-only riêng).
- [ ] Phase 7.5 teleop deadman + ESP32 watchdog verified with wheels lifted.
- [ ] SLAM dựng map đạt quality gate formal.
- [ ] Foxglove trên laptop xem được `/map` + `/scan` với fixed frame `map`
      (operator đã map outdoor bằng Foxglove; formal checklist còn mở).
- [x] **WSL RViz Nav topic CONNECT_OK** (2026-09-12 smoke): map
      `~/maps/warotrans_outdoor.yaml`; Pi `warotrans-nav-start-zenoh.sh` +
      WSL `~/start-rviz-nav-wsl.sh` (Zenoh TCP `:7447`) thấy `/map` `/scan`
      `/tf` `/amcl_pose`. **Chưa** claim outdoor Nav2 motion PASS. Stack đã
      `warotrans-stop-all` sau smoke.
- [ ] Phase 0.5 ultrasonic low-obstacle (RangeSensorLayer + Collision Monitor) —
      **chỉ FRONT + REAR** vào Nav2; LEFT/RIGHT không dùng. Node GPIO sẵn;
      mount FRONT/REAR **CHƯA XÁC NHẬN**; CM overlay sẵn nhưng tắt. Runbook:
      `docs/nav2-rviz-operator.md`, enable: `docs/ultrasonic-front-rear-nav2.md`.
- [ ] BNO055 / IMU integration.
- [ ] EKF localization fusion.
- [ ] Closed-loop motor PID.
- [ ] Camera model/interface được xác nhận và stream được.
- [ ] Fleet bridge end-to-end.
- [ ] Multi-robot.

## KNOWN UNKNOWNS

- Battery pack exact cell/BMS/fuse configuration: **không dùng DevKit này làm source of truth**.
- Encoder wiring hiện tại và VCC 3.3V: operator đã xác nhận; measurement record
  điện và runtime signal vẫn chưa có trong `docs/calibration.md`.
- Physical calibration constants: xem `docs/calibration.md`; các ô trống là chưa đo.
- Camera pose trên robot: chưa đo (xem mục camera bên dưới cho phần đã biết).

## CAMERA — phân biệt đã biết và chưa xác minh

Đánh dấu "chưa xác nhận" toàn bộ camera là quá thận trọng và làm mất thông tin
đã có. Tách rõ hai loại:

**Đã biết (từ quyết định mua, chưa verify trên robot):**

- Camera dự kiến: Raspberry Pi Camera Module 3, bản có auto focus, ống kính thường
  (không phải bản wide).
- Interface: CSI.

**CHƯA XÁC NHẬN bằng measurement:**

- Cáp đi kèm có đúng loại cắm được vào Pi 5 hay không.
- Camera có được kernel/driver trên Ubuntu 24.04 nhận hay không.
- Pose camera trên robot.

## KNOWN RISKS — camera (kiểm tra sớm, đừng để tới Phase 13)

Hai rủi ro dưới đây có **lead time** hoặc **thời gian build dài**. Phát hiện muộn
sẽ chặn cả phase, nên xác minh ngay khi mở hộp chứ không đợi tới lúc làm camera.

**R-CAM-1 — cáp CSI có thể không cắm được vào Pi 5.**
Đầu CSI trên Pi 5 nhỏ hơn đầu trên các đời Pi trước. Cáp đi kèm module camera
thường là loại cho đời cũ.
*Kiểm tra ngay:* mở hộp, đếm số chân trên đầu cáp và so với cổng CAM/DISP trên Pi 5.
Nếu không khớp → đặt mua cáp chuyển đúng loại, và chọn chiều dài đủ cho đường đi
từ Pi lên vị trí gắn camera cộng bán kính uốn.
*Ghi kết quả vào:* `docs/calibration.md` mục 5.

**R-CAM-2 — driver camera trên Ubuntu có thể không nhận cảm biến.**
Không phải mọi cảm biến camera đều được hỗ trợ bởi bản libcamera phổ thông cài qua
apt. Triệu chứng điển hình là thông báo không tìm thấy camera, rất dễ bị hiểu nhầm
thành hỏng phần cứng hoặc hỏng cáp.
*Kiểm tra ngay:* sau khi có cáp đúng, thử liệt kê camera ở tầng thấp nhất trước khi
đụng tới bất kỳ package ROS nào. Chỉ khi tầng đó thấy cảm biến mới đi tiếp lên ROS.
*Đường lui:* chuẩn bị một thẻ microSD thứ hai cài Raspberry Pi OS. Trên đó camera
chạy với công cụ có sẵn, dùng để phân biệt "lỗi driver/OS" với "lỗi phần cứng"
trong vài phút thay vì vài giờ. Thẻ này đồng thời là backup nếu thẻ chính hỏng
filesystem.

Cả hai rủi ro chỉ được gỡ khỏi mục này khi có evidence thật, không phải khi
"chắc là ổn".

## PHASE 8 — SLAM mapping status

**2026-09-07 evening:** Odom calibrated từ đo thật @ 15%:
`ticks_per_rev` L/R **2478.1**, `wheel_radius` L **0.03296** / R **0.03263**,
`wheel_separation` **0.4535**. Stack mapping full đã build + chạy trên Pi.

Runtime đang chạy (manual, không systemd):

```text
ros2 launch warotrans_bringup mapping.launch.py
# entry: /home/waro/bin/warotrans-mapping-start.sh
```

Thành phần: description + hardware + teleop `:8080` + LiDAR `/scan` +
`async_slam_toolbox_node` (lifecycle autostart) + foxglove `:8765`.

`warotrans-demo.service` vẫn **disabled** (không autostart demo).

Lưu map:

```bash
bash ~/tools/save_map.sh ~/maps/warotrans
```

Foxglove: `ws://<Pi-IP>:8765`, Fixed frame = **`map`**, bật `/map` + `/scan`.

DoD Phase 8 (quality gate) vẫn cần evidence sau khi lái quét đủ phòng — xem
`warotrans_slam/README.md`. Build/launch PASS ≠ Phase 8 PASS.

Blocker còn lại (không chặn chạy mapping):

- Straight-run có yaw ~29° — nên xác nhận lại bán kính bằng bài thẳng sạch hơn nếu map còn scale lệch.
- Firmware ESP32: reflash nếu muốn MCU dùng `wheel_separation` 0.4535 (odom Pi đã dùng 0.4535).

## PHASE 0 — STABLE ROS INTERFACE (FROZEN 2026-09-11)

**Result: PASS on real robot** (operator confirmed RViz path).  
**Freeze rule:** không đổi hành vi navigation đã verify trừ khi feature sau chứng minh lỗi tích hợp thật.

| Contract | Stable value |
|---|---|
| map frame | `map` |
| odom frame | `odom` |
| base frame (Nav2 / AMCL) | `base_footprint` |
| laser frame | `laser` |
| scan topic | `/scan` (`sensor_msgs/msg/LaserScan`) |
| odom topic | `/odom` (`nav_msgs/msg/Odometry`, child `base_footprint`) |
| NavigateToPose action | `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) |
| global path topic | `/plan` (`nav_msgs/msg/Path`, from `planner_server`) |
| cmd_vel topic | `/cmd_vel` (`geometry_msgs/msg/Twist`, **not** TwistStamped; **not** `/cmd_vel_nav`) |
| cmd_vel consumer | `esp32_bridge_node` (sole subscriber in nav mode) |
| saved map path | `$HOME/maps/warotrans.yaml` (+ `warotrans.pgm`) |
| navigation launch | `bash ~/tools/warotrans-nav-start.sh ~/maps/warotrans.yaml` **or** `ros2 launch warotrans_bringup navigation.launch.py map:=$HOME/maps/warotrans.yaml` |
| RViz config | `warotrans_navigation/rviz/nav2_warotrans.rviz` |
| TF ownership (nav) | `map→odom` AMCL; `odom→base_footprint` wheel_odom; static URDF via `robot_state_publisher` |

**Git checkpoint suggestion:**

```bash
git add -A   # review first; exclude secrets
git commit -m "checkpoint: Phase 0 Nav2 RViz real-robot PASS"
git tag -a nav2-rviz-real-robot-pass -m "Phase 0: RViz NavigateToPose PASS on real robot"
```

Tag name: **`nav2-rviz-real-robot-pass`** (chưa tạo trong session này — chạy lệnh trên khi bạn muốn checkpoint).

## PHASE 9 — Nav2 Navigation status

**Last runtime review:** 2026-09-11 — **Phase 0 PASS** (real robot + RViz operator confirm).

| Component | File | Status |
|---|---|---|
| Nav2 packages | `ros-jazzy-navigation2` + `ros-jazzy-nav2-bringup` | Installed on Pi |
| AMCL config | `warotrans_localization/config/amcl.yaml` | PASS (`set_initial_pose` PROVISIONAL boot; RViz overwrite) |
| Nav2 params | `warotrans_navigation/config/nav2_params.yaml` | Updated 2026-09-12 (TF/progress/RPP/inflation; filters default off) |
| Nav outdoor overlay | `config/nav2_params_outdoor.yaml` | Filters [] |
| Nav filters-on overlay | `config/nav2_params_filters_on.yaml` | Phase 3 `enable_costmap_filters:=true` |
| Resilient BT | `behavior_trees/navigate_to_pose_w_replanning_resilient.xml` | FollowPath fail → clear/wait/replan |
| Navigation launch | `warotrans_navigation/launch/navigation.launch.py` | `enable_costmap_filters` (default false) |
| Localization launch | `warotrans_localization/launch/localization.launch.py` | PASS (`amcl_params_file`) |
| Bringup compose | `warotrans_bringup/launch/navigation.launch.py` | PASS |
| Mapping RViz | `warotrans_slam/rviz/mapping.rviz` | Ready |
| Nav RViz | `warotrans_navigation/rviz/nav2_warotrans.rviz` | PASS |
| Diagnostics | `tools/check_nav2.sh`, `tools/navigation_doctor.sh` | Ready |
| Start helper | `tools/warotrans-nav-start.sh` + `tools/reset_rplidar_usb.sh` | Ready |

### Commands

```bash
# Mapping (Pi) — không chạy cùng navigation
ros2 launch warotrans_bringup mapping.launch.py

# Save map (Pi)
bash ~/warotrans/tools/save_map.sh ~/maps/warotrans

# Navigation (Pi) — outdoor map + WSL RViz (Zenoh)
bash ~/tools/warotrans-stop-all.sh
SUDO_PASS=12 SKIP_LIDAR_RESET=1 bash ~/tools/warotrans-nav-start-zenoh.sh ~/maps/warotrans_outdoor.yaml

# Diagnostics (Pi)
bash ~/tools/check_nav2.sh
bash ~/tools/navigation_doctor.sh

# WSL RViz (bash)
bash ~/start-rviz-nav-wsl.sh
# path: /mnt/e/.../warotrans_navigation/rviz/nav2_warotrans.rviz
# 1) 2D Pose Estimate  2) 2D Goal Pose
# Sau test: bash ~/tools/warotrans-stop-all.sh
```
### Runtime evidence 2026-09-11

- Launch arg collision fixed (`nav2_params_file` / `amcl_params_file`).
- Controller: **RegulatedPurePursuit** @ 10 Hz → `/cmd_vel` Twist.
- Lifecycle ACTIVE: map_server, amcl, controller, planner, behavior, bt.
- CLI + operator RViz: NavigateToPose **SUCCEEDED** on real robot.
- `esp32_bridge_node` is `/cmd_vel` subscriber in nav mode.

### Acceptance matrix (1–28) — Phase 0 closed

| # | Item | Result |
|---|---|---|
| 1–4 | `/scan`, `/odom`, TF odom/base/laser | PASS |
| 5–6 | SLAM realtime / RViz SLAM | NOT TESTED this closeout (saved-map nav path used) |
| 7–17 | save/load map, AMCL, Nav2 servers, costmaps, action | PASS |
| 18–26 | pose estimate, goal, `/plan`, `/cmd_vel`, motion, SUCCEEDED | PASS (operator RViz + prior CLI) |
| 27–28 | cancel / cmd_vel zero | PASS (operator Phase 0 confirm) |

**Known issues (non-blocking freeze):**

- Prefer `warotrans-nav-start.sh` if RPLIDAR USB hangs.
- AMCL boot pose provisional — always 2D Pose Estimate before warehouse goals.
- Velocity limits in calibration.yaml still PROVISIONAL numbers.

## PHASE Demo 1 — Web Endpoint Navigate (2026-09-11) — PASS (robot)

Code nằm ở **`demo/`** (tách khỏi `ros2_ws`). **Không** Lane/Zone/Workflow/DB.

| Component | Path | Status |
|---|---|---|
| FastAPI + rclpy gateway | `demo/backend/` | PASS |
| Vite React TS UI | `demo/frontend/` | PASS (build) |
| MapCoordinateTransformer | TS + Python + origin_yaw | Unit tests PASS (pytest 7, vitest 6) |
| Endpoint CRUD (RAM) | `/api/endpoints` | PASS |
| TF pose `map→base_footprint` | WS `robot` | PASS |
| Global path `/plan` (+ `/received_global_plan`) | WS `path` | PASS |
| NavigateToPose | `POST /api/navigate/{id}` | PASS → SUCCEEDED |

**Chạy:**

```bash
# Pi — Nav2 (Phase 0)
bash ~/tools/warotrans-nav-start.sh ~/maps/warotrans.yaml
# RViz: 2D Pose Estimate

# Pi — demo
bash ~/warotrans/tools/warotrans-demo-phase1-start.sh
# Browser: http://warotrans.local:8000/
```

**Acceptance (robot thật — evidence 2026-09-11):**

| # | Check | Result | Evidence |
|---|---|---|---|
| 1 | Browser map matches saved map | PASS | meta 60×62 @ 0.05, origin (−0.409, −2.357), PNG 200 |
| 2 | Robot position Web ≈ map TF | PASS | `/api/robot` valid from TF `map→base_footprint` |
| 3 | Robot yaw Web | PASS | yaw streamed with pose |
| 4 | Nav2 path Web | PASS | `path_n` 15→11 during nav from `/plan` |
| 5 | Add Endpoint from API/UI | PASS | POST `/api/endpoints` SHORT/PARK_WEB |
| 6 | Click/API NAVIGATE | PASS | status NAVIGATING |
| 7 | NavigateToPose accepted | PASS | goal accepted by bt_navigator |
| 8 | Nav2 path appears | PASS | WS/API path poses > 0 |
| 9 | Robot moves | PASS | dist > 0.25 m |
| 10 | Web robot updates realtime | PASS | pose changed while NAVIGATING |
| 11 | Nav2 SUCCEEDED | PASS | `status=SUCCEEDED` message Arrived |

**Unit tests:** pytest map_coords **7/7**, vitest **6/6**.

**Remaining (non-blocking):** operator visual side-by-side RViz vs browser for yaw/path cosmetics; cancel path clear timing.

**Freeze:** không đổi Nav2 Phase 0 config. Phase 2 chỉ thêm editor semantic (Lane/Zone) — **không** ràng buộc Nav2.

## PHASE Demo 2 — Semantic Facility Map Editor (2026-09-11) — PASS (API + UI deploy)

OTTO-inspired editor trên cùng `demo/`. Session RAM; Export/Import JSON map frame.
**Không** MapNode / RoutePlan / DB / multi-robot. Lane/Zone **chưa** nối Nav2.

| Component | Status |
|---|---|
| Lane CRUD (centerline + width) | PASS `/api/lanes` |
| TrafficZone CRUD (polygon) | PASS `/api/zones` |
| MapVersion export/import | PASS `GET/PUT /api/semantic-map` |
| Toolbar Select / Endpoint / Lane / Zone / Pan | UI deployed `phase:2` |
| Overlay z-order + Phase 1 robot/path/nav | robot TF valid; navigate API unchanged |

**Evidence (Pi 2026-09-11):**

- `GET /api/health` → `phase:2`, `map_ready:true`
- Create EP + 2 lanes + KEEP_OUT zone → export `version=2` → re-import → delete lane
- `/api/robot` `valid:true` (TF map→base_footprint) simultaneous with semantic data
- Unit: pytest **12**, vitest **7**, frontend build PASS

**Operator UI checklist (browser `http://warotrans.local:8000/`):** visual corridor/zone overlay + Navigate Phase 1 còn SUCCEEDED nếu cần xác nhận tay.

**STOP** — không mở Workflow / multi-robot.

## PHASE Demo 3 — Semantic Navigation Integration (2026-09-11) — stack smoke PASS / motion DoD operator

| Feature | Implementation |
|---|---|
| Lane → NavigateThroughPoses | `POST /api/navigate/lanes` + centerline sampling |
| Path display | vẫn `/plan` (Nav2) |
| KEEP_OUT | `KeepoutFilter` + mask LoadMap |
| SPEED_LIMIT | `SpeedFilter` type=2 absolute → `/speed_limit` |
| JUNCTION | semantic gap allow; no MapNode |
| SINGLE_ROBOT | config-only |

**Unit:** pytest **16**, vitest **7**, frontend build PASS.

**Live exec (Pi `warotrans`, 2026-09-11):**

- Nav2 + filter nodes UP: `keepout_filter_mask_server`, `speed_filter_mask_server`, `keepout_costmap_filter_info_server`, `speed_costmap_filter_info_server`, `bt_navigator`, `controller_server`.
- Costmap filters loaded: global `['keepout_filter','speed_filter']`, local `['keepout_filter']`.
- Demo `phase:3`, `map_ready:true`.
- `POST /api/filters/sync`: `keepout_cells`/`speed_cells` > 0, `load_keepout`/`load_speed` = `result=0`.
- KeepoutFilter/SpeedFilter log: mask received on `/keepout_filter_mask` + `/speed_filter_mask`; SpeedFilter absolute `base=0 + mask*0.01`.
- `POST /api/navigate/lanes` accepted (`mode=lanes`, e.g. `Lane nav (4 poses): P3L`).
- `/plan` path-length DoD **chưa ổn định** trên lần chạy này: planner `Failed to create plan` / behavior `Collision Ahead` → cần **2D Pose Estimate** + clear costmap tay trên RViz (không phải lỗi API Phase 3).

**Robot acceptance còn lại (operator UI `http://warotrans.local:8000/`):**

1. RViz 2D Pose Estimate → Navigate Lanes → `/plan` visible, robot moves.
2. KEEP_OUT trên free space → Sync Filters → plan detours.
3. SPEED_LIMIT → `/speed_limit` / slower `/cmd_vel` trong zone.

**STOP** — không mở Workflow / DB / MapNode / multi-robot.

## KNOWN ISSUES

- Chưa ghi nhận issue runtime mới sau mốc SSH/network.

## Cách cập nhật

Khi một checkpoint PASS, ghi thêm evidence ngắn, ví dụ:

```text
- [x] `/scan` 7.1–7.4 Hz trong 60 s, 2026-09-01.
```

Không tick chỉ vì code build thành công.
