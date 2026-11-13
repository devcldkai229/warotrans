# 07 — Gazebo trên MAP-ONE-V006 và hình học robot đo thật (2026-11-11)

## 1. Vì sao đổi

| Thay đổi | Nguồn |
|---|---|
| **Map.** Robot thật hiện chạy trên **MAP-ONE-V006**. Map này dựng từ bản quét `warotrans_lab_111026`: 70×102 px @ 0.05 m, origin (-1.424, -1.658), khoảng 3.5×5.1 m. Đây là bản quét khác với bản `remap_20261104_v3` mà world cũ dùng. | Backend: snapshot `warotrans-system/samples/maps/MAP-ONE-V006`. Robot: bundle `warotrans_navigation/maps/map_one_v006`. |
| **Hình học robot.** Đo bằng thước ngày 2026-11-11 (`docs/LAB_CALIBRATION_2026-11-11.md` Phần 3). | `geometry.yaml`: lidar ở x = 0.10 m; `base_footprint`, tức tâm quay, nằm 0.05 m sau tâm thân xe. Footprint Nav2 lệch về trước theo. |

V005 và V006 dùng chung PGM, lane, zone và endpoint. V006 chỉ khác là hạ giới hạn tốc độ của `ZONE-SPEED-ZONE-BOUND` từ 0.6 xuống **0.06 m/s**.

## 2. Robot trong Gazebo

URDF sim (`warotrans_simulation/urdf/warotrans_sim.urdf.xacro`) vẫn include URDF thật, nên lidar, IMU và `base_footprint → base_link` tự lấy theo `geometry.yaml`. Những phần chỉ sim mới có thì được sửa lại cho khớp robot thật:

| | Trước | Bây giờ (giống robot thật) |
|---|---|---|
| Trục bánh trước (so với `base_link` = tâm thân xe) | +0.073 | **+0.10**, ngay dưới lidar |
| Trục bánh sau | −0.073 | **−0.084** (wheelbase 0.184) |
| Khoảng cách tâm hai bánh trái–phải | ≈ 0.30 (bánh nằm ngoài tấm đế) | **0.214**, bánh nằm dưới tấm đế |
| Trọng tâm thân xe | tâm thân xe | **−0.02**, dồn về trục sau |
| Tâm quay khi quay tại chỗ | không ổn định: trọng tâm nằm đúng điểm giữa hai trục (tâm thân xe, 5 cm trước `base_footprint`) nên tâm quay nhảy giữa hai trục | **3.4 cm sau** `base_footprint`, ổn định |

Số liệu mới nằm trong `warotrans_description/config/calibration.yaml` (`chassis.wheelbase`, `track`, `front_axle_x`). Hiện chỉ URDF sim đọc các khoá này.

**Vì sao chọn trọng tâm −0.02.** Đây là kết quả đo bằng bài test quay tại chỗ trong Gazebo headless: quay 90–160°, rồi tính điểm đứng yên.

- Với ma sát Coulomb, xe skid-steer luôn quay quanh **trục chịu tải nhiều hơn**, nên tâm quay chỉ nhận một trong ba vị trí chứ không trượt liên tục:

  | Vị trí trọng tâm | Tâm quay |
  |---|---|
  | Sau điểm giữa hai trục (+0.008) | trục sau |
  | Đúng điểm giữa | điểm giữa |
  | Trước điểm giữa | trục trước |

- Robot thật dồn tải lên trục sau và quay quanh điểm khoảng 2 cm sau `base_footprint`. Lab đo được điểm đứng yên ở (−0.020, +0.017) và (−0.015, +0.018).
- Trục sau nằm 3.4 cm sau `base_footprint`, nên trọng tâm −0.02 cho sai số khoảng 1.4 cm và cách xa ngưỡng nhảy (+0.008).
- Đo lại trong sim:

  | Lệnh quay | Tâm quay (trong frame `base_footprint`) | `base_footprint` dịch chuyển |
  |---|---|---|
  | +0.6 rad/s | x = −0.034 | 0.067 m |
  | −0.6 rad/s | x = −0.034 | 0.066 m |
  | +0.3 rad/s | x = −0.034 | 0.063 m |

  Thử nghiệm quay ≈ 130–160°.

Muốn thử giá trị khác, truyền tham số khi chạy xacro: `sim_com_x:=<m>`.

**Footprint Nav2** trong sim lấy từ `warotrans_navigation/config/nav2_params.yaml`: `[[0.21, 0.13], [0.21, -0.13], [-0.11, -0.13], [-0.11, 0.13]]` quanh `base_footprint`. Overlay sim không ghi đè giá trị này, nên luôn khớp robot thật.

**Docking vào ô hẹp** (`warotrans_fleet/docking.py`, commit 1a28752). `sim_bringup.launch.py` truyền map sim vào gateway qua `docking_map_yaml`, giống cách `fleet.launch.py` làm trên robot thật. Nhờ vậy gateway trong sim cũng vào ô đỗ thẳng rồi quay trên trục ô, thay vì quay tại endpoint.

## 3. World MAP-ONE-V006

`config/warehouse_layout.yaml` là layout đang dùng. Layout cũ (V004) được giữ ở `config/layouts/remap_20261104_v3_map_one_v004.yaml`.

**Dựng lại world không cần backend:**

```bash
cd ~/warotrans_ws/src/warotrans_simulation
python3 scripts/sync_web_map.py --snapshot /mnt/d/.../warotrans-system/samples/maps/MAP-ONE-V006
```

`--snapshot` là tuỳ chọn mới. Nó đọc `snapshot.json`, `map.pgm` và `map.yaml` do backend export, ra kết quả giống lệnh tải qua API (`--user/--password`).

**Kết quả:**
- `maps/real/warotrans_lab_111026.{pgm,yaml}`: giống từng byte với `warotrans_navigation/maps/map_one_v006/map.pgm`.
- `maps/real/map_one_v006_overlay.json`: 11 endpoint, 7 lane, 8 zone, được vẽ lên sàn Gazebo.
- `worlds/warehouse.sdf`: 154 khối tường dựng từ các ô occupied, cao 0.8 m.
- `maps/sim_warehouse.{pgm,yaml}`: map Nav2 của sim, **giống từng byte** với map của robot thật.
- `config/sim_endpoints.json`.

**Mask ngữ nghĩa** cho chế độ `nav2_semantic` được chép nguyên từ bundle robot `map_one_v006` vào `maps/`: `keepout*`, `speed*`, `lanes.json`, `no_stop.json`, `single_occupancy.json`, `semantic_manifest.json`.

**Bố trí:**
- Robot spawn ở **EP-PARKING-A** (−0.541, 2.468), quay về hướng tây.
- Không trang trí kệ: bản quét không cho biết khối nào là kệ, và mọi ô occupied đều là khối đặc nên lidar sim thấy đúng như map.

## 4. Chạy với backend

**Bước 1.** Nhập map vào DB local (repo `warotrans-system`). Lệnh này làm 3 việc:
- V006 thành PUBLISHED, V004 chuyển ARCHIVED;
- RBT-SIM-01 được gắn với V006;
- `samples/simulation/sim-config.json` được cập nhật.

```powershell
powershell -ExecutionPolicy Bypass -File samples/maps/import-map-snapshot.ps1 -Snapshot samples/maps/MAP-ONE-V006
```

**Bước 2.** Chạy backend với traffic Semantic trỏ vào bundle V006:

```powershell
$env:ASPNETCORE_ENVIRONMENT="Simulation"
$env:Traffic__SemanticMapDirectory="D:/FPT_edu/CaptoneProject/sourceCode/robotSource/warotrans/ros2_ws/src/warotrans_navigation/maps/map_one_v006"
dotnet run --project backend/src/WaroTrans.Host
```

**Bước 3.** Chạy Gazebo (trong WSL):

```bash
ros2 launch warotrans_simulation sim_bringup.launch.py \
  gateway_params_file:=$(ros2 pkg prefix warotrans_simulation)/share/warotrans_simulation/config/robot_gateway_sim_semantic.yaml \
  map_version_code:=MAP-ONE-V006
```

## 5. Kiểm tra đã chạy

- `pytest warotrans_simulation/test`: 18 pass.
- `generate_warehouse.py --check`: 0, tức world khớp với layout.
- Test quay tại chỗ: xem bảng ở mục 2.
- **Sim headless kèm gateway** (2026-11-12): gửi `NAVIGATE_TO_POSE` qua MQTT giống backend, `stage1_open_grant:=true`, docking bật trên `sim_warehouse.yaml`.

  | Lệnh | Kết quả | Thời gian | Sai số vị trí | Sai số hướng |
  |---|---|---|---|---|
  | EP-PARKING-A → EP-PARKING-B | SUCCEEDED | 85 s, có 24 lần recovery ở chặng travel | 0.075 m | 0.013 rad |
  | → EP-INSPECTION | SUCCEEDED | 25 s | 0.056 m | 0.065 rad |
  | → EP-PARKING-A | SUCCEEDED | 114 s | 0.009 m | 0.272 rad |

- **Gửi goal thẳng cho Nav2, không qua gateway** (không có docking): EP-PARKING-B bị ABORTED vì "collision ahead" trong ô hẹp. Robot thật cũng gặp đúng lỗi này trước khi có docking (commit 1a28752). Vì vậy job phải đi qua gateway.

## 5b. E2E 3 workflow trên dữ liệu `infrastructure/snapshots/*20261112_164443` (2026-11-12)

**Môi trường:**
- Backend Development với `Traffic:Mode=Semantic`, bundle `map_one_v006`.
- Gazebo headless với gateway `robot_gateway_sim01_semantic_active.yaml`. `sim_bringup` tự nạp fingerprint, lanes và các zone từ `maps/`.
- Robot RBT-SIM-01 đăng ký trên V006.
- Storage location `SL-SHELF-A-01`, `SL-SHELF-B-01`, `SL-SHELF-B-02`, `SL-SHELF-C-01`, gắn với các endpoint kệ.
- Staff tạo phiếu nhập/xuất CONFIRMED, tạo TR, queue rồi start, và xác nhận các bước HUMAN qua API.

**Kết quả** (container `CTN-20261106-000004`, chạy liền một mạch, không can thiệp):

| Workflow | Lộ trình | Kết quả | Thời gian | Container sau |
|---|---|---|---|---|
| INBOUND | EP-INBOUND → SHELF-A-01 | TR và Job COMPLETED | 111 s | STORED @ SL-SHELF-A-01 |
| INTERNAL_RELOCATION | SHELF-A-01 → SHELF-B-02 | COMPLETED | 64 s | STORED @ SL-SHELF-B-02 |
| OUTBOUND_RETRIEVAL | SHELF-B-02 → EP-OUTBOUND | COMPLETED | 71 s | ISSUED |

Sổ `container_movements` của container ghi đủ 6 bước, theo đúng thứ tự:
1. PICKED_UP
2. DELIVERED (A-01)
3. PICKED_UP
4. DELIVERED (B-02)
5. PICKED_UP
6. ISSUED

**Lỗi đã tìm và sửa trong lúc test:**
- `nav2_params_sim.yaml` đặt `max_allowed_time_to_collision_up_to_carrot: 3.0`. Ở 0.20 m/s, footprint bị chiếu ra 0.6 m phía trước, vượt qua cuối đường exit 0.25 m khi docking ra khỏi SHELF-B-02 và đâm vào tường. Hệ quả: "collision ahead", rồi NAVIGATION_BLOCKED. Đã đặt về **1.0 s, giống robot thật**.

**Dữ liệu seed cần xử lý:**
- RBT-001 offline nhưng còn job `JOB-20261111-000001` ở trạng thái RECOVERY_REQUIRED. Vị trí cuối của nó là (1.57, −0.18), cạnh SHELF-B-01.
- Traffic Semantic vẫn giữ zone cho RBT-001, nên robot sim ở B-02 bị `QUARANTINED:SEED_CONFLICT` hoặc `Capacity blocker`.
- Trong Gazebo không có RBT-001, nên đã gọi `release-after-clearance` (có audit) để giải phóng.

## 6. Giới hạn

- **Quay theo chiều kim đồng hồ.** Robot thật khi quay CW còn lệch 0.11–0.13 m sang trái, do lực kéo hai bên không đều (lỗi cơ khí). Sim không mô phỏng lỗi này.
- **Tâm quay trong sim chỉ đúng tới khoảng 1.4 cm.** Với mô hình ma sát Coulomb, tâm quay chỉ có thể nằm ở một trục bánh hoặc ở điểm giữa hai trục.
- **Khoang đỗ bên trái (Parking A/B) rất chật.** Robot vẫn tới được, nhưng chậm: phải recovery nhiều lần, và về Parking A thì lệch hướng 0.27 rad. Rig robot thật trên V006 đạt sai số 0.02–0.05 m. Muốn sim nhanh hơn ở đoạn này cần tinh chỉnh Nav2 hoặc docking, chưa làm.
- **DiffDrive vẫn dùng `wheel.separation` hiệu dụng (0.4535 m)** để đổi lệnh `/cmd_vel` ra tốc độ bánh, giống `wheel_odom_node` trên robot thật.

## 8. Cập nhật 2026-11-13: tốc độ 0.40 m/s, zone SPEED 0.20 m/s, sai số đến đích 0.10 m

Commit `745571f` trên develop (theo yêu cầu vận hành sau buổi lab 13/11) đổi Nav2 của robot thật và xuất lại bộ semantic V006. Simulation được cập nhật theo:

| Hạng mục | Trước | Sau | File |
|---|---|---|---|
| Tốc độ chạy (FollowPath) | 0.20 m/s, lookahead cố định 0.28 m | 0.40 m/s, lookahead theo tốc độ (0.8 s, 0.15–0.45 m) | `warotrans_simulation/config/nav2_params_sim.yaml` |
| Sai số đến đích (goal checker) | 0.08 m / 0.40 rad | 0.10 m / 0.15 rad | như trên |
| Arrival refine, docking arrive/align của gateway | mặc định 0.05 m / 0.087 rad | 0.10 m / 0.15 rad | `config/robot_gateway_sim01_semantic_active.yaml` |
| Mặt nạ speed + manifest | fingerprint `cdc635729d…` | fingerprint `e0efbd2d…` (zone SPEED 0.20 m/s) | `warotrans_simulation/maps/speed.pgm`, `semantic_manifest.json`, chép từ `warotrans_navigation/maps/map_one_v006` |
| Overlay hiển thị | `maxSpeed` 0.06 | 0.2 | `maps/real/map_one_v006_overlay.json` |

Fingerprint của gateway sim (đọc từ `maps/semantic_manifest.json`) phải **trùng** với fingerprint mà backend nạp từ `Traffic:SemanticMapDirectory`. Mỗi khi bộ semantic V006 được xuất lại, phải chép lại `speed.pgm` và `semantic_manifest.json` (cùng các file khác nếu đổi), rồi khởi động lại **cả backend lẫn Gazebo**.

DB local được cập nhật theo `samples/maps/MAP-ONE-V006/snapshot.json` mới: `ZONE-SPEED-ZONE-BOUND.max_speed` 0.06 → 0.2, `position_tolerance` của 11 endpoint → 0.1.

**Lưu ý khi chạy backend cho Gazebo:**
- Dùng `Traffic__Mode=Semantic`. `dotnet run` trơn sẽ dùng chế độ Active (đi theo edge) và báo `UNREACHABLE` từ Parking A.
- Đặt `Mqtt__HeartbeatTimeoutSeconds=30` như `appsettings.Simulation.json`. Với mặc định 5 s của Development, gateway trong WSL có lúc bị khựng hơn 5 s khi đang lùi vào B-02, backend tưởng robot OFFLINE và huỷ lệnh (`OFFLINE_ABORT`).

**E2E trên Gazebo (GUI bật), container `CTN-20261106-000002`:**

| Workflow | TR | Thời gian | Kết quả |
|---|---|---|---|
| INBOUND → SL-SHELF-A-01 | REQ-20261113-000003 | 104 s | COMPLETED, STORED |
| INTERNAL_RELOCATION → SL-SHELF-B-02 | REQ-20261113-000004 | 12 s + 62 s | Lần đầu PAUSED do `OFFLINE_ABORT` (timeout 5 s); sau khi nâng lên 30 s và Resume thì COMPLETED, STORED |
| OUTBOUND_RETRIEVAL → EP-OUTBOUND | REQ-20261113-000005 | 98 s | COMPLETED, ISSUED (phiếu PX-20261113-000001) |

Không cần operator cứu tay ở B-02 (ngày 12/11 phải điều hướng tay để thoát): việc tự docking lại (`41c24ef`) cùng sai số đến đích 0.10 m đã giải quyết chỗ kẹt đó.

## 9. Sai số đến đích cho phép và vị trí endpoint (2026-11-13)

**Cách tính.** Endpoint là tư thế của `base_link`, tức tâm khung xe. Footprint so với `base_link` là x −0.16..0.16, y −0.13..0.13; tâm xoay `base_footprint` nằm sau 0.05 m. Với mỗi endpoint, dịch hoặc xoay footprint trên `map.pgm` của V006 cho tới khi chạm ô occupied/unknown hoặc lấn vào vùng NO_STOP (`no_stop.json`).

**Giới hạn dưới (độ chính xác robot đạt được):**
- AMCL ±0.02–0.03 m.
- Trôi khi dừng 0.01–0.02 m.
- Trượt khi xoay chỉnh hướng cuối: 2·0.034·sin(θ/2), tức 0.018 m khi θ = 30° và 0.048 m khi θ = 90°.

Cộng lại khoảng 0.08–0.10 m. Ở mức tolerance 0.10, sai số cuối thực đo là 0.049–0.092 m, đa số sát ngưỡng, nên robot hay phải chỉnh lại.

**Giới hạn trên (vẫn chấp nhận được):**
- Gần vùng NO_STOP nhất là EP-SHELF-B-01, còn 0.17 m.
- Nửa ô traffic là 0.125 m.

**Chọn: 0.12 m / 0.20 rad.** Áp vào:
- `nav2_params_sim.yaml` (general_goal_checker).
- `robot_gateway_sim01_semantic_active.yaml` (arrival_refine_*, docking_arrive_xy_m, docking_align_yaw_rad).
- `position_tolerance` của 11 endpoint V006 trong DB local.

**Dời endpoint.** EP-OUTBOUND và EP-SHELF-C-01 đặt quá sâu: khoảng trống phía sau footprint chỉ còn **0.06 m**, trong khi Nav2 cần footprint_padding 0.02 m cộng ô costmap 0.05 m. Vì vậy bước run_in luôn dừng cách đích khoảng 0.14 m, rồi bước align (chỉ chạy tiến) phải nhích tới lui trong 29.5 s. Đã dời cả hai từ x = −0.953 sang **x = −0.883** trong DB, `maps/real/map_one_v006_overlay.json` và `config/sim_endpoints.json`. Khoảng trống phía sau tăng lên 0.13 m.

| Endpoint | Tới tường (hướng hẹp nhất) | Tới NO_STOP |
|---|---|---|
| EP-QUEUE-POINT | **0.08 m** (phía sau) | > 0.6 m |
| EP-INSPECTION | 0.105 m | > 0.6 m |
| EP-CHARGE | 0.11 m (bên trái) | > 0.6 m |
| EP-PARKING-B | 0.12 m | 0.26 m |
| EP-OUTBOUND, EP-SHELF-C-01 (sau khi dời) | 0.13 m | > 0.6 m |
| EP-SHELF-A-01, EP-SHELF-B-02 | 0.14 m | 0.24 / 0.20 m |
| EP-PARKING-A | 0.155 m | 0.20 m |
| EP-SHELF-B-01 | 0.16 m | 0.17 m |
| EP-INBOUND | 0.255 m | > 0.6 m |

EP-QUEUE-POINT (0.08 m) và EP-INSPECTION (0.105 m) cũng sát tường, nhưng E2E chưa đi qua hai điểm này.

**E2E trên Gazebo (GUI), container `CTN-20261113-000001`:**

| Workflow | TR | Thời gian | So với tolerance 0.10 |
|---|---|---|---|
| INBOUND | REQ-20261113-000006 | 102 s | 104 s |
| INTERNAL_RELOCATION | REQ-20261113-000007 | 75 s | 62 s |
| OUTBOUND_RETRIEVAL | REQ-20261113-000008 | 126 s | 98 s |

- Bước align ở EP-OUTBOUND giảm từ 29.5 s xuống **6 s**, sai số cuối 0.033 m.
- OUTBOUND lâu hơn **không phải vì EP-OUTBOUND**. Robot kẹt 53 s với 18 recovery khi **rời EP-SHELF-B-02**: kế hoạch docking không có bước exit. Robot đứng lệch ngang 1.5 cm, ở (1.19, −0.10); tại đó `DockingGeometry.can_turn` (padding 0.03) cho là xoay được, nhưng costmap Nav2 (padding 0.02 + ô 0.05) không cho. Lần chạy ở mức 0.10, robot đứng ở chỗ planner cho là không xoay được, nên có bước exit 0.25 m.
- Không thể nâng `docking_padding` chung lên: ở 0.04 thì EP-CHARGE mất kế hoạch, ở 0.05 thì EP-SHELF-B-01 cũng mất.

**Hướng sửa:** chỉ riêng quyết định có exit hay không thì dùng biên rộng (`can_turn(start, strict=True)`, padding + 0.05); điểm xoay khi vào ô giữ nguyên. Cần thống nhất trước khi sửa, vì đây là code `warotrans_fleet/docking.py` dùng chung với robot thật.
- Sai số cuối sau khi đứng yên có lúc vượt ngưỡng một chút (0.135 m / 0.26 rad ở INBOUND, A-01): goal checker cho qua ngay khi lọt vào 0.12, sau đó robot trôi thêm khi dừng. Lần đến kệ kế tiếp sẽ docking lại.
