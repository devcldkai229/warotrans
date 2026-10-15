# GĐ 1 — Robot ảo chạy Nav2 trong Gazebo, điều khiển qua MQTT

> Trạng thái: **XONG** (2026-10-15), nhánh `feature/gazebo-simulation`. Quyết định kiến trúc: [ADR 0006](../adr/0006-simulation-gazebo-harmonic.md).
> Kết quả: lệnh `NAVIGATE_TO_POSE` gửi qua MQTT → robot ảo tự đi tới kệ → `command_result SUCCEEDED`.

## 1. Ý tưởng chính: thay phần cứng, giữ nguyên mọi thứ còn lại

Trên robot thật, stack ROS 2 gồm 3 tầng:

| Tầng | Robot thật (Pi) | Robot ảo (sim) |
|---|---|---|
| **Phần cứng** — bánh xe, encoder, lidar | `esp32_bridge_node`, `wheel_odom_node`, driver RPLIDAR | **Gazebo**: 4 bánh skid-steer + plugin `DiffDrive`, `gpu_lidar` + `ros_gz_bridge` |
| **Định vị + dẫn đường** | `map_server` + `amcl` + Nav2 | **giống hệt** (cùng params, thêm `use_sim_time` + tốc độ demo) |
| **Cầu nối backend** | `robot_gateway_node` (MQTT) | **giống hệt** (chỉ đổi `robot_code: RBT-SIM-01`) |

Chỉ tầng dưới cùng bị thay. Tên topic (`/cmd_vel`, `/odom`, `/scan`, `/tf`) và frame
(`map`, `odom`, `base_footprint`, `base_link`, `laser`) giữ nguyên. Vì vậy Nav2 và gateway **không biết** chúng
đang chạy trong mô phỏng, và backend cũng không biết, vì backend chỉ thấy MQTT.

## 2. Kiến trúc toàn hệ thống khi chạy sim

```
 Backend .NET / CLI test                 Mosquitto (Docker, Windows :1883)
        │  publish  warotrans/v1/robots/RBT-SIM-01/command         ▲  ack / result / telemetry / heartbeat
        ▼                                                          │
 ┌────────────────────────────── WSL2 Ubuntu ─────────────────────────────────────────────────┐
 │  robot_gateway_node (warotrans_fleet)                                                       │
 │     ├─ MQTT command → action client /navigate_to_pose (Nav2)                                │
 │     ├─ đọc TF map→base_link → telemetry pose; amcl_pose → LOCALIZED                         │
 │     └─ kết quả action → command_result SUCCEEDED / FAILED / CANCELED                        │
 │                │ NavigateToPose (action)                                                    │
 │                ▼                                                                            │
 │  Nav2: bt_navigator → planner_server (NavFn, global costmap từ map) → controller_server (RPP) │
 │                │ /cmd_vel (Twist)                          ▲ /scan, /odom, TF                │
 │                ▼                                           │                                │
 │  ros_gz_bridge ◄───────────────────────────────────────────┘                                │
 │                │ gz topics                                                                  │
 │                ▼                                                                            │
 │  Gazebo Harmonic: world "sim_warehouse" + robot "warotrans"                                 │
 │     DiffDrive: /cmd_vel → quay 4 bánh → /odom + TF odom→base_footprint                       │
 │     gpu_lidar trên frame "laser" → /scan (360 tia, 10 Hz)                                   │
 │                                                                                             │
 │  map_server (maps/sim_warehouse.yaml) + AMCL (/scan + map → TF map→odom)                     │
 │  robot_state_publisher (URDF sim → TF base_footprint→base_link→laser, bánh xe)              │
 └─────────────────────────────────────────────────────────────────────────────────────────────┘
```

Cây TF hoàn chỉnh: `map ─(AMCL)→ odom ─(Gazebo DiffDrive)→ base_footprint ─(URDF)→ base_link → laser / bánh xe`.

## 3. Một lệnh MOVE đi qua hệ thống như thế nào (theo từng bước)

Ví dụ backend muốn robot tới **kệ D** `(10.25, 6.3, yaw 0)`:

1. **Backend → MQTT**: publish lên `warotrans/v1/robots/RBT-SIM-01/command`
   `{"type":"NAVIGATE_TO_POSE","commandId":"…","payload":{"frameId":"map","x":10.25,"y":6.3,"yaw":0}}`.
2. **Gateway nhận lệnh**: kiểm tra robot đang `LOCALIZED` và không bận, rồi gửi `command_ack {accepted:true}`.
   Nếu không hợp lệ thì gửi `accepted:false` kèm `reasonCode` (`NOT_LOCALIZED`, `BUSY`, …).
3. **Gateway → Nav2**: gửi goal của action `navigate_to_pose` (pose trong frame `map`).
4. **Nav2 lập đường**: planner dùng **map tĩnh** để tìm đường tránh tường, kệ, pallet. Controller (Regulated Pure
   Pursuit) bám đường và phát `/cmd_vel` 10 lần/giây.
5. **Gazebo chạy robot**: bridge đưa `/cmd_vel` vào Gazebo. Plugin DiffDrive quay 4 bánh (2 trái, 2 phải, skid-steer), vật lý mô phỏng
   va chạm và ma sát, robot di chuyển trong world.
6. **Phản hồi cảm biến**: Gazebo phát `/odom` (50 Hz) và `/scan` (10 Hz). AMCL so `/scan` với map để sửa
   sai số odom (TF `map→odom`). Costmap cục bộ dùng `/scan` để né vật cản mới.
7. **Telemetry**: gateway đọc TF `map→base_link` 5 lần/giây, rồi publish `telemetry`
   `{navigationStatus:"NAVIGATING", pose:{x,y,yaw}, currentCommandId:"…"}`. Backend sẽ đẩy pose này lên web qua SignalR (GĐ 4).
8. **Tới nơi**: Nav2 báo action `SUCCEEDED`, gateway publish `command_result {outcome:"SUCCEEDED"}`. Backend
   (đã có sẵn `RobotCommandLifecycleJobStepHandler`) sẽ đánh dấu JobStep `COMPLETED`.

CANCEL: backend gửi `type:"CANCEL"` → gateway hủy goal Nav2 → robot dừng → `command_result CANCELED`.

## 4. Một nguồn toạ độ duy nhất: `warehouse_layout.yaml`

```
config/warehouse_layout.yaml ──scripts/generate_warehouse.py──┬─► worlds/warehouse.sdf        (Gazebo nhìn thấy)
   (tường, kệ A–F, vật cản,                                    ├─► maps/sim_warehouse.pgm/.yaml (Nav2/AMCL dùng)
    điểm đậu trước kệ, spawn)                                  └─► config/sim_endpoints.json   (backend tạo Endpoint — GĐ 2)
```

Vì world, map và endpoint sinh ra từ **cùng một file**, toạ độ `(x, y)` trong Gazebo, trên map Nav2 và trong
`Endpoint` của backend luôn trùng nhau (frame `map` ≡ world). Map được tính từ hình học, nên không cần chạy
SLAM; map chính xác tuyệt đối với world.

Kho mô phỏng hiện tại rộng 11 × 7.5 m, gồm inbound, 3 kệ A–C, outbound, trạm sạc và bãi đỗ. Layout và toạ độ xem ở [02-kho-demo.md](02-kho-demo.md).

Muốn đổi kho thì sửa `warehouse_layout.yaml`, chạy `04-build-workspace.sh` (đã gọi generator), rồi chạy lại sim.

## 5. Các file đã thêm / sửa (robot repo)

| File | Vai trò |
|---|---|
| `docs/adr/0006-simulation-gazebo-harmonic.md` | Quyết định chọn Gazebo Harmonic + ros_gz |
| `warotrans_simulation/config/warehouse_layout.yaml` | Layout kho (nguồn duy nhất) |
| `warotrans_simulation/scripts/generate_warehouse.py` | Sinh world/map/endpoints. `--check` để phát hiện file sinh ra bị cũ |
| `warotrans_simulation/worlds/warehouse.sdf`, `maps/sim_warehouse.*`, `config/sim_endpoints.json` | **File sinh ra**, không sửa tay |
| `warotrans_simulation/urdf/warotrans_sim.urdf.xacro` | Include URDF thật, thêm 4 bánh skid-steer (số đo từ `calibration.yaml`), quán tính, lidar, DiffDrive (chỉ dùng cho sim) |
| `warotrans_simulation/config/ros_gz_bridge.yaml` | Nối topic Gazebo ↔ ROS |
| `warotrans_simulation/config/nav2_params_sim.yaml` | Overlay Nav2: tốc độ demo 0.4 m/s (robot thật vẫn 0.10) |
| `warotrans_simulation/config/robot_gateway_sim.yaml` | Gateway cho robot ảo `RBT-SIM-01`, `use_sim_time` |
| `warotrans_simulation/launch/simulation.launch.py` | Gazebo + spawn robot + bridge + robot_state_publisher |
| `warotrans_simulation/launch/sim_bringup.launch.py` | Toàn bộ: sim + map_server/AMCL + Nav2 + gateway (+ RViz) |
| `warotrans_navigation/launch/navigation.launch.py` | Thêm arg tùy chọn `extra_params_file` (mặc định rỗng, nên robot thật không đổi) |

**Không sửa**: URDF thật, `nav2_params.yaml`, `amcl.yaml`, code gateway `warotrans_fleet`.

## 6. Cách chạy

Điều kiện: Docker đang chạy Mosquitto (`cd warotrans-system/infrastructure; docker compose up -d`).

```powershell
# Toàn bộ robot ảo có cửa sổ Gazebo (thêm rviz:=true để xem đường đi Nav2)
wsl -d Ubuntu-24.04 -- bash /mnt/d/WaroTransSim/scripts/run-sim.sh
wsl -d Ubuntu-24.04 -- bash /mnt/d/WaroTransSim/scripts/run-sim.sh rviz:=true
```

Ở terminal khác, gửi lệnh như backend sẽ gửi:

```bash
# trong Ubuntu (wsl -d Ubuntu-24.04)
python3 ~/warotrans_ws/src/warotrans_fleet/scripts/fleet_command_cli.py --robot RBT-SIM-01 navigate --x 10.25 --y 6.3 --yaw 0
mosquitto_sub -h localhost -t 'warotrans/v1/robots/RBT-SIM-01/#' -v     # xem toàn bộ MQTT
```

Test tự động, không cần GUI:

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/d/WaroTransSim/scripts/smoke-sim.sh    # Nav2 trực tiếp (không MQTT)
wsl -d Ubuntu-24.04 -- bash /mnt/d/WaroTransSim/scripts/smoke-mqtt.sh   # qua MQTT, đúng đường backend
```

## 7. Kết quả kiểm chứng (2026-10-15, headless, robot 4 bánh skid-steer)

| Test | Kết quả |
|---|---|
| `/scan` / `/odom` | 10.0 Hz / ~46–50 Hz |
| TF `map→base_footprint` lúc khởi động | `(6.000, 0.800)`, yaw π/2, trùng spawn |
| Nav2 action PARK → kệ D | `SUCCEEDED` sau 22 s, tới `(10.23, 6.18)` (trong goal tolerance 0.15 m) |
| MQTT telemetry | `localizationStatus=LOCALIZED`, `navigationStatus=IDLE`, pose đúng spawn; heartbeat đều |
| MQTT `NAVIGATE_TO_POSE` → kệ E | `accepted=true` → `SUCCEEDED` sau 16 s (lần chạy đầu, robot 2 bánh; luồng MQTT không đổi) |
| MQTT `NAVIGATE_TO_POSE` → kệ A (phải quay đầu 180°) | `accepted=true` → `SUCCEEDED` sau 24 s |

Chưa kiểm bằng mắt cửa sổ Gazebo GUI. Bạn chạy `run-sim.sh` để xem.

## 8. Giới hạn đã biết

- Robot ảo là **4 bánh skid-steer** giống khung thật (wheelbase 0.146 m, bánh R≈0.0328 m, `wheel_separation` hiệu dụng 0.4535 m lấy từ `calibration.yaml`). Độ trượt khi quay phụ thuộc ma sát Gazebo (`mu2 = 0.4`), nên **không** dùng sim để hiệu chỉnh bánh/PID.
- Pin: telemetry `batteryPercent = null` (robot thật cũng chưa có cảm biến pin). Bước `BATTERY_MIN` ở GĐ 3 cần xử lý null.
- Mỗi lần chạy, robot luôn spawn ở PARK. Muốn đổi thì sửa `spawn` trong layout.

## 9. GĐ 2 (backend nhận robot ảo) — XONG, xem backend repo `docs/simulation/02-backend-nhan-robot-ao.md`

1. Import `maps/sim_warehouse.pgm/.yaml` thành MapVersion trong backend, rồi publish.
2. Tạo Endpoint A–F, PARK theo `config/sim_endpoints.json`.
3. Đăng ký robot `RBT-SIM-01` trên map đó, chạy gateway với `map_version_code:=<code của map>`.
4. DoD: backend gọi `POST /api/fleet/robots/{id}/commands/navigate`, robot trong Gazebo chạy, backend lưu pose/online.
