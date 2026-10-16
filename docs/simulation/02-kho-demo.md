# Kho mô phỏng 11 × 7.5 m

> Trạng thái: **XONG** (2026-10-15). Map backend: `MAP-WHSIM-V011` (9 endpoint + 5 zone).
> Kết quả: workflow `INBOUND_PUTAWAY` cất 3 thùng vừa nhận (IN1→A, IN1→B, IN2→C). Job `COMPLETED` 19/19 trong 131 s.

## 1. Layout

```
 y=7.5 +------------------------------------------------------------------+
       |                              [CHG]                                |  trạm sạc sát tường Bắc
  dock |  INBOUND                       ^                        OUTBOUND  | dock
  door |  < IN1          +--+     +--+     +--+                 OUT1 >    | door
       |                 |A |     |B |     |C |                           |  3 kệ đứng giữa kho (0.6 × 2.4 m, cao 2 m, 3 tầng)
       |                 |  |     |  |     |  |                           |  cách mọi tường >= 2.5 m
  dock |  < IN2       >  +--+  >  +--+  >  +--+                 OUT2 >    | dock
  door |                              v                                    | door
       |                            PARK                                   |  bãi đỗ, robot xuất phát ở đây
 y=0   +------------------------------------------------------------------+
      x=0                                                               x=11
```

Hàng đi từ Tây sang Đông: nhận ở cửa Tây (inbound), cất lên kệ, xuất ra cửa Đông (outbound).

| Endpoint (code) | Tên | Loại | x, y (m) | Hướng robot |
|---|---|---|---|---|
| IN1 / IN2 | Inbound 1 / 2 | INBOUND | 0.9, 5.5 / 0.9, 2.0 | quay về cửa Tây |
| A / B / C | Rack A / B / C | STORAGE | 3.2 / 4.875 / 6.625, 3.75 | quay về mặt Tây của kệ |
| OUT1 / OUT2 | Outbound 1 / 2 | OUTBOUND | 10.1, 5.5 / 10.1, 2.0 | quay về cửa Đông |
| CHG | Charger | CHARGING | 5.75, 6.75 | quay về trạm sạc |
| PARK | Parking | PARKING | 5.75, 0.9 | quay lên Bắc |

Lối giữa các kệ rộng 1.15 m, các lối khác ≥ 1.5 m. Robot 0.32 × 0.26 m quay tại chỗ được ở mọi điểm.

**Vạch sơn trên sàn = Zone trong backend.** Mỗi vùng sơn có khối `zone:` trong layout. `seed-sim.ps1` tạo Zone cùng hình chữ nhật đó trên map, nên web thấy đúng vùng Gazebo đang sơn:

| Zone (backend) | Loại | Sức chứa | Toạ độ (x0, y0 → x1, y1) | Màu sàn Gazebo |
|---|---|---|---|---|
| Inbound Area | OPERATIONAL_AREA | 2 | 0.15, 0.6 → 2.3, 6.9 | nền xanh lá, viền vàng |
| Outbound Area | OPERATIONAL_AREA | 2 | 8.7, 0.6 → 10.85, 6.9 | nền cam nhạt, viền vàng |
| Rack Area | NARROW_AREA (max 0.3 m/s) | 1 | 2.75, 2.2 → 8.05, 5.3 | viền vàng |
| Charging Area | OPERATIONAL_AREA | 1 | 5.2, 6.25 → 6.3, 7.45 | nền vàng nhạt |
| Parking Area | OPERATIONAL_AREA | 1 | 5.3, 0.35 → 6.2, 1.45 | viền xám |

Backend chưa có loại zone riêng cho inbound/outbound, nên dùng `OPERATIONAL_AREA` kèm tên.
Thêm hoặc sửa zone: sửa `floor_zones` trong layout, build, rồi chạy `seed-sim.ps1`. Seed thấy map đang publish thiếu zone nên tạo map version mới.

## 1b. Workflow inbound `INBOUND_PUTAWAY`

File: `samples/workflows/INBOUND_PUTAWAY.request.json` (backend repo). `run-job-sim.ps1` tự tạo và publish nếu DB chưa có.

| Task | Step | Loại / operation | Ý nghĩa |
|---|---|---|---|
| Receive at inbound dock | Check robot is ready / minimum battery / request is still active | CHECK | kiểm tra một lần cho cả job |
| | Check inbound dock is available | CHECK `ENDPOINT_AVAILABLE` | mỗi điểm nhận |
| | Move to inbound dock | MOVE `PICKUP` | robot chạy tới IN1 / IN2 |
| | Staff confirm received container loaded onto robot | HUMAN `PICKUP_CONFIRM` | nhân viên quét từng thùng, đặt lên robot |
| Put away on storage rack | Check storage rack is available | CHECK `ENDPOINT_AVAILABLE` | mỗi kệ đích |
| | Move to storage rack | MOVE `DROPOFF` | robot chạy tới kệ |
| | Staff confirm container stored on rack | HUMAN `DROPOFF_CONFIRM` | nhân viên cất thùng lên kệ |

Container demo: `CTN-IN-000001..3`, trạng thái `PACKED` (vừa nhận ở cửa). Request: IN1→A, IN1→B, IN2→C.
Planner gom lại: lấy 2 thùng ở IN1, lấy 1 thùng ở IN2, rồi giao lần lượt A, B, C. Tổng cộng 19 step.

Mỗi endpoint có một ô vuông sơn trên sàn, màu theo loại: kệ xanh dương, inbound xanh lá, outbound cam, sạc vàng, bãi đỗ xám.

## 2. Gazebo dựng kho thế nào (để vừa đẹp vừa khớp map)

`scripts/generate_warehouse.py` đọc `config/warehouse_layout.yaml` và sinh **world** cùng **map** từ cùng một nguồn:

| Thứ trong Gazebo | Có va chạm? | Có trong map Nav2? | Ghi chú |
|---|---|---|---|
| Tường cao 2.5 m | Có | Có | |
| Kệ: chân kệ đặc 0–0.45 m | Có | Có (đúng footprint 0.6 × 2.4 m) | Lidar quét ở ~0.2 m, nên thấy đúng hình chữ nhật như trên map |
| Kệ: cột xanh, thanh ngang cam, 3 mâm, thùng carton / bin xanh | Cột và mâm có, thùng không | Không (nằm trên tầm lidar) | Chỉ để nhìn giống kho thật |
| Trạm sạc: đế 0.5 × 0.25 m, tháp, đèn xanh | Đế có | Đế có | |
| Vạch sơn, ô điểm đậu | Không | Không | Dày 2 mm, thấp hơn lidar |
| Cửa cuốn ở cửa dock | Không | Không | Vẽ trên mặt trong của tường |

Nguyên tắc: **mọi thứ ở độ cao lidar đều nằm trong map**. Phần trang trí chỉ nằm trên cao hoặc sát sàn, nên AMCL vẫn khớp chính xác.

## 3. Thông số Nav2 riêng cho sim (`nav2_params_sim.yaml`)

| Thông số | Robot thật | Sim | Lý do |
|---|---|---|---|
| `inflation_radius` | 0.15 | 0.35 | Giữ đường đi ở giữa lối, tránh RPP báo "collision ahead" sát kệ |
| `cost_scaling_factor` | 2.0 | 4.0 | Vùng đệm giảm nhanh, robot vẫn vào được điểm đậu |
| `desired_linear_vel` | 0.10 | 0.35 | Motor ảo không giới hạn như motor thật |

## 4. Dữ liệu tiếng Anh

Toàn bộ dữ liệu demo đều bằng tiếng Anh:
- workflow `INTERNAL_RELOCATION`: tên, task, step, hướng dẫn;
- tên endpoint, kho ("Simulation Warehouse"), robot ("Gazebo Simulation Robot");
- danh mục và sản phẩm.

File nguồn là `samples/workflows/INTERNAL_RELOCATION.request.json` (backend repo).

## 5. Chạy

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/d/WaroTransSim/scripts/04-build-workspace.sh     # sau khi sửa layout
powershell -ExecutionPolicy Bypass -File samples\simulation\seed-sim.ps1           # backend repo
wsl -d Ubuntu-24.04 -- bash /mnt/d/WaroTransSim/scripts/run-sim.sh
powershell -ExecutionPolicy Bypass -File samples\simulation\run-job-sim.ps1        # demo inbound (mặc định)
powershell -ExecutionPolicy Bypass -File samples\simulation\run-job-sim.ps1 -Scenario relocation   # demo chuyển kệ / xuất
```
