# Demo map v6 trên Gazebo (sàn thật 3.50 × 4.55 m)

> Nhánh `feature/simulation-demo-map-v6` · 2026-10-20. Thay cho kho 11 × 7.5 m của [02-kho-demo.md](02-kho-demo.md).
> Layout kho cũ vẫn giữ ở `config/layouts/demo_warehouse_11x7_5.yaml`. Muốn quay lại thì chạy
> `generate_warehouse.py --layout config/layouts/demo_warehouse_11x7_5.yaml`.

## 1. Nguồn

Bản vẽ **WAROTRANS · DEMO MAP v6** (sàn thật 3.50 × 4.55 m, ô gạch 0.6 m). Mọi số đo trong
`config/warehouse_layout.yaml` lấy từ thước của bản vẽ. Gốc toạ độ map nằm ở góc **tây nam**, trục x hướng đông, trục y
hướng bắc.

| Hàng (từ tường bắc) | Cao | y (m) |
|---|---|---|
| Bay CHG / REC / MAINT / PARK-01 / PARK-02 (5 × 0.70) | 0.60 | 3.95 – 4.55 |
| Lane trên (tách làn ở giữa) | 0.55 | 3.40 – 3.95 |
| SHELF-01, SHELF-02 | 0.60 | 2.80 – 3.40 |
| EP-S01/S02 · X-02 · X-01 · X-03 | 0.45 | 2.35 – 2.80 |
| EP-S03/S04 | 0.45 | 1.90 – 2.35 |
| SHELF-03, SHELF-04 | 0.60 | 1.30 – 1.90 |
| Lane dưới (speed 0.15 · nhập làn) | 0.55 | 0.75 – 1.30 |
| INBOUND 0.80 · QUALITY INSPECTION 1.90 · OUTBOUND 0.80 | 0.75 | 0.00 – 0.75 |

Các cột tính từ tường tây: 0.55 | 0.80 | 0.80 | 0.80 | 0.55, tức x = 0, 0.55, 1.35, 2.15, 2.95, 3.50. Mỗi kệ
0.70 × 0.50 m nằm giữa một cột 0.80, tại x = 0.95 / 2.55 và y = 3.10 / 1.60.

## 2. Những gì được sinh ra

- **Vật cản thật** (có trong map Nav2 và lidar quét thấy): tường bao cao 0.6 m, 4 kệ, trạm sạc sát tường bắc trong bay
  CHG-01.
- **Sơn sàn** (chỉ để nhìn, không vào map): nền lane màu xanh, màu từng zone theo bản vẽ, ô dừng 0.24 m tại mỗi
  endpoint.
- **13 endpoint** (`sim_endpoints.json` → backend qua `seed-sim.ps1`):

| Code | Tên | Loại | x, y | Hướng |
|---|---|---|---|---|
| S01 / S02 | EP-S01 / EP-S02 | STORAGE | 0.95 / 2.55, 2.575 | bắc (nhìn vào SHELF-01/02) |
| S03 / S04 | EP-S03 / EP-S04 | STORAGE | 0.95 / 2.55, 2.125 | nam (nhìn vào SHELF-03/04) |
| CHG-01 | EP-CHG-01 | CHARGING | 0.35, 4.22 | bắc |
| REC-01 | EP-REC-01 | RECOVERY | 1.05, 4.22 | bắc |
| MAINT-01 | EP-MAINT-01 | MAINTENANCE | 1.75, 4.22 | bắc |
| PARK-01 / PARK-02 | EP-PARK-01 / 02 | PARKING | 2.45 / 3.15, 4.22 | bắc |
| IN | EP-IN | INBOUND | 0.40, 0.405 | nam |
| QC-01 / QC-02 | EP-QC-01 / 02 | INSPECTION | 1.30 / 2.20, 0.405 | nam |
| OUT | EP-OUT | OUTBOUND | 3.10, 0.405 | nam |

- **15 zone**:
  - 5 bay CHG-01, REC-01, MAINT-01, PARK-01, PARK-02: `OPERATIONAL_AREA`, capacity 1;
  - X-01 ngã 4, X-02 / X-03 ngã 3, tách làn, nhập làn: `INTERSECTION`, capacity 1;
  - 2 lane tốc độ 0.15 m/s trước IN / QC / OUT: `NARROW_AREA`, capacity 1, max 0.15 m/s;
  - Inbound, Outbound: `OPERATIONAL_AREA`, capacity 1;
  - Quality Inspection: `OPERATIONAL_AREA`, capacity 2.
- Robot xuất hiện tại **EP-PARK-01**, mũi hướng bắc. Đây cũng là pose khởi tạo của AMCL.

**Không sinh** waypoint (chấm tròn trong bản vẽ) và chiều chạy một chiều của lane. Hai phần này để setup trên web
Map Editor.

## 3. Kiểm chứng

Gazebo headless, Nav2 như robot thật (inflation 0.35). Robot chạy một vòng liên tục:

```
PARK-01 → S02 → S03 → IN → QC-01 → QC-02 → OUT → S04 → S01 → CHG-01 → PARK-01
```

Kết quả: **10/10 SUCCEEDED**, 0 recovery, mỗi chặng 6–23 s, dừng cách đích 0.13–0.16 m. Lệnh chạy lại:

```bash
ros2 launch warotrans_simulation sim_bringup.launch.py gateway:=false            # terminal 1
ros2 run warotrans_simulation nav_regression.py --endpoints S02 S03 IN QC-01 QC-02 OUT S04 S01 CHG-01 PARK-01
```

`nav_regression.py --endpoints <code...>` dùng được với mọi layout, vì nó đọc toạ độ từ `sim_endpoints.json`.

Có một lần khởi động, `lifecycle_manager_localization` bị kẹt ở bước cấu hình `map_server`: log báo "failed to send
response … timeout", AMCL không lên, kéo theo Nav2 báo "Aborting bringup". Đây là lỗi race của DDS lúc khởi động,
không do map; chạy lại là hết. Nếu gặp lại thì tắt sim rồi bật lại.

## 4. Đưa map mới vào backend

```powershell
powershell -ExecutionPolicy Bypass -File samples/simulation/seed-sim.ps1     # repo backend
```

Script import map v6 thành MapVersion mới, tạo 13 endpoint và 15 zone, publish, rồi ghi lại `sim-config.json`
(gateway lấy `map_version_code` từ file này). Mỗi endpoint STORAGE được gắn một storage location
`LOC-SIM-S01..S04`.

Kịch bản `run-job-sim.ps1` (inbound / relocation) đang dùng alias của kho cũ (A, B, C, IN1, IN2, OUT1). Phải đổi sang
S01..S04, IN, OUT thì mới chạy được trên map v6.
