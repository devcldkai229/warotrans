# 06 — World Gazebo từ map quét thật `warotrans_remap_20261104_v3`

## Nguồn
- Map SLAM quét ngày 2026-11-04 (bản v3): `warotrans_remap_20261104_v3.{yaml,pgm}`
  - 66 × 103 px, độ phân giải 0.05 m → sàn ≈ 3.3 × 5.15 m
  - origin `[-0.341, -0.475, 0]`, trinary
- Bản trong sim: `ros2_ws/src/warotrans_simulation/maps/real/`.
  Bản dùng cho robot thật: `ros2_ws/src/warotrans_navigation/maps/` (để cạnh các remap trước).

## Map được biến thành world như thế nào
`config/warehouse_layout.yaml` là layout đang dùng, kiểu `source_map`.
`scripts/generate_warehouse.py` xử lý layout này như sau:
1. Mỗi ô occupied của PGM thành một khối đặc cao 0.8 m. Nhờ vậy lidar trong sim thấy đúng những gì AMCL/Nav2 thấy trên map.
2. Ô occupied nằm trong một hình chữ nhật `shelves` thì đổi sang màu đế kệ. Phía trên được vẽ thêm kệ, gồm cột, sàn và hàng hoá, chỉ từ z ≥ 0.45 m (cao hơn mặt lidar).
3. Generator ghi ra các file dùng chung tên, nên lệnh launch không cần đổi tham số:
   - `worlds/warehouse.sdf`
   - `maps/sim_warehouse.{pgm,yaml}`
   - `config/sim_endpoints.json`

Map được dùng nguyên trạng. Ngoại lệ duy nhất là `clear_cells`: những ô nằm trong các hình chữ nhật này được đặt thành trống, cho cả world Gazebo lẫn `maps/sim_warehouse.pgm` (map Nav2 trong sim), nên hai bên luôn khớp nhau. File gốc trong `maps/real/` không bị sửa.

Hiện có 2 vùng:
- row 77–78, col 59–65 (x 2.60–2.96, y 0.73–0.82): đoạn tường ngay trước SHELF-03. Bỏ đi để kệ quay mặt ra khoang phía bắc.
- row 65–77, col 0–3 (x -0.34 – -0.15, y 0.78–1.42): đoạn tường tây ngay trước SHELF-04. Bỏ đi để hốc trái dẫn thẳng vào kệ. Hai đoạn tường ngang của hốc (row 64 và row 78) được giữ lại.

Lưu ý: map Nav2 trên robot thật vẫn còn 2 đoạn tường này.

## Vị trí kệ
Công thức đổi ô PGM sang toạ độ (tâm ô):
- `x = -0.341 + 0.05·(col + 0.5)`
- `y = -0.475 + 0.05·(102.5 − row)`

| Kệ | Ô PGM | x (m) | y (m) | Ghi chú |
|---|---|---|---|---|
| SHELF-01 | col 36–44, row 29–34 | 1.46 – 1.91 | 2.93 – 3.23 | viền đen, ruột xám (lidar không nhìn vào trong) |
| SHELF-02 | col 36–44, row 64–77 | 1.46 – 1.91 | 0.78 – 1.48 | như trên |
| SHELF-03 | col 58–65, row 79–85 | 2.56 – 2.96 | 0.38 – 0.73 | sát mép phải map, kệ đặc từ sàn, mặt bắc mở (xem `clear_cells`) |
| SHELF-04 | ngoài tường tây, ngang row 64–78 | -0.80 – -0.35 | 0.78 – 1.48 | ngoài vùng quét, kệ đặc từ sàn (`solid_base: true`) |

Biên của mỗi hình chữ nhật được chọn sao cho:
- tâm các ô viền kệ nằm bên trong;
- tâm các ô tường liền kề nằm bên ngoài, nên vẫn là tường.

**SHELF-04** đứng ở phía ngoài tường tây, nằm ngoài vùng robot quét được, nên trong PGM không có ô nào của nó. Generator in cảnh báo `no occupied cell`, đây là điều đã biết trước.
Kệ này dùng `solid_base: true`, tức là một kệ đặc từ sàn lên. Tường tây che nó khỏi lidar, nên Nav2/AMCL không bị ảnh hưởng.
Tuỳ chọn `solid_base` được thêm vào `render_from_source_map` (mặc định `false`). Nó dành cho những kệ nằm ngoài vùng quét.

Hốc ở tường trái (row 64–78, col 3–9) dẫn vào SHELF-04, vì đoạn tường tây trước kệ đã được bỏ (xem `clear_cells`).

## Mask ngữ nghĩa (bắt buộc với `nav_runtime=nav2_semantic`)
Ở chế độ mặc định, `sim_bringup.launch.py` cần các file nằm cạnh map: `keepout.yaml`, `speed.yaml`, `lanes.json`.
Các file này được sinh bằng `semantic_map_adapter.export_from_snapshot({}, ...)` và đặt trong `maps/`. Hiện chúng rỗng: không có vùng cấm, không giới hạn tốc độ, không có lane.
Khi map có zone trên web, cần sinh lại các mask này từ snapshot.

## Spawn
Robot spawn tại `(0, 0, yaw 0)`. Đây là gốc map, cũng là nơi robot đứng khi bắt đầu quét SLAM.

## Chạy
```bash
cd ~/warotrans_ws/src/warotrans_simulation
python3 scripts/generate_warehouse.py          # sau khi sửa layout
python3 scripts/generate_warehouse.py --check  # 0 = world đã khớp layout
cd ~/warotrans_ws && colcon build --symlink-install --packages-up-to warotrans_simulation
ros2 launch warotrans_simulation sim_bringup.launch.py
```

## Quay lại map cũ (MAP-WH42CFE65F-V003)
```bash
cp config/layouts/map_wh42cfe65f_v003.yaml config/warehouse_layout.yaml
python3 scripts/generate_warehouse.py
```

## Giới hạn hiện tại
- Backend/web chưa có MapVersion cho map này, và chưa có endpoint hay zone (layout không có `web.overlay`).
  Muốn chạy job end-to-end trên web với map mới, cần làm 3 bước:
  1. Import map lên web Map Editor.
  2. Tạo endpoint.
  3. Chạy `scripts/sync_web_map.py`, rồi thêm `web.overlay` và `spawn_endpoint_code` vào layout.
