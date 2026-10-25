# Gazebo dựng từ map upload trên web (warotrans_map3 · MAP-WHSIM-V016)

> Nhánh `feature/sim-world-from-web-map` · 2026-10-25. Layout v7 (vẽ tay) chuyển sang `config/layouts/demo_map_v7.yaml`.
>
> **Cập nhật cuối ngày:** layout đang trỏ tới **MAP-WHSIM-V018** (id `7cb001f7-…`). Đây là bản clone của V016/V017, cùng
> ảnh map, gồm 11 endpoint, 13 lane và 12 zone; snapshot của nó nằm ở repo backend tại `samples/maps/MAP-WHSIM-V018`.
> Lane được sơn theo kiểu web mới: dải rộng đúng width của lane, kèm chevron; lane 2 chiều có vạch giữa và 2 hàng
> chevron ngược nhau. Route graph V018 hợp lệ. Chạy thử `route_graph:=` tới EP-PARK-1/2, EP-CHARGER-1, EP-INSPECTION
> rồi về EP-INBOUND: 5/5 thành công. Robot dừng cách điểm khoảng 0.14 m và không quay đúng yaw của endpoint vì
> `xy_goal_tolerance` = 0.15 và `yaw_goal_tolerance` = 3.14; việc này chưa sửa.

## 1. Ý tưởng: một file map cho cả 3 nơi

Gốc toạ độ là `origin` của file yaml, nên một toạ độ trên web là đúng chỗ đó trong Gazebo.

```
web (upload map.yaml + map.pgm) ──► backend data/maps/…/map.pgm ──► sync_web_map.py ──► maps/web/warotrans_map3.pgm
                                                                                      │   (giống từng byte)
                                     Nav2 / AMCL  ◄── maps/sim_warehouse.pgm ◄────────┤
                                     Gazebo world ◄── mỗi ô đen = 1 khối đặc ◄────────┘
```

- **Không sửa file map.** File pgm trong sim trùng sha256 với file bạn upload.
- **Lidar ảo (`gpu_lidar`, cao ~0.2 m) thấy đúng map.** Từ sàn lên 0.30 m, Gazebo chỉ có các khối dựng từ ô đen của
  map. Kiểm chứng: 960/960 ô đen trùng với vật cản Gazebo ở độ cao lidar, không thiếu, không thừa. Vì vậy AMCL định
  vị như trên robot thật.
- **Phần làm đẹp nằm trên tầm lidar hoặc chỉ là sơn sàn:**
  - tường: chân tường xám đậm 0–0.30 m, phần trên màu kem tới 0.8 m;
  - kệ: đế lấy từ ô của map, cao 0.45 m; trụ xanh, thanh ngang cam, 2 tầng kệ và thùng hàng nằm **từ 0.45 m trở lên**;
  - sàn: bê tông, ron mỗi 0.5 m, khớp với lưới 0.5 m của ảnh map;
  - zone, lane, endpoint vẽ trên web được sơn xuống sàn (cao vài mm), dùng **đúng màu của web** (`mapStyle.ts`).

## 2. Kệ

Năm kệ bạn đánh dấu đỏ được khai báo bằng hình chữ nhật (mét, frame map) trong `config/warehouse_layout.yaml`:

| Kệ | x | y | Ghi chú |
|---|---|---|---|
| SHELF-01 | 0.756 – 1.206 | −1.219 – −0.469 | map thấy viền rỗng |
| SHELF-02 | 1.956 – 2.406 | −1.219 – −0.469 | |
| SHELF-03 | 3.206 – 3.656 | −1.219 – −0.419 | |
| SHELF-04 | 0.806 – 1.956 | −2.619 – −2.219 | trong hốc tường nam |
| SHELF-05 | 2.556 – 3.656 | −2.619 – −2.169 | trong hốc tường nam |

Ô đen **trong** hình chữ nhật thành kệ, ô đen ngoài thành tường. Sửa toạ độ ở đây chỉ đổi phần nhìn, không đổi map.

## 3. Đồng bộ sau khi sửa trên web

```bash
cd ~/warotrans_ws/src/warotrans_simulation
python3 scripts/sync_web_map.py --user admin --password '<mật khẩu>'      # hoặc WAROTRANS_TOKEN=<jwt>
```

Script làm 4 việc:
1. Lấy `GET /api/navigation/maps/{id}/detail`.
2. Tải `map.pgm` / `map.yaml` từ `/map-assets`. Nếu map trên web đổi thì in `map CHANGED`.
3. Ghi `config/web/warotrans_map3.overlay.json`, gồm endpoint, lane và zone.
4. Chạy `generate_warehouse.py`.

Sau đó **tắt và bật lại simulation** để Gazebo nạp world mới. Endpoint trên web cũng được ghi vào
`sim_endpoints.json`, nên dùng được `nav_regression.py --endpoints <code…>`.

- **Đổi sang map version khác** (clone, upload mới): chạy `--map-id <guid>`, rồi sửa `web.map_version_id` /
  `map_version_code` trong layout.
- **Chưa hỗ trợ:** map có `origin` xoay (yaw ≠ 0) hoặc `negate: 1`. Script sẽ báo lỗi rõ.

## 4. Chạy

```powershell
wsl -d Ubuntu-24.04 -- bash /mnt/d/WaroTransSim/scripts/run-sim.sh            # map code lấy từ sim-config.json
```

- Robot xuất hiện tại (0, 0), hướng +x. Đó là chỗ robot thật bắt đầu SLAM, và cũng là pose khởi tạo của AMCL.
- Lần chạy 2026-10-25 đã làm thêm 2 việc ngoài repo:
  - `RBT-SIM-01` được chuyển sang `MAP-WHSIM-V016` bằng SQL (`fleet.robots.current_map_version_id`);
  - `samples/simulation/sim-config.json` được trỏ tới V016.
- Backend nhận robot `AVAILABLE` / `LOCALIZED`, toạ độ khớp Gazebo.

## 5. Kiểm chứng (Gazebo, Nav2 như robot thật)

| Goal (x, y, yaw) | Kết quả |
|---|---|
| (1.58, −1.70, 0) lối giữa hàng kệ và kệ phía nam | SUCCEEDED 23 s |
| (4.10, −1.80, π/2) phía đông | SUCCEEDED 22 s |
| (2.80, −0.80, −π/2) lối giữa SHELF-02 / SHELF-03 | SUCCEEDED 25 s |
| (2.80, −1.60, 0) | SUCCEEDED 13 s |
| (0, 0, 0) về chỗ xuất phát | SUCCEEDED 23–26 s |
| (2.80, −0.80, **π**): quay đầu 180° trong lối 0.8 m | **không xong sau 150 s** |

Ca cuối thất bại vì RPP liên tục báo `detected collision ahead` khi xoay tại chỗ giữa 2 kệ. Đây là giới hạn của
cấu hình Nav2 ở lối hẹp, không phải do map: cùng điểm đó với hướng dọc lối thì chạy được. Khi đặt endpoint trên web,
nên cho hướng dọc theo lối đi.

## 6. Quay lại map khác

```bash
python3 scripts/generate_warehouse.py --layout config/layouts/demo_map_v7.yaml   # hoặc demo_map_v6 / demo_warehouse_11x7_5
```
