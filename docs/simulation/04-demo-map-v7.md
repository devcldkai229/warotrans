# Demo map v7 trên Gazebo: chỉ tường và kệ, phần còn lại setup trên web

> Nhánh `feature/simulation-demo-map-v6` · 2026-10-21. Thay cho [demo map v6](03-demo-map-v6.md); layout v6 được
> giữ ở `config/layouts/demo_map_v6.yaml`.

## 1. Có gì trong map

Bản vẽ tay **WAROTRANS · DEMO MAP v7** (sàn thật 3.50 × 4.55 m). `config/warehouse_layout.yaml` **chỉ** dựng phần
vật lý:

- tường bao cao 0.6 m;
- 3 kệ 1.20 × 0.35 m (vật cản thật, lidar quét thấy):

| Kệ | x (m) | y (m) |
|---|---|---|
| SHELF-01 | 0.85 – 2.05 | 3.00 – 3.35 |
| SHELF-02 | 0.85 – 2.05 | 1.75 – 2.10 |
| SHELF-03 | 0.85 – 2.05 | 1.35 – 1.70 |

SHELF-02 và SHELF-03 chỉ cách nhau 5 cm, nhỏ hơn một ô 5 cm của map. Vì vậy trong map Nav2 chúng thành **một khối**.
Robot cũng không chui qua khe đó được.

**Không tạo** endpoint, zone, lane hay sơn sàn. Các phần CHARGING, PARK-01/02, 1 ROBOT, ngã 3, IN / OUT / QC, lane
một chiều… đều vẽ trên web Map Editor. Robot xuất hiện trong bay PARK-01 (2.45, 4.22), mũi hướng nam (lối "ra").

Gốc toạ độ ở góc tây nam. Các hàng tính từ tường bắc: 0.65 | 0.55 | 0.35 | 0.45 | 0.45 | 0.40 | 0.40 | 0.55 | 0.75 m.

## 2. Đưa vào backend ở trạng thái DRAFT

```powershell
powershell -ExecutionPolicy Bypass -File samples/simulation/seed-sim.ps1 -Draft      # repo backend
```

`-Draft` chỉ import ảnh map, **không** tạo endpoint hay zone và **không** publish. Chạy lại thì dùng lại bản DRAFT có
cùng nội dung. Lần chạy 2026-10-21 tạo **MAP-WHSIM-V013 (DRAFT)**. Robot RBT-SIM-01 được chuyển sang map này, và
`sim-config.json` trỏ gateway tới `MAP-WHSIM-V013`.

Sau khi vẽ xong endpoint, zone và lane trên web thì publish map từ web. Map published trước đó, MAP-WHSIM-V012
(demo map v6), vẫn là active map cho tới khi đó.

Các tính năng chỉ chạy trên map published (tạo transport request, chạy job) sẽ dùng được với map v7 sau khi publish.
