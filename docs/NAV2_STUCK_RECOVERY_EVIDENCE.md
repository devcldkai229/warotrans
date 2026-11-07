# Nav2: kẹt sát tường và lệch path — bằng chứng đo được

Cập nhật 2026-11-06. **Chưa có số đo nào từ robot thật.** Pi không kết nối được trong đợt này và robot chưa được phép chuyển động (chưa có E-stop phần cứng). Vì vậy chưa thể khẳng định lần kẹt khi test bằng RViz do nguyên nhân nào; rig chỉ tái hiện được triệu chứng giống vậy.

Mức bằng chứng:

- **Source:** đọc source Nav2 Jazzy.
- **Rig:** đo trong `tools/nav_recovery_rig` (Nav2 thật, config thật, robot và LiDAR mô phỏng, map→odom tĩnh, không AMCL, không trượt bánh, không có backend).
- **Tính toán / Config:** từ số hình học hoặc file cấu hình.
- **Giả thuyết:** chưa đo.

## Nguyên nhân

| Nội dung | Kết luận | Mức |
|---|---|---|
| Collision Monitor mô phỏng cả lệnh xoay; comment cũ "xoay không chạm tường" là sai | **Đã xác minh** | Source + Rig |
| Có từ 4 điểm scan trong polygon thì mọi lệnh về 0, kể cả lùi (đo được `-0.06 → -0.0`) | **Đã xác minh** | Source + Rig |
| Sát biên padding, robot chỉ thoát khi nhiễu làm số điểm tụt dưới 4 | **Đã xác minh trong rig**, cần đo trên LiDAR thật | Rig |
| Controller tự xoay về phía tường rồi tự khoá: `side_05` khe hở 5,5 → 2,4 cm, sau đó monitor chặn đến hết 240 s | **Đã xác minh trong rig** | Rig |
| BackUp đầu tiên đến rất muộn: khoảng 127 s ở `corner_05`, `side_05`, `rear_blocked` | **Đã xác minh trong rig** | Rig |
| BackUp và Spin của Nav2 bị costmap từ chối sát tường (`Collision Ahead`, `side_10`) | **Đã xác minh trong rig** | Source + Rig |
| Ultrasonic là nguyên nhân | **Bác bỏ**: chưa nối vào Nav2 | Config |
| Hạ `obstacle_min_range` xuống 0,15 m sẽ giúp thoát | **Không được ủng hộ**: kết hợp với allowance 10 s, `side_10` chuyển thành abort | Rig |
| Lệch path do zigzag, EKF tắt hoặc motor open-loop | **Giả thuyết**: rig không có AMCL và trượt bánh nên không kiểm tra được | — |

## Lỗi đã sửa ở Semantic Traffic

Gate cho phép vào mọi cell trong grant, kể cả cell robot đã báo `RESOURCES_CLEARED` mà backend có thể đã cấp cho robot khác. Đi tiến thì không gặp, nhưng lùi hoặc replan quay lại thì có thể. Giờ gate đóng với lý do `IN_RELEASED_RESOURCE`, và recovery cũng từ chối lùi vào đó. Có test trong `test_semantic_traffic_client.py`.

## Trước và sau

Cùng rig, không deadband, chỉ tính lần chạy hợp lệ. Trong mọi lần chạy: **0 va chạm**.

| Ca | Trước | Sau |
|---|---|---|
| Mũi 10 cm | tới đích 43 s | tới đích 46 s |
| Mũi 20 cm | tới đích 41 s | tới đích 44 s |
| Mũi 2 cm | không hợp lệ | tới đích 90 s, 89 s |
| Mũi 5 cm | không hợp lệ | tới đích 86 s |
| Góc tường | tới đích 171 s | tới đích 84 s, 86 s |
| Hông 10 cm | **abort** 39 s | **tới đích** 71 s, 58 s, 58 s |
| Hông 1 cm | còn kẹt lúc 240 s | còn kẹt lúc 240 s |
| Hông 5 cm | còn kẹt lúc 240 s | còn kẹt lúc 240 s (2 lần) |
| Đường cụt | còn kẹt lúc 240 s | còn kẹt lúc 240 s |
| Vật cản phía sau | còn kẹt lúc 240 s | còn kẹt lúc 240 s; abort 216 s |
| Gate đóng | 0 chuyển động | 0 chuyển động, guard từ chối `NO_GRANT` |
| Đi thẳng | tới đích 14 s | tới đích 14 s |

Thời điểm guard được gọi lần đầu ở `corner_05`, `side_05`, `rear_blocked`: khoảng 42 s, so với khoảng 127 s cho BackUp đầu tiên trước đây.

Số lần lặp ít và có dao động, nên các con số chỉ có giá trị định hướng. Ca mũi 2 cm và 5 cm trước khi sửa bị loại vì global costmap chưa nhận map, nên không so sánh được.

Rig với deadband giả định (0,03 m/s; 0,2 rad/s) chỉ chạy trên cây cũ và cho kết quả khác đáng kể: `nose_05` chưa tới đích sau 240 s. Deadband thật chưa đo, nên kết quả trên robot có thể khác.

## Giới hạn còn lại

Những ca sau **không thoát được, và đó là chủ ý**, vì không nới lớp an toàn:

- Hông cách tường dưới khoảng 5 cm, hoặc robot đã tự xoay tới mức padding chạm tường: cần người đẩy ra.
- Lùi: vật thấp hơn mặt quét LiDAR (~17,5 cm) ở phía sau **không được phát hiện**.
- Vật mảnh trong vùng mù 0,15 m của LiDAR không được phát hiện.
- Muốn xoay tròn tự do cần cách thân xe 7,4 cm ở mũi và 10,4 cm ở hông.

## Recovery hiện tại

BT gọi `guarded_backup` và `guarded_spin` thay vì gọi thẳng Nav2. Guard (`warotrans_fleet/recovery_guard.py`) không có đầu ra vận tốc; nó từ chối, hoặc chuyển nguyên goal cho Nav2.

- **Từ chối khi:** scan quá 0,5 s, không có pose, vùng quét có từ 4 điểm trở lên, hoặc gateway không xác nhận grant trong 0,5 s.
- **BackUp** xét nửa sau footprint có padding cộng vùng quét. Điểm phía trước không tính vì lùi đi xa khỏi chúng. **Spin** xét toàn bộ footprint và cung xoay theo từng chiều.
- **Grant:** lùi 0,10 m nằm trong phần dư tính được ≥ 0,131 m. Chỉ tính cho grant còn hiệu lực; trong client, mọi vị trí trên đường lùi vẫn được kiểm tra lại.
- Collision Monitor và authority gate vẫn kiểm từng lệnh trong lúc recovery chạy.

Không đổi: padding 0,02 m, `time_before_collision` 1,2 s, `min_points` 4, chế độ chỉ đi tiến của controller chính, gate.

## Tham số chưa đổi

| Tham số | Số đo | Quyết định |
|---|---|---|
| `movement_time_allowance` 20 → 10 s | `corner_05` 65–68 s so với 84–86 s; `rear_blocked` abort 136 s so với 216 s; `side_10` không tệ đi | **Giữ 20 s**: quay 180° trên robot thật chưa đo tốc độ, có thể bị tính là không tiến triển; chưa test khi gate giữ xe |
| `obstacle_min_range` 0,20 → 0,15 m | Kết hợp với trên: `side_10` và `side_05` abort | **Giữ 0,20 m**; cần đo điểm tự chạm trên robot thật |
| SmoothPath | Lệch ngang p95 khoảng 3–6 cm trong rig, chưa có dữ liệu thật | **Không thêm**. `smoother_server` hiện không được launch; khi thêm phải chuyển gateway và validator sang `plan_smoothed`, nếu không robot chạy khác path được grant |
| Goal tolerance | Xem dưới | **Chưa đổi**, cần số đo thật và quyết định của bạn |

## Goal tolerance

Endpoint V002 yêu cầu 0,08 m và 0,175 rad; V003 yêu cầu 0,08 m và 0,140 rad. Nav2 chấp nhận 0,15 m và 3,14 rad.

Trong rig, các chặng thành công **đều dừng cách đích 0,14–0,15 m** vì Nav2 dừng ngay khi vào vòng tolerance, và sai hướng tới khoảng 2,4 rad. Tức là gần như mọi lần tới nơi đều vượt `PositionTolerance` 0,08 m, nhưng backend vẫn tính là đạt, vì khi tới nơi nó chỉ tin Nav2 SUCCESS. `PositionTolerance` chỉ dùng khi resume job; `YawTolerance` không được kiểm tra khi tới nơi.

Gateway đã ghi `arrivalErrorXy` và `arrivalErrorYaw` để có số thật.

## MAP-ONE-V004: planner bám tim lane (2026-11-07)

V004 thay V003 làm map duy nhất. Lần chạy thật trên V003 cho thấy robot đi giữa hai lane ngược chiều và đường uốn lượn. Nguyên nhân: `LaneAwareNavfn` tính mọi ô trong lane cùng giá nên đường bám mép corridor, và heuristic không tương thích với giá `lane_prefer_factor`.

Đã sửa: thêm `lane_center_weight`, sửa heuristic, rút gọn đường ngay trong planner. Rig 9 chặng V004, chỉ mô phỏng, không có AMCL, trượt bánh hay robot thật. Planner cũ là một lần chạy, planner mới là một lần chạy (riêng PARKING_A → INBOUND chạy lại do lần đầu INVALID):

| Chặng | Cũ | Mới | Lệch tim lane trung bình, cũ → mới |
|---|---|---|---|
| PARKING_A → INBOUND | tới | tới | TTB-C 0,33 → 0,08 m |
| INBOUND → SHELF_A | **abort 105** | **tới** | n/a |
| SHELF_B2 → QUEUE | tới | tới | RTL-B 0,28 → 0,10; TTB-C 0,31 → 0,10 m |
| SHELF_A → OUTBOUND | abort 104 | **vẫn abort 104** | n/a |
| OUTBOUND → PARKING_A | abort 104 | **vẫn abort 104** | BTT-C 0,32 → 0,12 m |
| 4 chặng còn lại | tới | tới | không tệ đi |

Hai chặng còn abort báo RPP "collision ahead": SHELF_A → OUTBOUND ngay gần SHELF_A, OUTBOUND → PARKING_A quanh (1,9; 3,7). Chưa có nguyên nhân gốc, chặn chạy thật hai chặng này.

## MAP-ONE-V003 (đã thay bằng V004)

**Đây là vấn đề lớn nhất trước khi chạy thật.** Map đã được publish lúc 14:28 ngày 2026-11-06; V002 thành ARCHIVED; RBT-001 trong DB vẫn gắn V002.

Rig đi 9 chặng giữa các endpoint V003: 6 tới đích, **3 abort**. Ba chặng abort lặp lại ở mọi lần, với cả cây mới và cây cũ (mỗi cây 2 lần), nên **không phải do đợt sửa này, và đợt sửa này cũng không giải quyết được**:

| Chặng | Dừng ở | Lỗi |
|---|---|---|
| INBOUND → SHELF-A-01 | khoảng (1,6; 0,5), cạnh góc khối | RPP "collision ahead", code 104 |
| SHELF-A-01 → OUTBOUND | ngay tại SHELF-A-01 | Monitor chặn, code 104/105 |
| OUTBOUND → PARKING-A | khoảng (1,88; 3,72), cạnh đầu keepout x≈2,16 | RPP "collision ahead", code 104 |

Giả thuyết, chưa xác nhận: path ôm góc và pure pursuit cắt góc, hoặc lối đi quá hẹp với lane và keepout hiện tại.

Khoảng trống quanh endpoint V003 (`tools/nav_recovery_rig/map_clearance.py`, tính theo pixel map):

| Endpoint | Tình trạng |
|---|---|
| EP-QUEUE-POINT | **Không xoay tại chỗ được**: đuôi chỉ cách tường 2,4 cm |
| EP-SHELF-A-01 | **Không xoay tại chỗ được**: hông phải cách 9,4 cm |
| 8 endpoint còn lại | Đủ chỗ xoay |

## Việc cần làm trên Pi (khoá motor)

Chưa làm; cần Pi bật và quyền SSH. Dùng `MOTORS=false`, không gửi goal.

1. **Audit:** so `ros2 param dump`, topic graph, TF và BT đang nạp với repo.
2. **Scan:** đặt tường ở 10, 15, 20, 30 cm theo thước, ghi bằng `nav_stuck_probe.py record`, tìm điểm tự chạm và vùng mù thật.
3. **IMU/EKF:** bật `EKF=true`, xoay tay 90° và 360° so với mốc. Đề xuất đạt: sai dưới 3° và 5°. Đạt thì dùng EKF; không đạt thì giữ mặc định `EKF=false` (AMCL alpha 0,3/0,2).
4. **Odom bánh:** đẩy tay 1 m, so với thước.
5. **AMCL:** đứng yên 5 phút, xem độ nhiễu.

Sai số bám path chỉ đo được sau khi có E-stop đã test, với người giám sát.

## Tái hiện

```bash
wsl -e bash tools/nav_recovery_rig/build_ws.sh
wsl -e bash -lc 'source /opt/ros/jazzy/setup.bash; python3 tools/nav_recovery_rig/run_rig.py --label before --parallel 3'
wsl -e bash -lc 'source /opt/ros/jazzy/setup.bash; python3 tools/nav_recovery_rig/run_rig.py --label v004 --map map_one_v004'
python tools/nav_recovery_rig/map_clearance.py <bundle> <endpoints.csv>
```

Rig dùng `ROS_LOCALHOST_ONLY=1`, không có MQTT, backend hay DB. Lần chạy nào có stack khởi động lỗi hoặc thiếu dữ liệu sẽ tự đánh dấu `INVALID` và chạy lại.

Harness traffic phải chạy **từng file một**, vì Host của session giữ cổng 5199 và lease.
