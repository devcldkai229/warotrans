# LAB_RUNBOOK — MAP-ONE-V004 (RBT-001 rồi RBT-002)

Thứ tự cứng cho buổi lab. Không bỏ qua bước FAIL; ghi số đo vào `docs/calibration.md` khi đã có evidence.

**Map:** MAP-ONE-V004 (`map_one_v004`).  
**Hai robot:** xem `docs/DUAL_LANE_STATUS.md`.  
**Deploy:** `./tools/deploy.sh` (không wipe remote mặc định; sync giờ Pi).

**Trước buổi lab:** bundle V004 đã được tạo lại nên fingerprint đổi. Deploy lên Pi và khởi động lại Host để hai bên nạp cùng bundle, nếu không mọi plan bị từ chối `FINGERPRINT_MISMATCH`. Planner cũng đổi nên Pi phải build lại `warotrans_lane_planner`.

**Khi Host chạy bản có WH-01 và WH-08** (nhánh `feature/khai-*` của warotrans-system): TR nhiều thùng thành nhiều Job (`JobPlanning:MaxContainersPerJob`, mặc định 1), Job sau tự chạy khi Job trước kết thúc, sau khi người vận hành đã start Job đầu. Xác nhận lấy hàng làm thùng thành `IN_TRANSIT` và trừ tồn kệ nguồn. Job lỗi hoặc bị hủy khi hàng còn trên xe thì thùng vẫn `IN_TRANSIT` tới khi có người khai báo kệ (Containers → Recover). Bản này chưa chạy trên robot thật. Giữ bản cũ cho buổi lab đầu nếu không muốn đổi luồng.

**Chỉ giao job sau khi readiness đạt.** Goal gửi lúc mask keep-out chưa tới thì plan đầu đi xuyên keep-out: backend từ chối đúng, gate đóng, robot đứng tới khi có plan sạch (khoảng 20 giây trên rig).

---

## 1. Motor khóa — readiness

1. Bundle V004 trên Pi; Host `Traffic__SemanticMapDirectory` trỏ cùng fingerprint.
2. `BROKER=<ip> MOTORS=false ~/warotrans_tools/pi_run_semantic.sh start`
3. `python3 ~/warotrans_tools/real_robot_readiness.py --manifest <bundle>/semantic_manifest.json --mqtt-host <ip> --backend-url http://<ip>:5090 --home-x <EP-PARKING-A x> --home-y <y> --expect-motors-disabled`
4. Kỳ vọng: hai filter keep-out và speed có publisher, ba mask có ô đánh dấu, `/speed_limit` có publisher, fingerprint khớp Host log. Dòng E-stop sẽ FAIL nếu không truyền `--estop-tested`: đó là phần người vận hành xác nhận, không phải lỗi phần mềm.
5. Pi audit: `~/warotrans_tools/doctor.sh` (hoặc check.sh).
6. IMU (nếu có): quay tay 90° / 360° — ghi vào calibration; chưa có → **CHƯA XÁC NHẬN**.
7. LiDAR: `ros2 topic hz /scan`; self-hit scan không đâm thân.
8. Nút HUMAN_CONFIRM: `human_confirm_enabled: true` — bấm → log `BUTTON_PRESSED` / MQTT confirm.

## 2. Motor mở — đo động học

1. `MOTORS=true` chỉ khi E-stop (khuyên) trong tầm với.
2. `python3 ~/warotrans_tools/measure_speed.py --seconds 20` trong khi teleop/Nav2 — ghi mean odom/cmd và peak.
3. Quay tại chỗ + quán tính: so yaw lệnh vs đo; chốt `yaw_goal_tolerance` (code: 0.13 rad) sau lab.
4. EKF on/off: chỉ đổi khi có evidence BNO055; mặc định wheel-only.

## 3. Endpoint + chặng chính

1. 10 endpoint V004: đặt pose, NavigateToPose hoặc TR → đo sai số xy/yaw vs scan-match.
2. RESULT MQTT có `arrivalErrorXyM` / `arrivalErrorYawRad` (chưa hard-reject).
3. Lặp chặng chính (PARKING_A↔INBOUND, INBOUND→SHELF_*, →OUTBOUND).
4. Rig tham chiếu: `run_rig.py --map map_one_v004 --all-pairs` (90 cặp) khi cần offline.

## 4. Obstacle + mất liên lạc + handover

1. Tham chiếu rig: `run_rig.py --obstacle-suite` (6 kịch bản). Hộp đặt giữa lane không có đường vòng hợp lệ: khoảng trống trong lane nhỏ hơn thân xe, lane đối diện một chiều, nên robot chờ rồi báo chặn. V004 không có đường vòng cho lane bị chắn hẳn (`docs/DUAL_LANE_STATUS.md`). Rig không chạy handler thật nên thời gian abort của nó (tới 167 giây) không phải mốc PAUSED; mốc 60 giây phải xem ở bước 2.
2. Lab: chắn lối → sau khoảng 60 giây không tiến được (gate mở), job **PAUSED** (`NAVIGATION_BLOCKED`); Resume sau khi dọn. Chờ traffic không tính vào 60 giây này.
3. Mất Wi-Fi ngắn: gate đóng; khi nối lại robot báo trạng thái, nếu backend đã đóng lệnh thì backend gửi CANCEL hủy goal mồ côi.
4. TR → nút HUMAN_CONFIRM → handover → COMPLETED.

## 5. RBT-002 (chỉ sau khi 1–4 PASS trên RBT-001)

1. Đỗ `EP-OUTBOUND`; `ROBOT_CODE=RBT-002 ROS_DOMAIN_ID=<khác> pi_run_semantic.sh start`.
2. LiDAR thấy nhau trước khi cấp lệnh song song.
3. Chạy trước với thiết lập mặc định: robot sau chạy tới trước xung đột rồi chờ, an toàn, không song song.
4. Không kỳ vọng song song hai lane trên V004 ở buổi này: xem mục "Giới hạn đã đo" trong `docs/DUAL_LANE_STATUS.md`.

---

## Ngoài phạm vi (ghi nhận, không block buổi 1)

- BMS / fuse cell — CHƯA XÁC NHẬN  
- Ultrasonic low-obstacle  
- systemd autostart  
- Active Mode V004  
- E-stop HW (khuyên khi 2 robot)  
- Đoạn PARKING-A→center không có lane trên V004  

## Checklist nhanh DoD

| Hạng mục | Rig / unit | Lab |
|---|---|---|
| Mọi endpoint | 90/90, 0 va chạm | 10/10 EP |
| Yaw | tối đa 0,109 rad (giới hạn 0,14, có quán tính) | đo thật |
| Tim lane | tối đa 0,071 m (giới hạn 0,10) | đo thật |
| Speed mask | Không vượt trần | odom ≤ trần |
| Keep-out | thân xe cách ≥ 0,079 m; thiếu filter → readiness FAIL | không vào vùng |
| Obstacle | 6/6 kịch bản; PAUSED @60s (unit test) | như trên |
| Hai robot | `test_dual_lane_harness.py` 5/5, song song chưa đạt với plan thật | Chạy lần lượt |

Sát tường nhất 3,2 cm khi xoay tại SHELF-A trên rig (không tính sai số định vị): quan sát kỹ chỗ này trên robot thật.
