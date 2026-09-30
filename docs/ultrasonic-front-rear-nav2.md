# Phase 0.5 — Ultrasonic FRONT/REAR → Nav2 (enable guide)

> **Không bật** Collision Monitor / RangeSensorLayer trước khi  
> `tools/warotrans-check-ultrasonic-front-rear-geometry.sh` in `FRONT_REAR_GEOMETRY_OK`.  
> LEFT/RIGHT **không** đưa vào Nav2.

## Đã chuẩn bị trong repo

1. URDF: link `ultrasonic_front` / `ultrasonic_rear` chỉ khi `geometry.yaml` có đủ x/y/z/yaw (không null).
2. Bringup: `start_ultrasonic:=true` → include `warotrans_sensors/ultrasonic.launch.py` (publish Range; chưa dừng motor).
3. Overlay mẫu: [`ros2_ws/src/warotrans_navigation/config/nav2_collision_front_rear.yaml`](../ros2_ws/src/warotrans_navigation/config/nav2_collision_front_rear.yaml) — Collision Monitor sources FRONT+REAR.

## Bật an toàn (sau đo mount) — safety review

Wiring Collision Monitor **đổi ownership `/cmd_vel`**:

```text
controller_server → /cmd_vel_nav
collision_monitor  → đọc Range FRONT/REAR + /cmd_vel_nav → /cmd_vel
esp32_bridge       ← /cmd_vel
```

WHY: dừng/giảm tốc khi FRONT/REAR quá gần (vật đột ngột trước/sau).  
RISK: sai TF hoặc range lỗi → dừng nhầm / không dừng.  
TEST: flat target 0.2 / 0.5 m trước FRONT; verify `collision_monitor_state`; Nav2 Goal không đâm.  
ROLLBACK: `enable_ultrasonic_safety:=false`, controller lại publish thẳng `/cmd_vel`.

Các bước cụ thể (khi geometry OK):

1. Rebuild `warotrans_description` + `warotrans_navigation` trên Pi; `tf2_echo base_link ultrasonic_front` và `ultrasonic_rear`.
2. Launch: `start_ultrasonic:=true`.
3. Merge overlay CM + remap controller (chỉ khi đã review) — xem comment trong `nav2_collision_front_rear.yaml`.
4. Smoke DoD FRONT rồi REAR; không require LEFT/RIGHT.

## Monitoring-only (không đo mount)

```bash
ros2 launch warotrans_sensors ultrasonic.launch.py
# hoặc validate:
bash ~/tools/warotrans-ultrasonic-validate.sh
```

Chỉ xem `/ultrasonic/front` và `/ultrasonic/rear`; **không** tin vào Nav2 né bằng ultrasonic.
