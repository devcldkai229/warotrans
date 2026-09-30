# warotrans_teleop

Mobile web teleop for Phase 7.5 commissioning. Serves a phone-friendly UI on port **8080** and publishes `/cmd_vel` plus `/motor_command_state`.

## Architecture

```text
Phone browser -> HTTP/WebSocket-less API -> web_teleop_node
    -> /cmd_vel -> esp32_bridge_node -> V <linear> <angular> -> ESP32 -> MDD10A
    -> /motor_command_state (computed PWM command debug)
    <- /odom (encoder-estimated speed display only)
```

The browser never accesses GPIO, serial, or `/dev/warotrans`.

## Build (Raspberry Pi)

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select \
  warotrans_msgs warotrans_hardware warotrans_teleop warotrans_bringup
source install/setup.bash
```

## Launch

Full mobile base commissioning stack:

```bash
ros2 launch warotrans_bringup mobile_base.launch.py
```

Teleop only:

```bash
ros2 launch warotrans_teleop teleop.launch.py
```

## Phone access

1. Connect phone and Pi to the same Wi-Fi network.
2. On Pi: `hostname -I` to find the IP address.
3. Open on phone: `http://<pi-ip>:8080`

Optional hotspot helper (run manually on Pi, not automatic):

```bash
bash ~/warotrans_tools/hotspot_setup.sh
```

## Safety layers

| Layer | Timeout | Action |
|---|---:|---|
| Finger release | immediate | browser POST `/api/stop` |
| Server deadman | 250 ms | zero `/cmd_vel` |
| ESP32 watchdog | 300 ms | PWM left/right = 0 |

## Commissioning constants (PROVISIONAL)

| Item | Value |
|---|---|
| PWM resolution | 8-bit, max raw 255 |
| Hard PWM ceiling | 40% = raw 102 |
| Default speed level | 20% = raw 51 |
| Full-scale linear command | 0.32 m/s (not measured) |
| Full-scale angular command | 2.9317 rad/s (derived) |
| Motor inversion defaults | `LEFT_MOTOR_INVERTED=false`, `RIGHT_MOTOR_INVERTED=false` |

## Wheels-lifted test procedure

**Always lift wheels before Tests 2-6.**

### Test 1 — Boot idle

Launch stack, send no command.

Expected: LEFT PWM = 0, RIGHT PWM = 0.

### Test 2 — Hold FORWARD

Expected while held: left forward, right forward. On release: immediate STOP.

### Test 3 — Hold BACKWARD

Expected: left backward, right backward.

### Test 4 — Hold LEFT

Expected: left backward, right forward.

### Test 5 — Hold RIGHT

Expected: left forward, right backward.

### Test 6 — Speed sweep

Set 10 → 15 → 20 → 25 → 30 → 35 → 40 %. UI PWM command raw values:

26, 38, 51, 64, 77, 89, 102. Never exceed 40%.

### Test 7 — Wi-Fi drop while moving

Hold FORWARD, disable phone Wi-Fi.

Expected: server deadman stops `/cmd_vel`; ESP32 watchdog stops motors.

### Test 8 — Close browser while moving

Expected: robot stops.

### Test 9 — Ctrl+C teleop

Expected: zero `/cmd_vel` on shutdown; motors stop.

### Test 10 — Phase 5/6 regression

```bash
ros2 topic hz /wheel_ticks    # ~50 Hz
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo base_link laser
```

TF tree unchanged; no duplicate `odom` publisher.

## Motor inversion

If a side runs backwards, edit firmware constants in
`firmware/warotrans_low_level/warotrans_low_level.ino`:

```cpp
constexpr bool LEFT_MOTOR_INVERTED = false;
constexpr bool RIGHT_MOTOR_INVERTED = false;
```

Reflash ESP32. Do not expose inversion toggles in the web UI.
