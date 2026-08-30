# WaroTrans — Architecture Boundaries

## 1. Dependency direction

```text
                 fleet / web / mobile
                         │
                         ▼
                  high-level goals
                         │
                         ▼
navigation ─────► localization ◄──── perception
    │                    ▲
    │                    │
    ▼                    │
/cmd_vel              /odom, /imu
    │                    │
    └──────────► hardware/base
                         │
                         ▼
                       ESP32
                         │
                         ▼
                       MDD10A
```

Phần trên **không được biết GPIO/PWM chi tiết**.

## 2. ROS package ownership

| Package | Trách nhiệm |
|---|---|
| `warotrans_msgs` | msg/srv/action custom nếu thật sự cần |
| `warotrans_description` | URDF/xacro/mesh/static geometry |
| `warotrans_hardware` | ESP32 bridge, encoder, wheel odometry, hardware diagnostics |
| `warotrans_sensors` | Sensor driver layer: LiDAR now, IMU later. Publishes raw topics only |
| `warotrans_localization` | EKF/AMCL/localization configuration |
| `warotrans_slam` | SLAM Toolbox mapping configuration |
| `warotrans_navigation` | Nav2 config, map release, behavior trees |
| `warotrans_perception` | camera/vision |
| `warotrans_fleet` | task FSM + backend bridge + protocol adapters |
| `warotrans_simulation` | simulation only |
| `warotrans_bringup` | orchestration: launch/config include, **không chứa domain node** |

## 3. Migration từ prototype

### Trước map đầu tiên

Giữ implementation nhỏ nhất để đạt:

```text
/scan + /odom + TF → SLAM → saved map
```

Nếu `wheel_odom_node` hiện vừa serial vừa odometry thì **chưa cần refactor chỉ để đẹp**.

### Sau map đầu tiên đạt quality gate

Tách dần:

```text
ESP32 serial
  ↓
esp32_bridge_node
  ↓
encoder/tick data
  ↓
wheel_odom_node
```

Sau đó mới thêm:

```text
BNO055 → EKF
PID → Nav2
Camera
Fleet
```

## 3b. Calibration ownership

```text
docs/calibration.md
   human log: date, method, conditions, evidence
        │
        ▼
warotrans_description/config/calibration.yaml
   machine value read by BOTH the URDF and node parameters
        │
        ├──► URDF via xacro.load_yaml
        └──► node parameters via launch
```

A value enters `calibration.yaml` only after it has a record in
`docs/calibration.md`. No component hardcodes a value that lives there.
`null` means not measured; a consumer pointed at `null` fails loudly, which is
correct.

## 3c. Package boundaries — chi tiết

### `warotrans_description`
Owns robot geometry. It does **not** read serial or publish odometry.

### `warotrans_hardware`
Owns low-level robot I/O and odometry. It does **not** own SLAM/Nav2 configuration.

### `warotrans_sensors`
Owns sensor **drivers** and driver configuration: LiDAR now, IMU later.
It publishes raw sensor topics and nothing else. It does **not** estimate state,
does **not** build maps, and does **not** command motors.

The split against `warotrans_hardware` is: **`hardware` moves the robot,
`sensors` observes the world.** Keeping passive drivers out of `hardware` keeps
the future `ros2_control` migration contained to one package.

### `warotrans_localization`
Owns state-estimation/localization configuration. It does **not** command motors.

### `warotrans_slam`
Owns mapping configuration. It does **not** parse encoders.

### `warotrans_navigation`
Owns Nav2 configuration. It outputs velocity commands through the agreed ROS contract.

### `warotrans_perception`
Owns camera/perception. Camera is not a required dependency for 2D LiDAR SLAM.

### `warotrans_fleet`
Owns high-level task/status integration. It must not send PWM or depend on motor GPIO.

### `warotrans_bringup`
Owns orchestration only.

## 3d. No invented hardware facts

Never invent:
- encoder pinout;
- sensor voltage;
- ticks/rev;
- wheel radius;
- effective wheel separation;
- LiDAR/camera pose;
- camera model;
- motor polarity.

Measure or verify first.

## 4. Architectural invariants

- Một TF edge = một publisher.
- Backend không gửi PWM.
- Nav2 không biết GPIO.
- Physical constants là parameters + calibration evidence, và chỉ tồn tại ở MỘT file máy đọc được.
- Sensor driver và state estimation nằm ở hai package khác nhau.
- Runtime data (`maps`, `bags`, logs) không bị deploy source xóa.
- Hardware safety không phụ thuộc vào network ổn định.
- Multi-robot chỉ bắt đầu sau khi single-robot end-to-end PASS.
