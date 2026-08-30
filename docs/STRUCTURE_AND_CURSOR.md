# WaroTrans — Repo Structure & Controlled Vibe Coding

Tài liệu này tập trung vào **cách tổ chức repo và cách làm việc với AI/Cursor**. Roadmap kỹ thuật đầy đủ nằm ở `MASTER_ENGINEERING_GUIDE.md`.

---

# 1. Cấu trúc đích

```text
warotrans/
├── .cursor/rules/
├── firmware/
│   └── esp32_low_level/
├── ros2_ws/src/
│   ├── warotrans_msgs/
│   ├── warotrans_description/
│   ├── warotrans_hardware/
│   ├── warotrans_localization/
│   ├── warotrans_slam/
│   ├── warotrans_navigation/
│   ├── warotrans_perception/
│   ├── warotrans_fleet/
│   ├── warotrans_simulation/
│   └── warotrans_bringup/
├── backend/
├── web/
├── mobile/
├── docs/
└── tools/
```

## Package ownership

| Package | Lý do thay đổi |
|---|---|
| `warotrans_description` | geometry/URDF/sensor pose đổi |
| `warotrans_hardware` | serial/encoder/odometry/hardware health đổi |
| `warotrans_localization` | EKF/AMCL/localization strategy đổi |
| `warotrans_slam` | mapping config đổi |
| `warotrans_navigation` | Nav2/map/BT đổi |
| `warotrans_perception` | camera/vision đổi |
| `warotrans_fleet` | task/fleet/backend protocol đổi |
| `warotrans_bringup` | cách ghép subsystem/startup đổi |

**Bringup không chứa node business/hardware logic.**

---

# 2. Đừng over-engineer trước map đầu tiên

Hiện tại ưu tiên:

```text
ROS → LiDAR /scan → encoder → /odom → TF → calibration → SLAM → saved map
```

Nếu prototype node hiện đang gộp serial + odometry mà chạy ổn, không cần refactor lớn trước khi có map đầu tiên.

Sau khi map PASS quality gate mới tách:

```text
serial transport → esp32_bridge_node
kinematics       → wheel_odom_node
```

rồi thêm EKF/PID/Nav2.

---

# 3. Source-of-truth cho Cursor

Trước task robotics:

```text
@docs/STATUS.md
@docs/interfaces.md
@docs/calibration.md
```

Task kiến trúc lớn:

```text
@docs/architecture.md
@docs/MASTER_ENGINEERING_GUIDE.md
```

Rule quan trọng:

> AI được sáng tạo implementation, không được sáng tạo hardware truth.

---

# 4. Prompt pattern chuẩn

## Feature

```text
Context:
@docs/STATUS.md
@docs/interfaces.md
@<files>

Goal:
<Một kết quả cụ thể>

Constraints:
- Không đổi TF contract.
- Không đổi physical constants.
- Không thêm dependency nếu chưa giải thích.

Before editing:
Liệt kê file sẽ sửa, file sẽ không sửa, cách test.

Definition of Done:
<runtime evidence cụ thể>
```

## Debug

```text
Symptom:
...

Evidence:
...

Expected:
...

Do not edit code yet.
Rank hypotheses by probability and give one verification command/test for each.
```

## Review trước commit

```text
Review current diff as a senior robotics engineer.
Only report issues, ordered by severity.

Check:
- duplicate TF publishers
- unmeasured physical constants
- blocking I/O in ROS callbacks
- missing device reconnect/error handling
- watchdog/safe-stop regressions
- breaking ROS/serial interfaces
- unsafe deploy/runtime file deletion
```

---

# 5. Ví dụ: không biến số minh họa thành measurement

Sai:

```text
Tạo Nav2 footprint 0.32 × 0.26, inflation 0.25, resolution 0.03
```

nếu các số cuối chưa được chọn/đo.

Đúng:

```text
Chassis outer size đã VERIFIED 0.32 × 0.26 m.
Hãy tạo footprint từ geometry thật.
Các tham số inflation/resolution chưa được measured/tuned: đề xuất baseline có giải thích,
đánh dấu TUNING START VALUE, không ghi vào calibration như truth.
```

Phân biệt:

- **Physical measurement:** không bịa.
- **Algorithm tuning start value:** có thể đề xuất, nhưng phải đánh dấu là baseline để test, không phải truth.

---

# 6. Cursor usage policy

### Nên dùng mạnh

- boilerplate ROS launch/config;
- docs/test/tooling;
- backend/web/mobile;
- pure math/unit tests;
- refactor nhỏ có test.

### Cần review kỹ

- ROS nodes;
- firmware;
- TF;
- serial reconnect;
- concurrency/timing.

### Không giao quyền tự quyết

- điện áp/dòng/wiring;
- GPIO;
- calibration;
- watchdog;
- SLAM/Nav2 tuning cuối cùng;
- OS/distro migration;
- fleet protocol architecture.

---

# 7. Git workflow

```text
main
feature/lidar
feature/odom
feature/slam
feature/nav2
fix/serial-reconnect
```

Trước Agent edit:

```bash
git status
git add ...
git commit -m "checkpoint: known-good baseline"
```

Sau edit:

```bash
git diff
./tools/check.sh
```

Rồi mới deploy/test robot thật.

---

# 8. Definition of Done mẫu

### LiDAR

```text
/dev/rplidar ổn định
/scan tồn tại
ros2 topic hz /scan ổn định
ranges hợp lệ
visualization đúng hướng
```

### Odometry

```text
/odom ổn định
TF đúng
2 m straight test đạt tolerance đã chọn
360° rotation test đạt tolerance đã chọn
```

### SLAM

```text
/map publish
loop closure hợp lý
không double wall rõ
map save + reload được
```

### Nav2

```text
localization ổn
NavigateToPose tới goal
obstacle response an toàn
/cmd_vel → ESP32 → motor closed-loop
```

Build success chỉ là một phần của DoD.
