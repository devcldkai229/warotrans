# ADR 0006 — Simulation stack: Gazebo Harmonic + ros_gz, trên WSL2

**Status:** Accepted
**Date:** 2026-10-15

## Context

Backend WaroTrans cần một robot để chạy thử workflow (MOVE → xác nhận nhân viên → MOVE …) mà
không phụ thuộc robot thật (pin, phần cứng, mặt bằng). Package `warotrans_simulation` đã để trống
và yêu cầu chọn simulator bằng ADR.

Ràng buộc:
- ROS 2 **Jazzy** (ADR robot, Ubuntu 24.04).
- Backend chỉ nói chuyện với robot qua **MQTT contract** (ADR 0005). Robot ảo phải nói đúng contract
  đó, để backend không phân biệt thật/ảo.
- Máy dev là Windows 11 → chạy trong **WSL2** (WSLg hiển thị cửa sổ).
- Không được làm thay đổi hành vi của robot thật (URDF, launch, params của Pi).

## Decision

- Simulator: **Gazebo Harmonic** (`gz sim`), cầu nối ROS ↔ Gazebo bằng **`ros_gz_bridge`**,
  là cặp chính thức của ROS 2 Jazzy (`ros-jazzy-ros-gz`).
- Mô hình sim **include URDF thật** (`warotrans_description`) và chỉ thêm phần sim
  (`warotrans_simulation/urdf/warotrans_sim.urdf.xacro`): **4 bánh skid-steer** như khung thật, quán tính,
  `gpu_lidar`, `DiffDrive` (2 khớp mỗi bên). Bán kính bánh, wheelbase và **wheel_separation hiệu dụng** đọc
  thẳng từ `warotrans_description/config/calibration.yaml` của robot thật.
- Gazebo thay thế đúng các node phần cứng: `esp32_bridge_node` (/cmd_vel), `wheel_odom_node`
  (/odom, TF odom→base_footprint), driver RPLIDAR (/scan). Tên topic và frame giữ nguyên.
- AMCL, Nav2 và `robot_gateway_node` là **cùng code, cùng params** với robot thật. Chỉ thêm overlay
  sim (`use_sim_time`, tốc độ demo, `robot_code: RBT-SIM-01`).
- World, map Nav2 và toạ độ endpoint được **sinh từ một file layout**
  (`config/warehouse_layout.yaml` → `scripts/generate_warehouse.py`), để Gazebo, map và backend luôn
  cùng hệ toạ độ.

## Alternatives considered

- **Gazebo Classic 11**: hết hỗ trợ, không có bản Jazzy.
- **Webots / Isaac Sim**: nặng hoặc không có sẵn trên WSL2. Bridge ROS 2 của chúng không phải mặc định của Jazzy.
- **Robot giả chỉ bằng MQTT** (script nội suy pose, không ROS): nhẹ, nhưng không chạy Nav2/AMCL/gateway
  thật, nên không kiểm chứng được stack sẽ chạy trên Pi. Có thể thêm sau cho CI backend.

## Consequences

### Positive
- Sim-to-real: chuyển sang robot thật chỉ đổi launch (hardware thay Gazebo), map và `robot_code`.
- Backend/web không cần code riêng cho sim.
- Layout là nguồn duy nhất → không lệch toạ độ giữa world, map và endpoint.

### Negative / trade-offs
- Sim là skid-steer 4 bánh, nhưng độ trượt bánh khi quay do ma sát Gazebo (`mu2 = 0.4`) quyết định, không
  đo từ mặt sàn thật. Không dùng sim để tune `wheel_separation` / PID. Hướng ngược lại thì được:
  sim dùng giá trị đã hiệu chỉnh của robot thật.
- Tốc độ demo trong sim (0.4 m/s) cao hơn giới hạn tạm của robot thật (0.10 m/s).
- WSL2 + WSLg: render `ogre2` qua GPU ảo, có thể phải dùng software rendering trên một số máy.

## Validation

`ros2 launch warotrans_simulation sim_bringup.launch.py`, sau đó
`fleet_command_cli.py navigate x y yaw` qua Mosquitto → nhận ACK → robot ảo chạy tới pose trong Gazebo
→ RESULT `SUCCEEDED`. CANCEL giữa đường thì robot dừng và nhận RESULT `CANCELED`.
Chi tiết: `docs/simulation/`.
