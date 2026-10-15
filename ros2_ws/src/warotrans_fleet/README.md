# warotrans_fleet

Fleet / backend bridge. **Robot Gateway** — bidirectional MQTT with `warotrans-system` (heartbeat/telemetry uplink + command downlink).

Protocol ADR: `docs/adr/0005-fleet-mqtt-command-protocol.md`.

## Shared MQTT contract

See `warotrans-system/docs/mqtt-robot-contracts.md`.

| Topic | Direction | QoS |
|-------|-----------|-----|
| `…/heartbeat` | robot → backend (~1 Hz) | 0 |
| `…/telemetry` | robot → backend (~5 Hz) | 0 |
| `…/command` | backend → robot | 1 |
| `…/command_ack` | robot → backend | 1 |
| `…/command_result` | robot → backend | 1 |
| `…/teleop` | backend → robot (manual drive, ~5 Hz while a button is held) | 0 |

Commands: `NAVIGATE_TO_POSE`, `CANCEL` only. Robot never sets business `RobotStatus`. Backend owns `IsOnline` and operational status.

**Teleop** (`warotrans_fleet/teleop.py`, không phải command, không ACK/RESULT): `{linear, angular}` → `geometry_msgs/Twist`
trên `cmd_vel_topic`. Kẹp theo `teleop_max_linear` / `teleop_max_angular` (robot thật 0.10 m/s, 0.45 rad/s).
Không nhận lệnh trong `teleop_deadman_sec` (0.5 s) → tự gửi Twist 0. Bỏ qua khi đang có lệnh navigate / goal Nav2.
Tắt hẳn: `teleop_enabled: false`.

**Drive Assist** (`warotrans_fleet/drive_assist.py`, teleop `assist`, mặc định bật): `/scan` → `base_link` (TF; lidar
gắn yaw +pi/2) → hành lang rộng bằng thân xe theo hướng chạy → giảm tốc dưới `assist_slow_gap`, dừng dưới
`assist_stop_gap`. Không có scan mới → không tiến/lùi (quay tại chỗ vẫn được).

Robot chỉ nhận "đi tới pose này" — không biết TransportRequest / Job / Workflow. Backend resolve Endpoint → `x/y/yaw` trước khi gửi.

## ROS sources

| Field | Source |
|-------|--------|
| Pose | TF `map` → `base_link` (params `map_frame` / `base_frame`) |
| Velocity | `/odom` twist |
| NavigationStatus | Nav2 `navigate_to_pose` action status |
| LocalizationStatus | Valid recent TF → `LOCALIZED`; else `LOST` / `UNKNOWN` |
| Battery | `null` (no sensor in current stack) |

When localization is not `LOCALIZED`, telemetry sends `"pose": null` (never a stale fake pose).

## Config

[`config/robot_gateway.yaml`](config/robot_gateway.yaml) plus env overrides:

| Env | Param |
|-----|--------|
| `WARO_ROBOT_CODE` | `robot_code` |
| `WARO_MQTT_HOST` | `mqtt_host` |
| `WARO_MQTT_PORT` | `mqtt_port` |
| `WARO_MAP_VERSION_CODE` | `map_version_code` |

## Launch

```bash
# With navigation stack already running (AMCL + TF map→…→base_link):
ros2 launch warotrans_bringup navigation.launch.py map:=$HOME/maps/warotrans.yaml

export WARO_ROBOT_CODE=RBT-001
export WARO_MQTT_HOST=localhost
export WARO_MQTT_PORT=1883
export WARO_MAP_VERSION_CODE=MAP-WH001-V001

ros2 launch warotrans_fleet fleet.launch.py
```

## Developer MQTT sniff

```bash
# From package share or source tree:
bash src/warotrans_fleet/scripts/mqtt_sniff.sh localhost 1883 RBT-001
```

Expect heartbeat ~1/s and telemetry ~5/s.

## Unit tests (no ROS required)

```bash
cd ros2_ws/src/warotrans_fleet
PYTHONPATH=. python3 -m pytest test -q          # test ROS integration tự skip
```

Integration (gateway node thật + mock Nav2 action server, cần Jazzy + `ros-jazzy-nav2-msgs`):

```bash
source /opt/ros/jazzy/setup.bash
PYTHONPATH=.:$PYTHONPATH python3 -m pytest test -v   # giữ PYTHONPATH của ROS
```

## E2E: Robot Gateway → MQTT → warotrans-system → API / SignalR

1. Start Mosquitto (e.g. `warotrans-system/infrastructure` compose `mosquitto` on port 1883).
2. Start Postgres/Mongo + `WaroTrans.Host` (Development applies migrations; MQTT subscriber connects).
3. Register robot via `POST /api/fleet/robots` (note returned `code`, e.g. `RBT-001`).
4. Launch robot nav stack + `warotrans_fleet` with matching `WARO_ROBOT_CODE`.
5. Confirm broker traffic with `mqtt_sniff.sh`.
6. `GET /api/fleet/robots/{id}` → `isOnline: true`, pose/nav/localization from telemetry.
7. Stop gateway → after heartbeat timeout (~5 s) → `isOnline: false` (business `status` unchanged).
8. UI / Live Map: subscribe SignalR hub `/hubs/notifications` events `robotConnectivityChanged`, `robotTelemetryUpdated`.

## MOVE command (NAVIGATE_TO_POSE / CANCEL)

### Kiến trúc

```text
MQTT thread (paho)          ROS executor thread (rclpy.spin)
──────────────────          ───────────────────────────────────────────────
on_message                   _drain_inbox (guard condition)
  parse JSON                   └─ CommandRouter.handle_message
  inbox.put + trigger ───────►      ├─ validate envelope (schemaVersion/robotCode/type/payload)
  (không block, không rclpy)        ├─ dedup theo commandId (ledger)
                                    ├─ NAVIGATE_TO_POSE → MoveCommandHandler
                                    └─ CANCEL           → CancelCommandHandler → MoveCommandHandler
                               Nav2ActionNavigator (ActionClient navigate_to_pose, futures async)
                               timer: router.tick (timeout / Nav2 không phản hồi / hold status)
                               timer: heartbeat, telemetry
```

| File | Vai trò |
|------|---------|
| `command_contract.py` | Hằng số contract + parse/validate envelope và payload |
| `command_router.py` | `CommandRouter`, `CommandSession`, `AckDecision`, `CommandHandler` protocol |
| `move_command_handler.py` | `MoveCommandHandler` (goal lifecycle, cancel, timeout), `CancelCommandHandler` |
| `navigator.py` | Port tới Nav2 (`NavigatorPort`) — fake được trong test |
| `nav2_navigator.py` | Implementation rclpy `ActionClient(NavigateToPose)` |
| `navigation_state.py` | `navigationStatus` IDLE/NAVIGATING/SUCCEEDED/FAILED/CANCELED |
| `command_stack.py` | Wiring router + handler (node và test dùng chung) |

Thêm step type sau này (CHECK / WAIT nếu chạy trên robot): viết class có `command_type` + `handle(command, session) -> AckDecision` (tuỳ chọn `tick()`), register trong `command_stack.py`. HUMAN_INTERACTION thuộc Backend + Staff App.

### Contract thực tế

Command (backend → robot, QoS1):

```json
{
  "schemaVersion": 1, "messageId": "uuid", "commandId": "uuid", "robotCode": "RBT-001",
  "type": "NAVIGATE_TO_POSE", "issuedAt": "2026-10-13T08:00:00.000Z",
  "jobAssignmentId": "uuid|null", "jobStepId": "uuid|null",
  "payload": { "frameId": "map", "x": 4.25, "y": 2.10, "yaw": 1.5708,
               "timeoutSeconds": 300, "mapVersionCode": "MAP-V1" }
}
```

`timeoutSeconds`, `mapVersionCode` là **optional** (mở rộng tương thích). `frameId` phải là `map_frame` (mặc định `map`).

ACK (`…/command_ack`, QoS1) — "đã nhận và chấp nhận xử lý", **không** phải "đã tới nơi":

```json
{ "schemaVersion": 1, "messageId": "uuid", "commandId": "same", "robotCode": "RBT-001",
  "accepted": true, "reasonCode": null, "reasonMessage": null,
  "jobAssignmentId": "…", "jobStepId": "…", "sentAt": "…" }
```

RESULT (`…/command_result`, QoS1) — chỉ gửi khi Nav2 trả kết quả thật:

```json
{ "schemaVersion": 1, "messageId": "uuid", "commandId": "same", "robotCode": "RBT-001",
  "outcome": "SUCCEEDED", "errorCode": null, "errorMessage": null,
  "jobAssignmentId": "…", "jobStepId": "…",
  "result": { "navigationResult": "SUCCEEDED", "nav2Status": 4, "nav2ErrorCode": 0, "elapsedSec": 12.3 },
  "sentAt": "…" }
```

`outcome` chỉ có `SUCCEEDED | FAILED | CANCELED` (backend validator). Timeout = `outcome: FAILED` + `errorCode: NAVIGATION_TIMEOUT` + `result.navigationResult: TIMEOUT`.

### Mã lỗi

| Khi | Field | Code |
|-----|-------|------|
| JSON/field sai, frame sai, số không hữu hạn | ACK `reasonCode` | `INVALID_PAYLOAD` |
| `robotCode` khác robot | ACK | `WRONG_ROBOT` |
| `type` khác NAVIGATE_TO_POSE/CANCEL | ACK | `UNSUPPORTED_COMMAND` |
| Đang có MOVE active, hoặc Nav2 đang chạy goal ngoài fleet (RViz) | ACK | `BUSY` |
| `localizationStatus != LOCALIZED` | ACK | `NOT_LOCALIZED` |
| `mapVersionCode` khác `map_version_code` của robot | ACK | `MAP_MISMATCH` |
| Action server chưa sẵn sàng | ACK | `NAV2_UNAVAILABLE` |
| CANCEL target không phải command active | ACK | `UNKNOWN_TARGET` |
| Exception trong handler | ACK | `INTERNAL_ERROR` |
| commandId trùng (QoS1) | ACK accepted=true | `ALREADY_ACCEPTED` |
| Nav2 từ chối goal | RESULT FAILED | `GOAL_REJECTED` |
| Nav2 ABORTED | RESULT FAILED | `NAVIGATION_FAILED` |
| Quá `timeoutSeconds` (đã cancel Nav2) | RESULT FAILED | `NAVIGATION_TIMEOUT` |
| Bị cancel (backend / RViz) | RESULT CANCELED | `NAVIGATION_CANCELED` |
| Nav2 không trả lời goal request | RESULT FAILED | `NAV2_UNAVAILABLE` |
| Đã cancel nhưng Nav2 không kết thúc goal | RESULT FAILED | `NAV2_CANCEL_UNCONFIRMED` |
| CANCEL command: Nav2 từ chối cancel / goal kết thúc trước | RESULT FAILED (của CANCEL) | `CANCEL_REJECTED` / `CANCEL_INEFFECTIVE` |

### Hành vi

- Một MOVE active tại một thời điểm. MOVE mới khi đang chạy → `BUSY`, **không** tự cancel goal cũ.
- Duplicate `commandId`: không bao giờ tạo goal Nav2 thứ hai. Đang chạy → ACK `ALREADY_ACCEPTED`; đã xong → ACK `ALREADY_ACCEPTED` + gửi lại RESULT đã cache; đã reject → gửi lại đúng ACK reject.
- CANCEL / timeout: `cancel_goal_async` → chờ Nav2 kết thúc goal → RESULT. Nếu Nav2 từ chối cancel, robot vẫn chạy và RESULT của navigate phản ánh kết quả thật. Thứ tự khi CANCEL thành công: RESULT navigate `CANCELED` rồi RESULT của CANCEL `SUCCEEDED`.
- `navigationStatus`: `IDLE → NAVIGATING → SUCCEEDED|FAILED|CANCELED → (sau terminal_status_hold_sec) IDLE`. `currentCommandId` = command đang active.
- Feedback Nav2 không đi ra `command_result`; chỉ log `[NavigationProgress]` mỗi ~5 s và phản ánh qua telemetry.

### Demo trên robot thật

```bash
# Pi — terminal 1: Nav2 stack (Phase 0) + 2D Pose Estimate trong RViz để AMCL LOCALIZED
ros2 launch warotrans_bringup navigation.launch.py map:=$HOME/maps/warotrans.yaml

# Pi — terminal 2: gateway
export WARO_ROBOT_CODE=RBT-001 WARO_MQTT_HOST=<broker> WARO_MQTT_PORT=1883
ros2 launch warotrans_fleet fleet.launch.py

# Laptop / Pi — terminal 3: đóng vai Backend (hoặc dùng warotrans-system Host)
python3 src/warotrans_fleet/scripts/fleet_command_cli.py --host <broker> --robot RBT-001 \
    navigate --x 1.0 --y 0.5 --yaw 0.0 --timeout 120
# Ctrl+C khi robot đang chạy → gửi CANCEL cho command đó
```

Chọn `x/y/yaw` trong vùng free của map (đọc từ RViz "Publish Point" hoặc pose hiện tại trên telemetry).
Log gateway mong đợi theo thứ tự:

```text
[CommandReceived] → [CommandAccepted] → [Nav2GoalSent] → [Nav2GoalAccepted] → [NavigationStarted]
  → [NavigationProgress]… → [NavigationSucceeded] → [CommandResultPublished]
```

Log không chứa credential (MQTT hiện không dùng user/password).
