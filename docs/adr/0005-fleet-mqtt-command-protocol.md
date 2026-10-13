# ADR 0005 — Fleet MQTT command protocol (Backend ↔ Robot)

**Status:** Accepted  
**Date:** 2026-10-07

## Context

WaroTrans đã có uplink MQTT (`heartbeat` / `telemetry`). Cần downlink chính thức để backend điều khiển robot (dispatch) mà không khóa vào VDA5050, và không để robot tự quyết business `RobotStatus`.

## Decision

- Transport: MQTT topics dưới `warotrans/v1/robots/{robotCode}/…`
- QoS: `command` / `command_ack` / `command_result` = QoS1; `heartbeat` / `telemetry` giữ QoS0
- Topics:
  - `…/command` (backend → robot)
  - `…/command_ack` (robot → backend)
  - `…/command_result` (robot → backend)
- Command types (chỉ các loại này cho đến khi có use case mới):
  - `NAVIGATE_TO_POSE`
  - `CANCEL`
- `commandId` (Guid) do backend cấp; robot echo trong ack/result và `telemetry.currentCommandId`
- Robot báo technical state (localization/navigation/ACK/result). Backend/Fleet là SoT của `Robot.Status` và `JobAssignment`
- Reject navigate khi không `LOCALIZED`, hoặc đang có command active khác
- Cancel chỉ áp dụng cho `targetCommandId` đang active
- Heartbeat timeout khi command đang chạy → backend abort command (`ABORTED`), end assignment `ROBOT_OFFLINE`, clear `CurrentCommandId`
- Shared payload SoT: `warotrans-system/docs/mqtt-robot-contracts.md`

## Alternatives considered

- VDA5050 — quá nặng / khóa vendor; chưa cần
- HTTP robot API — không khớp boundary MQTT đã chọn trong kiến trúc hệ thống
- Chỉ telemetry polling, không ACK/result — thiếu correlation và idempotency

## Consequences

### Positive

- Correlation rõ bằng `commandId`
- Tách technical vs business ownership
- Mở rộng thêm command type sau này mà không đổi topic shape

### Negative / trade-offs

- Cần bảng `fleet.robot_commands` để timeout/idempotency
- QoS1 tăng tải broker nhẹ so với uplink

## Validation

- Unit + MQTT broker integration tests (system + robot)
- Manual: Host publish navigate → robot ACK → Nav2 → result → DB `RobotCommand` / `JobAssignment` / `CurrentCommandId`
