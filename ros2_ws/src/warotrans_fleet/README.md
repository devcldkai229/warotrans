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

Commands: `NAVIGATE_TO_POSE`, `CANCEL` only. Robot never sets business `RobotStatus`. Backend owns `IsOnline` and operational status.

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
PYTHONPATH=. python3 -m pytest test -q
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
