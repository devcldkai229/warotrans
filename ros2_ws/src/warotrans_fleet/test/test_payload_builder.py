from warotrans_fleet.nav2_status_tracker import Nav2StatusTracker
from warotrans_fleet.payload_builder import build_heartbeat, build_telemetry


def test_heartbeat_schema():
    payload = build_heartbeat(
        robot_code="RBT-001",
        boot_id="boot-uuid",
        sequence=3,
        sent_at="2026-10-07T05:00:00.000Z",
        message_id="msg-1",
    )
    assert payload["schemaVersion"] == 1
    assert payload["messageId"] == "msg-1"
    assert payload["robotCode"] == "RBT-001"
    assert payload["bootId"] == "boot-uuid"
    assert payload["sequence"] == 3
    assert payload["sentAt"] == "2026-10-07T05:00:00.000Z"


def test_telemetry_null_battery_and_null_pose_when_lost():
    payload = build_telemetry(
        robot_code="RBT-001",
        boot_id="boot-uuid",
        sequence=1,
        map_version_code="MAP-1",
        pose=None,
        battery_percent=None,
        navigation_status="IDLE",
        localization_status="LOST",
        linear_velocity=0.0,
        angular_velocity=0.0,
        message_id="msg-2",
        sent_at="2026-10-07T05:00:00.000Z",
    )
    assert payload["pose"] is None
    assert payload["batteryPercent"] is None
    assert payload["localizationStatus"] == "LOST"
    assert payload["navigationStatus"] == "IDLE"


def test_telemetry_localized_pose():
    payload = build_telemetry(
        robot_code="RBT-001",
        boot_id="boot",
        sequence=2,
        map_version_code=None,
        pose={"x": 1.5, "y": 2.5, "yaw": 0.3},
        battery_percent=None,
        navigation_status="NAVIGATING",
        localization_status="LOCALIZED",
        linear_velocity=0.4,
        angular_velocity=0.1,
    )
    assert payload["pose"] == {"x": 1.5, "y": 2.5, "yaw": 0.3}
    assert payload["linearVelocity"] == 0.4


def test_nav2_status_mapping():
    tracker = Nav2StatusTracker()
    assert tracker.map_goal_status(2) == "NAVIGATING"
    assert tracker.map_goal_status(4) == "SUCCEEDED"
    assert tracker.map_goal_status(6) == "FAILED"
    assert tracker.map_goal_status(5) == "CANCELED"
    assert tracker.update_from_goal_status(2) == "NAVIGATING"
    assert tracker.navigation_status == "NAVIGATING"
