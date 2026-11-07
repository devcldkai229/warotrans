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


from types import SimpleNamespace

from warotrans_fleet.battery_telemetry import battery_fields, unavailable_fields


def _telemetry(battery):
    return build_telemetry(
        robot_code="RBT-001", boot_id="b", sequence=1, map_version_code="MAP-1", pose=None,
        battery_percent=None, navigation_status="IDLE", localization_status="LOST",
        linear_velocity=0.0, angular_velocity=0.0, message_id="m", sent_at="2026-11-06T00:00:00.000Z",
        battery=battery,
    )


def _estimate(**kw):
    base = dict(mode="VOLTAGE_ONLY", confidence="MEDIUM", charging_state="UNKNOWN", percentage=0.634, voltage=11.4321)
    base.update(kw)
    return SimpleNamespace(**base)


def test_battery_unavailable_is_null_never_zero():
    payload = _telemetry(battery_fields(simulated_percent=None))
    assert payload["batteryPercent"] is None
    assert payload["batteryVoltage"] is None
    assert payload["batteryEstimationMode"] == "UNAVAILABLE"
    assert payload["batteryConfidence"] == "NONE"
    assert payload["batteryChargingState"] == "UNKNOWN"


def test_battery_fresh_voltage_only_estimate():
    fields = battery_fields(simulated_percent=None, estimate=_estimate(), estimate_age_s=0.4,
                            measured_at="2026-11-06T00:00:00.000Z")
    payload = _telemetry(fields)
    assert payload["batteryPercent"] == 63.4
    assert payload["batteryVoltage"] == 11.432
    assert payload["batteryEstimationMode"] == "VOLTAGE_ONLY"
    assert payload["batteryConfidence"] == "MEDIUM"
    assert payload["batteryChargingState"] == "UNKNOWN"
    assert payload["batteryMeasuredAt"] == "2026-11-06T00:00:00.000Z"


def test_battery_stale_or_unavailable_estimate_is_null():
    assert battery_fields(simulated_percent=None, estimate=_estimate(), estimate_age_s=5.0,
                          stale_after_s=3.0) == unavailable_fields()
    assert battery_fields(simulated_percent=None, estimate=_estimate(mode="UNAVAILABLE"),
                          estimate_age_s=0.1) == unavailable_fields()
    assert battery_fields(simulated_percent=None, estimate=_estimate(percentage=float("nan")),
                          estimate_age_s=0.1) == unavailable_fields()


def test_battery_simulated_value_is_marked_simulated():
    fields = battery_fields(simulated_percent=87.0, estimate=None)
    assert fields["batteryPercent"] == 87.0
    assert fields["batteryEstimationMode"] == "SIMULATED"


def test_legacy_battery_percent_kept_when_no_battery_fields():
    payload = build_telemetry(
        robot_code="RBT-001", boot_id="b", sequence=1, map_version_code=None, pose=None,
        battery_percent=55.0, navigation_status="IDLE", localization_status="LOST",
        linear_velocity=0.0, angular_velocity=0.0,
    )
    assert payload["batteryPercent"] == 55.0
    assert "batteryEstimationMode" not in payload
