"""RB-10 health snapshot and the RB-03 / RB-09 payload fields."""

from warotrans_fleet import health_monitor as hm
from warotrans_fleet.payload_builder import build_heartbeat, build_telemetry


def test_a_required_sensor_that_goes_silent_is_an_error_an_optional_one_stale():
    lidar = hm.TopicCheck("lidar", "/scan", 5.0)
    imu = hm.TopicCheck("imu", "/imu/data", 20.0, required=False)
    for i in range(30):
        lidar.record(100.0 + i * 0.1)
    assert lidar.evaluate(103.0)["level"] == hm.OK

    assert lidar.evaluate(110.0)["level"] == hm.ERROR
    assert imu.evaluate(110.0)["level"] == hm.UNKNOWN
    imu.record(100.0)
    assert imu.evaluate(110.0)["level"] == hm.STALE


def test_a_slow_sensor_is_a_warning_with_its_rate():
    lidar = hm.TopicCheck("lidar", "/scan", 5.0)
    for i in range(5):
        lidar.record(100.0 + i)
    item = lidar.evaluate(104.5)
    assert item["level"] == hm.WARN and item["value"] == 1.0


def test_unmeasurable_values_are_unknown_never_invented():
    assert hm.cpu_load_item(lambda: None)["level"] == hm.UNKNOWN
    assert hm.temperature_item(lambda: None)["value"] is None
    assert hm.memory_item(lambda: None)["level"] == hm.UNKNOWN
    assert hm.tf_item(None, 0.5)["level"] == hm.ERROR
    assert hm.nav2_item(None)["level"] == hm.UNKNOWN


def test_thresholds():
    assert hm.cpu_load_item(lambda: 3.9, cpu_count=4)["level"] == hm.ERROR
    assert hm.temperature_item(lambda: 75.0)["level"] == hm.WARN
    assert hm.memory_item(lambda: 50.0)["level"] == hm.OK
    assert hm.tf_item(0.9, 0.5)["level"] == hm.STALE


def test_topic_specs_parse_and_skip_bad_ones():
    checks = hm.parse_topic_checks(["lidar:/scan:sensor_msgs/msg/LaserScan:5", "bad", "imu:/imu:sensor_msgs/msg/Imu:x",
                                    "us:/us:sensor_msgs/msg/Range:5:optional"])
    assert [(c.name, c.required, t) for c, t in checks] == [
        ("lidar", True, "sensor_msgs/msg/LaserScan"), ("us", False, "sensor_msgs/msg/Range")]


def test_heartbeat_announces_capabilities_and_telemetry_the_hold():
    hb = build_heartbeat(robot_code="RBT-1", boot_id="b", sequence=1, capabilities=["RESTART_STACK"])
    assert hb["capabilities"] == ["RESTART_STACK"]
    assert "capabilities" not in build_heartbeat(robot_code="RBT-1", boot_id="b", sequence=1)

    tel = build_telemetry(robot_code="RBT-1", boot_id="b", sequence=1, map_version_code=None, pose=None,
                          battery_percent=None, navigation_status="IDLE", localization_status="LOCALIZED",
                          linear_velocity=0.0, angular_velocity=0.0, motion_hold=True)
    assert tel["motionHold"] is True
