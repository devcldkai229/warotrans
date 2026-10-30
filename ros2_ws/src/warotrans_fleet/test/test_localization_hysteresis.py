from warotrans_fleet.localization_hysteresis import LocalizationHysteresis


def test_holds_localized_through_brief_stale_blips():
    h = LocalizationHysteresis(stale_limit=3)
    pose = {"x": 1.0, "y": 2.0, "yaw": 0.1}
    assert h.on_good(pose) == (pose, "LOCALIZED")

    p1, s1 = h.on_stale()
    p2, s2 = h.on_stale()
    assert s1 == "LOCALIZED" and p1 == pose
    assert s2 == "LOCALIZED" and p2 == pose

    p3, s3 = h.on_stale()
    assert s3 == "LOST" and p3 is None


def test_unknown_until_first_good_pose():
    h = LocalizationHysteresis(stale_limit=2)
    p, s = h.on_stale()
    assert s == "UNKNOWN" and p is None
    p2, s2 = h.on_stale()
    assert s2 == "UNKNOWN"


def test_recovery_resets_stale_count():
    h = LocalizationHysteresis(stale_limit=3)
    pose = {"x": 0.0, "y": 0.0, "yaw": 0.0}
    h.on_good(pose)
    h.on_stale()
    h.on_stale()
    h.on_good({"x": 1.0, "y": 0.0, "yaw": 0.0})
    p, s = h.on_stale()
    assert s == "LOCALIZED" and p == {"x": 1.0, "y": 0.0, "yaw": 0.0}
