from warotrans_evidence_agent.sampling import SilenceWatch


def test_silence_reported_once_then_recovered():
    w = SilenceWatch("lidar", limit_s=5.0)
    assert w.check(100.0) is None          # never heard: not a stall
    assert w.heard(100.0) is None
    assert w.check(104.0) is None
    stale = w.check(106.0)
    assert stale == {"sensor": "lidar", "silentS": 6.0, "limitS": 5.0, "declared": True}
    assert w.check(110.0) is None          # once only
    back = w.heard(120.0)
    assert back == {"sensor": "lidar", "silentS": 20.0, "declared": True}
    assert w.heard(121.0) is None


def test_gate_events_on_change_and_while_closed():
    from warotrans_evidence_agent.sampling import GateWatch

    g = GateWatch(repeat_s=10.0)
    assert g.sample(None, None, 0.0) is None
    assert g.sample(True, "OK", 0.0) == {"open": True, "reason": "OK", "changed": True}
    assert g.sample(True, "OK", 30.0) is None                     # open: only changes
    closed = g.sample(False, "OUTSIDE_RESERVATION", 31.0)
    assert closed == {"open": False, "reason": "OUTSIDE_RESERVATION", "changed": True}
    assert g.sample(False, "OUTSIDE_RESERVATION", 35.0) is None
    assert g.sample(False, "OUTSIDE_RESERVATION", 41.0)["changed"] is False   # still closed: repeated


def test_nav_goal_payload_pairs_the_goal_with_the_step_endpoint():
    from warotrans_evidence_agent.sampling import nav_goal_payload
    cmd = {"commandId": "c1", "endpoint": {"x": 1.0, "y": 2.0, "frame": "map"},
           "nav2Goal": {"x": 4.0, "y": 6.0, "yaw": 0.1}}
    p = nav_goal_payload("ab", cmd)
    assert p["goal"] == {"x": 4.0, "y": 6.0, "yaw": 0.1} and p["endpoint"] == {"x": 1.0, "y": 2.0}
    assert p["goalOffEndpointM"] == 5.0 and p["commandId"] == "c1"
    assert nav_goal_payload("ab", None) == {"goalId": "ab"}
