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
