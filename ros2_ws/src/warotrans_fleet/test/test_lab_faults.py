import pytest

from warotrans_fleet.lab_faults import UNREACHABLE_GOAL, fake_fingerprint, LabFaults, LabMessageError, contradicting_outcome


def msg(scenario, action="inject", ttl=30, **params):
    return {"schemaVersion": 1, "action": action, "injectionId": "inj-1", "scenarioId": scenario,
            "ttlSeconds": ttl, "parameters": params}


def test_disabled_robot_ignores_everything():
    lab = LabFaults(enabled=False)
    assert lab.handle(msg("IOT-01"), 0.0) == ("ignored", None)
    assert lab.mqtt_cut_until() is None


def test_unknown_scenario_and_bad_ttl_are_refused():
    lab = LabFaults(enabled=True)
    with pytest.raises(LabMessageError):
        lab.handle(msg("TRA-01"), 0.0)  # a host fault never runs on the robot
    with pytest.raises(LabMessageError):
        lab.handle(msg("IOT-01", ttl=5000), 0.0)
    with pytest.raises(LabMessageError):
        lab.handle({"schemaVersion": 2}, 0.0)


def test_every_fault_ends_by_itself():
    lab = LabFaults(enabled=True)
    lab.handle(msg("IOT-01", ttl=75), 100.0)
    assert lab.mqtt_cut_until() == 175.0
    assert lab.expire(174.9) == []
    ended = lab.expire(175.0)
    assert [f.scenario_id for f in ended] == ["IOT-01"]
    assert lab.mqtt_cut_until() is None


def test_clear_needs_the_same_injection():
    lab = LabFaults(enabled=True)
    lab.handle(msg("MAP-01"), 0.0)
    other = dict(msg("MAP-01", action="clear"), injectionId="inj-2")
    assert lab.handle(other, 1.0) == ("ignored", None)
    real = "27e8c7d9af1908e1d6ed4cce0123abcd"
    fake = lab.fingerprint(real)
    assert fake == fake_fingerprint(real) != real and len(fake) == len(real) and "lab" not in fake
    int(fake, 16)
    kind, _ = lab.handle(msg("MAP-01", action="clear"), 1.0)
    assert kind == "clear"
    assert lab.fingerprint("real") == "real"


def test_nav_abort_diverts_count_goals_then_stops():
    lab = LabFaults(enabled=True)
    lab.handle(msg("NAV-01", count="3"), 0.0)
    assert [lab.divert_goal() for _ in range(4)] == [UNREACHABLE_GOAL] * 3 + [None]


def test_freeze_and_stale_result_happen_once():
    lab = LabFaults(enabled=True)
    lab.handle(msg("NAV-02"), 0.0)
    assert lab.freeze_next_move() is not None
    assert lab.freeze_next_move() is None
    lab.handle(msg("OPS-04"), 0.0)
    assert lab.contradict_result() is True
    assert lab.contradict_result() is False
    assert contradicting_outcome("SUCCEEDED") == "FAILED"
    assert contradicting_outcome("CANCELED") == "SUCCEEDED"


def test_clock_offset_default_and_parameter():
    lab = LabFaults(enabled=True)
    assert lab.clock_offset_s() == 0.0
    lab.handle(msg("IOT-03", offset_s="60"), 0.0)
    assert lab.clock_offset_s() == 60.0
