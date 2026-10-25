import importlib

import pytest
from warotrans_fleet.cmd_vel_authority_gate import AuthorityGateLogic, should_forward


def test_should_forward_matrix():
    assert should_forward(True, 0.0) is True
    assert should_forward(True, 0.3) is True
    assert should_forward(True, 0.31) is False  # quá cũ
    assert should_forward(False, 0.0) is False
    assert should_forward(True, None) is False  # chưa từng nhận
    assert should_forward(True, -0.1) is False  # đồng hồ nhảy lùi
    assert should_forward(True, float("nan")) is False
    assert should_forward(True, 0.5, max_age=1.0) is True


def test_truthy_non_bool_does_not_forward():
    assert should_forward(1, 0.0) is False  # type: ignore[arg-type]


def test_logic_no_message_blocks():
    logic = AuthorityGateLogic()
    assert logic.decide(10.0).forward is False


def test_logic_forwards_then_goes_stale():
    logic = AuthorityGateLogic(max_age_sec=0.3)
    logic.on_authority_ok(True, 10.0)
    assert logic.decide(10.1).forward is True
    decision = logic.decide(10.5)  # authority_ok không được refresh → chặn
    assert decision.forward is False and decision.publish_zero is True


def test_logic_false_blocks_immediately_and_zero_hold_expires():
    logic = AuthorityGateLogic(max_age_sec=0.3, zero_hold_sec=1.0)
    logic.on_authority_ok(True, 10.0)
    assert logic.decide(10.0).forward
    logic.on_authority_ok(False, 10.05)
    first = logic.decide(10.05)
    assert (first.forward, first.publish_zero) == (False, True)
    logic.on_authority_ok(False, 11.5)
    late = logic.decide(11.5)
    assert (late.forward, late.publish_zero) == (False, False)  # đã đứng yên, không tranh /cmd_vel
    logic.on_authority_ok(True, 11.6)
    assert logic.decide(11.6).forward is True


def test_invalid_max_age():
    with pytest.raises(ValueError):
        AuthorityGateLogic(max_age_sec=0)


def test_module_importable_without_rclpy_and_main_exists():
    module = importlib.import_module("warotrans_fleet.cmd_vel_authority_gate")
    assert callable(module.main)
