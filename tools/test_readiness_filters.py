"""The costmap filter checks of real_robot_readiness.py, without ROS: rclpy and friends are stubbed for the import."""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from unittest import mock

import pytest

STUBS = (
    "rclpy", "rclpy.duration", "rclpy.node", "rclpy.qos", "rclpy.time", "geometry_msgs.msg", "geometry_msgs",
    "nav_msgs", "nav_msgs.msg", "rcl_interfaces", "rcl_interfaces.srv", "sensor_msgs", "sensor_msgs.msg",
    "std_msgs", "std_msgs.msg", "std_srvs", "std_srvs.srv", "tf2_ros",
)


@pytest.fixture(scope="module")
def readiness():
    with mock.patch.dict(sys.modules, {name: mock.MagicMock() for name in STUBS}):
        sys.modules["rclpy.node"].Node = object
        spec = importlib.util.spec_from_file_location(
            "real_robot_readiness", Path(__file__).with_name("real_robot_readiness.py"))
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module  # dataclasses look their module up here
        spec.loader.exec_module(module)
        yield module


def _healthy(readiness):
    return {t: 28 for t in readiness.MASK_TOPICS}


def test_all_filters_alive_pass(readiness):
    checks = readiness.costmap_filter_checks(lambda _t: 1, _healthy(readiness))
    assert checks and all(ok for _, ok, _ in checks)


def test_speed_limit_with_only_a_subscriber_fails(readiness):
    """The controller always subscribes to /speed_limit: the topic exists even when the filter is dead."""
    counts = {"/speed_limit": 0}
    checks = readiness.costmap_filter_checks(lambda t: counts.get(t, 1), _healthy(readiness))
    failed = [name for name, ok, _ in checks if not ok]
    assert len(failed) == 1 and "/speed_limit" in failed[0]


def test_missing_planner_filter_fails(readiness):
    counts = {"/keepout_planning_costmap_filter_info": 0}
    checks = readiness.costmap_filter_checks(lambda t: counts.get(t, 1), _healthy(readiness))
    assert [name for name, ok, _ in checks if not ok]


@pytest.mark.parametrize("masks", [{}, {"/keepout_filter_mask": 0}])
def test_mask_not_received_or_empty_fails(readiness, masks):
    all_masks = {**_healthy(readiness), **masks} if masks else {}
    checks = readiness.costmap_filter_checks(lambda _t: 1, all_masks)
    assert [name for name, ok, _ in checks if not ok]
