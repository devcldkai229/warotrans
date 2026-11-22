"""Nav2SegmentExecutor Spin without a current yaw (no ROS needed: __init__ is bypassed)."""

import threading

import pytest

from warotrans_fleet.authority_store import ManeuverStep
from warotrans_fleet.nav2_segment_executor import Nav2SegmentExecutor
from warotrans_fleet.segment_executor import RESULT_ABORTED


class _Server:
    def wait_for_server(self, timeout_sec=0.0):
        return True

    def send_goal_async(self, goal):  # must never be reached without a yaw
        raise AssertionError("Spin goal sent without a current yaw")


class _Callbacks:
    def __init__(self):
        self.results = []

    def on_result(self, result):
        self.results.append(result)


def _executor(get_yaw_fn):
    ex = Nav2SegmentExecutor.__new__(Nav2SegmentExecutor)
    ex._spin_client = _Server()
    ex._Spin = object
    ex._get_yaw_fn = get_yaw_fn
    ex._lock = threading.Lock()
    ex._ticket = None
    ex._callbacks = _Callbacks()
    ex._goal_handle = None
    ex._pending_steps = []
    ex._log = lambda _m: None
    return ex


def _boom():
    raise RuntimeError("no tf")


@pytest.mark.parametrize("get_yaw_fn", [None, lambda: None, _boom])
def test_spin_aborts_without_current_yaw(get_yaw_fn):
    ex = _executor(get_yaw_fn)
    ex._start_spin(ManeuverStep(kind="Spin", yaw_target=1.57))
    [result] = ex._callbacks.results
    assert result.status == RESULT_ABORTED
    assert result.detail == "SPIN_YAW_UNAVAILABLE"
