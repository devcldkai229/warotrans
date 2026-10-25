import pytest
from warotrans_fleet.segment_executor import (
    RESULT_ABORTED,
    RESULT_CANCELED,
    RESULT_SUCCEEDED,
    RESULT_TIMEOUT,
    FakeSegmentExecutor,
    SegmentCallbacks,
    SegmentResult,
    UnavailableSegmentExecutor,
)


def _callbacks(log):
    return SegmentCallbacks(
        on_edge_entered=lambda e: log.append(("in", e)),
        on_edge_exited=lambda e: log.append(("out", e)),
        on_result=lambda r: log.append(("result", r.status)),
    )


def test_compute_route_default_and_override():
    ex = FakeSegmentExecutor(default_route=[1, 2])
    a, b = (0.0, 0.0, 0.0), (1.0, 1.0, 0.0)
    assert ex.compute_route_edge_ids(a, b) == [1, 2]
    ex.routes[(a, b)] = [9]
    assert ex.compute_route_edge_ids(a, b) == [9]
    assert ex.compute_calls == [(a, b), (a, b)]


@pytest.mark.parametrize("status", [RESULT_SUCCEEDED, RESULT_ABORTED, RESULT_CANCELED, RESULT_TIMEOUT])
def test_follow_path_reports_each_status(status):
    log = []
    ex = FakeSegmentExecutor()
    ticket = ex.follow_path([1, 2], _callbacks(log))
    assert ticket.edge_ids == (1, 2) and ex.running
    ex.enter_edge(1)
    ex.exit_edge(1)
    ex.finish(status)
    assert log == [("in", 1), ("out", 1), ("result", status)]
    assert not ex.running
    ex.finish(RESULT_SUCCEEDED)  # kết quả thứ hai bị bỏ qua
    assert log[-1] == ("result", status)


def test_cancel_emits_canceled_by_default_and_can_be_deferred():
    log = []
    ex = FakeSegmentExecutor()
    ticket = ex.follow_path([1], _callbacks(log))
    ex.cancel()
    assert ticket.canceled and log == [("result", RESULT_CANCELED)]

    log2 = []
    ex2 = FakeSegmentExecutor(auto_cancel_result=False)
    ex2.follow_path([1], _callbacks(log2))
    ex2.cancel()
    assert log2 == [] and ex2.cancel_calls == 1 and ex2.running
    ex2.finish(RESULT_CANCELED)
    assert log2 == [("result", RESULT_CANCELED)]


def test_invalid_status_rejected():
    with pytest.raises(ValueError):
        SegmentResult("DONE")


def test_fail_follow_raises():
    ex = FakeSegmentExecutor(fail_follow=True)
    with pytest.raises(RuntimeError):
        ex.follow_path([1], _callbacks([]))


def test_unavailable_executor_never_runs():
    ex = UnavailableSegmentExecutor()
    assert ex.compute_route_edge_ids((0, 0, 0), (1, 1, 0)) == []
    with pytest.raises(RuntimeError):
        ex.follow_path([1], _callbacks([]))
    ex.cancel()
