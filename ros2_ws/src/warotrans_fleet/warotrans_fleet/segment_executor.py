"""Port chạy một segment (danh sách edge đã được cấp quyền) — Active Mode.

Implementation ROS thật (Nav2 route_server ComputeRoute + FollowPath/ComputeAndTrackRoute) chưa có;
ở đây chỉ có protocol + FakeSegmentExecutor cho test + UnavailableSegmentExecutor (fail-safe).
Pure Python — không import ROS.
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass, field
from typing import Callable, Optional, Protocol, Sequence

# Trạng thái kết quả của một lần follow_path.
RESULT_SUCCEEDED = "SUCCEEDED"
RESULT_ABORTED = "ABORTED"
RESULT_CANCELED = "CANCELED"
RESULT_TIMEOUT = "TIMEOUT"
RESULT_STATUSES = (RESULT_SUCCEEDED, RESULT_ABORTED, RESULT_CANCELED, RESULT_TIMEOUT)

Pose2D = tuple[float, float, float]  # (x, y, yaw) trong frame map


@dataclass(frozen=True)
class SegmentResult:
    status: str  # một trong RESULT_STATUSES
    detail: Optional[str] = None

    def __post_init__(self) -> None:
        if self.status not in RESULT_STATUSES:
            raise ValueError(f"invalid segment result status {self.status!r}")


@dataclass(frozen=True)
class SegmentCallbacks:
    on_edge_entered: Callable[[int], None]
    on_edge_exited: Callable[[int], None]
    on_result: Callable[[SegmentResult], None]


class SegmentTicket:
    """Handle opaque cho một lần follow_path. Adapter ROS có thể gắn goal handle vào ``handle``."""

    _ids = itertools.count(1)

    def __init__(self, edge_ids: Sequence[int]) -> None:
        self.ticket_id = next(SegmentTicket._ids)
        self.edge_ids: tuple[int, ...] = tuple(edge_ids)
        self.handle: object = None
        self.canceled = False
        self.finished = False

    def __repr__(self) -> str:  # pragma: no cover
        return f"SegmentTicket(id={self.ticket_id}, edges={list(self.edge_ids)})"


class SegmentExecutorPort(Protocol):
    def compute_route_edge_ids(self, start: Pose2D, goal: Pose2D) -> list[int]:
        """Non-blocking-friendly: trả danh sách edge id route_server sẽ đi từ start tới goal ([] nếu không có)."""
        ...

    def follow_path(self, edge_ids: Sequence[int], callbacks: SegmentCallbacks) -> SegmentTicket:
        """Bắt đầu chạy đúng các edge này (không đi ngoài). Kết quả về qua callbacks."""
        ...

    def cancel(self) -> None:
        """Dừng segment đang chạy; kết quả cuối (CANCELED/ABORTED) vẫn về qua on_result."""
        ...


class UnavailableSegmentExecutor:
    """Fail-safe khi chưa có implementation ROS: không bao giờ chạy."""

    def compute_route_edge_ids(self, start: Pose2D, goal: Pose2D) -> list[int]:
        return []

    def follow_path(self, edge_ids: Sequence[int], callbacks: SegmentCallbacks) -> SegmentTicket:
        raise RuntimeError("segment executor not available (Nav2 FollowPath adapter not implemented)")

    def cancel(self) -> None:
        return None


@dataclass
class FakeSegmentExecutor:
    """Test double: test điều khiển edge/kết quả thủ công.

    - ``routes``: dict {(start, goal): [edge ids]}; không có key → ``default_route``.
    - ``fail_follow``: follow_path raise RuntimeError.
    - ``auto_cancel_result``: cancel() tự phát CANCELED ngay (đặt False để test trạng thái STOPPING).
    """

    default_route: list[int] = field(default_factory=list)
    routes: dict[tuple[Pose2D, Pose2D], list[int]] = field(default_factory=dict)
    fail_follow: bool = False
    auto_cancel_result: bool = True
    follow_calls: list[tuple[int, ...]] = field(default_factory=list)
    compute_calls: list[tuple[Pose2D, Pose2D]] = field(default_factory=list)
    cancel_calls: int = 0
    ticket: Optional[SegmentTicket] = None
    _callbacks: Optional[SegmentCallbacks] = None

    def compute_route_edge_ids(self, start: Pose2D, goal: Pose2D) -> list[int]:
        self.compute_calls.append((start, goal))
        return list(self.routes.get((start, goal), self.default_route))

    def follow_path(self, edge_ids: Sequence[int], callbacks: SegmentCallbacks) -> SegmentTicket:
        if self.fail_follow:
            raise RuntimeError("fake follow_path failure")
        self.follow_calls.append(tuple(edge_ids))
        self.ticket = SegmentTicket(edge_ids)
        self._callbacks = callbacks
        return self.ticket

    def cancel(self) -> None:
        self.cancel_calls += 1
        if self.ticket is not None:
            self.ticket.canceled = True
        if self.auto_cancel_result:
            self.finish(RESULT_CANCELED)

    # -- helper cho test

    @property
    def running(self) -> bool:
        return self.ticket is not None and not self.ticket.finished

    def enter_edge(self, edge_id: int) -> None:
        assert self._callbacks is not None
        self._callbacks.on_edge_entered(edge_id)

    def exit_edge(self, edge_id: int) -> None:
        assert self._callbacks is not None
        self._callbacks.on_edge_exited(edge_id)

    def finish(self, status: str, detail: Optional[str] = None) -> None:
        if self.ticket is None or self._callbacks is None or self.ticket.finished:
            return
        self.ticket.finished = True
        self._callbacks.on_result(SegmentResult(status, detail))
