"""Port giữa MoveCommandHandler và Nav2 — implementation ROS ở nav2_navigator.py, fake trong test.

Robot KHÔNG tự lập đường đi: chỉ gửi goal NavigateToPose và theo dõi lifecycle của goal.
Nav2 lo global/local planning, tránh vật cản, replanning, controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Optional, Protocol

from warotrans_fleet.nav2_status_tracker import (
    STATUS_ABORTED,
    STATUS_CANCELED,
    STATUS_SUCCEEDED,
    STATUS_UNKNOWN,
)

__all__ = [
    "STATUS_ABORTED",
    "STATUS_CANCELED",
    "STATUS_SUCCEEDED",
    "STATUS_UNKNOWN",
    "GoalCallbacks",
    "GoalTicket",
    "NavFeedback",
    "NavGoal",
    "NavResult",
    "NavigatorPort",
    "UnavailableNavigator",
]


@dataclass(frozen=True)
class NavGoal:
    command_id: str
    frame_id: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class NavFeedback:
    distance_remaining: Optional[float] = None
    navigation_time_sec: Optional[float] = None
    estimated_time_remaining_sec: Optional[float] = None
    number_of_recoveries: Optional[int] = None


@dataclass(frozen=True)
class NavResult:
    status: int  # action_msgs/GoalStatus code
    error_code: Optional[int] = None  # NavigateToPose.Result.error_code (Jazzy)
    error_message: Optional[str] = None


class GoalTicket:
    """Opaque handle; adapter gắn goal handle thật vào `handle` khi goal được accept."""

    def __init__(self) -> None:
        self.handle: Any = None


@dataclass(frozen=True)
class GoalCallbacks:
    # (accepted, transport_error). transport_error != None nghĩa là không gửi/nhận được response.
    on_response: Callable[[bool, Optional[str]], None]
    on_feedback: Callable[[NavFeedback], None]
    on_result: Callable[[NavResult], None]


class NavigatorPort(Protocol):
    def is_ready(self) -> bool:
        """Non-blocking: action server đã sẵn sàng chưa."""
        ...

    def send_goal(self, goal: NavGoal, ticket: GoalTicket, callbacks: GoalCallbacks) -> None:
        """Non-blocking; kết quả về qua callbacks."""
        ...

    def cancel_goal(self, ticket: GoalTicket, on_response: Callable[[bool, str], None]) -> None:
        """Non-blocking; on_response(accepted, detail) khi Nav2 trả lời cancel request."""
        ...


class UnavailableNavigator:
    """Dùng khi không import được nav2_msgs — mọi MOVE bị reject NAV2_UNAVAILABLE."""

    def is_ready(self) -> bool:
        return False

    def send_goal(self, goal: NavGoal, ticket: GoalTicket, callbacks: GoalCallbacks) -> None:
        raise RuntimeError("nav2_msgs not available")

    def cancel_goal(self, ticket: GoalTicket, on_response: Callable[[bool, str], None]) -> None:
        on_response(False, "nav2_msgs not available")
