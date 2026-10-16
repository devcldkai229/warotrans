"""Fakes dùng chung cho unit test command stack (không cần ROS)."""

from __future__ import annotations

import logging
from typing import Any, Callable, Optional

from warotrans_fleet.command_stack import CommandStack, build_command_stack
from warotrans_fleet.navigator import GoalCallbacks, GoalTicket, NavFeedback, NavGoal, NavResult

ROBOT = "RBT-001"


class FakeClock:
    def __init__(self) -> None:
        self.now = 1000.0

    def __call__(self) -> float:
        return self.now

    def advance(self, seconds: float) -> None:
        self.now += seconds


class Recorder:
    """CommandEventPublisher + event log để kiểm tra thứ tự ACK → goal → RESULT."""

    def __init__(self) -> None:
        self.events: list[tuple[str, Any]] = []

    def publish_ack(self, payload: dict[str, Any]) -> None:
        self.events.append(("ack", payload))

    def publish_result(self, payload: dict[str, Any]) -> None:
        self.events.append(("result", payload))

    @property
    def acks(self) -> list[dict[str, Any]]:
        return [p for kind, p in self.events if kind == "ack"]

    @property
    def results(self) -> list[dict[str, Any]]:
        return [p for kind, p in self.events if kind == "result"]

    def kinds(self) -> list[str]:
        return [kind for kind, _ in self.events]


class FakeNavigator:
    """NavigatorPort điều khiển tay: test quyết định khi nào Nav2 accept / feedback / kết thúc."""

    def __init__(self, recorder: Optional[Recorder] = None) -> None:
        self.ready = True
        self.goals: list[tuple[NavGoal, GoalTicket, GoalCallbacks]] = []
        self.cancel_requests: list[tuple[GoalTicket, Callable[[bool, str], None]]] = []
        self._recorder = recorder

    def is_ready(self) -> bool:
        return self.ready

    def send_goal(self, goal: NavGoal, ticket: GoalTicket, callbacks: GoalCallbacks) -> None:
        self.goals.append((goal, ticket, callbacks))
        if self._recorder is not None:
            self._recorder.events.append(("nav2_goal", goal))

    def cancel_goal(self, ticket: GoalTicket, on_response: Callable[[bool, str], None]) -> None:
        self.cancel_requests.append((ticket, on_response))
        if self._recorder is not None:
            self._recorder.events.append(("nav2_cancel", ticket))

    # --- điều khiển phía "Nav2"
    def _cb(self, index: int = -1) -> GoalCallbacks:
        return self.goals[index][2]

    def accept(self, index: int = -1) -> None:
        self.goals[index][1].handle = object()
        self._cb(index).on_response(True, None)

    def reject(self, index: int = -1) -> None:
        self._cb(index).on_response(False, None)

    def feedback(self, distance: float, index: int = -1) -> None:
        self._cb(index).on_feedback(NavFeedback(distance_remaining=distance, number_of_recoveries=0))

    def finish(self, status: int, index: int = -1, error_code: Optional[int] = None, error_message: Optional[str] = None) -> None:
        self._cb(index).on_result(NavResult(status=status, error_code=error_code, error_message=error_message))

    def answer_cancel(self, accepted: bool, detail: str = "ERROR_NONE", index: int = -1) -> None:
        self.cancel_requests[index][1](accepted, detail)


class Harness:
    def __init__(self, **overrides: Any) -> None:
        self.clock = FakeClock()
        self.recorder = Recorder()
        self.nav = FakeNavigator(self.recorder)
        self.localization = "LOCALIZED"
        self.external_goal = False
        kwargs: dict[str, Any] = dict(
            robot_code=ROBOT,
            publisher=self.recorder,
            navigator=self.nav,
            localization_provider=lambda: self.localization,
            log=logging.getLogger("test.fleet"),
            clock=self.clock,
            goal_response_timeout_sec=10.0,
            cancel_confirm_timeout_sec=10.0,
            terminal_status_hold_sec=3.0,
            external_goal_active=lambda: self.external_goal,
        )
        kwargs.update(overrides)
        self.stack: CommandStack = build_command_stack(**kwargs)

    def send(self, body: dict[str, Any]) -> None:
        self.stack.router.handle_message(body)

    def tick(self, advance: float = 0.0) -> None:
        self.clock.advance(advance)
        self.stack.router.tick()

    @property
    def move(self):
        return self.stack.move


def navigate(command_id: str = "11111111-1111-1111-1111-111111111111", **payload_overrides: Any) -> dict[str, Any]:
    payload: dict[str, Any] = {"frameId": "map", "x": 4.25, "y": 2.10, "yaw": 1.5708}
    payload.update(payload_overrides)
    return {
        "schemaVersion": 1,
        "messageId": "m-" + command_id,
        "commandId": command_id,
        "robotCode": ROBOT,
        "type": "NAVIGATE_TO_POSE",
        "issuedAt": "2026-10-13T08:00:00.000Z",
        "jobAssignmentId": "aaaaaaaa-0000-0000-0000-000000000001",
        "jobStepId": "bbbbbbbb-0000-0000-0000-000000000001",
        "payload": payload,
    }


def cancel(target: str, command_id: str = "22222222-2222-2222-2222-222222222222") -> dict[str, Any]:
    return {
        "schemaVersion": 1,
        "messageId": "m-" + command_id,
        "commandId": command_id,
        "robotCode": ROBOT,
        "type": "CANCEL",
        "issuedAt": "2026-10-13T08:00:05.000Z",
        "payload": {"targetCommandId": target},
    }
