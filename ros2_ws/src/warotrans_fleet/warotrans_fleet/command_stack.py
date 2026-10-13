"""Wiring CommandRouter + handlers (dùng chung cho node và test để không lệch cấu hình).

Step type tương lai (CHECK / WAIT / HUMAN_INTERACTION) nếu có phần chạy trên robot:
viết handler mới theo CommandHandler protocol và register ở đây.
HUMAN_INTERACTION chủ yếu thuộc Backend + Staff App, không bắt buộc có handler trên robot.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

from warotrans_fleet.command_router import CommandEventPublisher, CommandRouter, LogSink
from warotrans_fleet.move_command_handler import CancelCommandHandler, MoveCommandHandler
from warotrans_fleet.navigator import NavigatorPort


@dataclass
class CommandStack:
    router: CommandRouter
    move: MoveCommandHandler


def build_command_stack(
    *,
    robot_code: str,
    publisher: CommandEventPublisher,
    navigator: NavigatorPort,
    localization_provider: Callable[[], str],
    log: LogSink,
    clock: Callable[[], float] = time.monotonic,
    map_frame: str = "map",
    robot_map_version_code: Optional[str] = None,
    default_timeout_sec: Optional[float] = None,
    goal_response_timeout_sec: float = 10.0,
    cancel_confirm_timeout_sec: float = 10.0,
    terminal_status_hold_sec: float = 3.0,
    ledger_capacity: int = 500,
    external_goal_active: Callable[[], bool] = lambda: False,
) -> CommandStack:
    lock = threading.RLock()
    router = CommandRouter(
        robot_code=robot_code,
        publisher=publisher,
        log=log,
        lock=lock,
        ledger_capacity=ledger_capacity,
    )
    move = MoveCommandHandler(
        navigator=navigator,
        localization_provider=localization_provider,
        log=log,
        lock=lock,
        clock=clock,
        map_frame=map_frame,
        robot_map_version_code=robot_map_version_code,
        default_timeout_sec=default_timeout_sec,
        goal_response_timeout_sec=goal_response_timeout_sec,
        cancel_confirm_timeout_sec=cancel_confirm_timeout_sec,
        terminal_status_hold_sec=terminal_status_hold_sec,
        external_goal_active=external_goal_active,
    )
    router.register(move)
    router.register(CancelCommandHandler(move))
    return CommandStack(router=router, move=move)
