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
from warotrans_fleet.motion_hold import (
    HoldGuard,
    MotionHold,
    ResumeCommandHandler,
    SpeedProvider,
    StopCommandHandler,
    SystemActionHandler,
)
from warotrans_fleet.move_command_handler import CancelCommandHandler, MoveCommandHandler
from warotrans_fleet.navigator import NavigatorPort
from warotrans_fleet.route_enforcer import RouteEnforcer

if False:  # TYPE_CHECKING without importing typing.TYPE_CHECKING for py39-ish
    from warotrans_fleet.traffic_client import TrafficClient


@dataclass
class CommandStack:
    router: CommandRouter
    move: MoveCommandHandler
    hold: Optional[MotionHold] = None
    system_actions: Optional[SystemActionHandler] = None


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
    route_enforcer: Optional[RouteEnforcer] = None,
    traffic_mode: str = "legacy",
    traffic_client: Optional[object] = None,
    expected_graph_hash: Optional[str] = None,
    traffic_hold_active: Callable[[], bool] = lambda: False,
    arrival_pose: Callable[[], Optional[tuple[float, float, float]]] = lambda: None,
    speed_provider: SpeedProvider = lambda: None,
    stop_settle_sec: float = 0.5,
    stop_verify_timeout_sec: float = 5.0,
    system_actions_enabled: tuple[str, ...] = (),
    run_system_action: Callable[[str], None] = lambda action: None,
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
        route_enforcer=route_enforcer,
        traffic_mode=traffic_mode,
        traffic_client=traffic_client,  # type: ignore[arg-type]
        expected_graph_hash=expected_graph_hash,
        traffic_hold_active=traffic_hold_active,
        arrival_pose=arrival_pose,
    )
    # RB-03: STOP / PAUSE / RESUME work in every traffic mode; a held robot takes no NAVIGATE.
    hold = MotionHold(
        stop_active=move.stop_active,
        speed_provider=speed_provider,
        clock=clock,
        log=log,
        settle_sec=stop_settle_sec,
        verify_timeout_sec=stop_verify_timeout_sec,
    )
    router.register(HoldGuard(move, hold))
    router.register(CancelCommandHandler(move))
    router.register(StopCommandHandler(hold))
    router.register(ResumeCommandHandler(hold))
    system_actions = SystemActionHandler(
        hold,
        enabled_actions=system_actions_enabled,
        goal_active=lambda: move.active_command_id is not None,
        run_action=run_system_action,
    )
    router.register(system_actions)
    if traffic_mode == "active" and traffic_client is not None:
        from warotrans_fleet.authority_command_handler import (
            ExtendAuthorityHandler,
            RenewAuthorityHandler,
            RevokeAuthorityHandler,
        )

        router.register(ExtendAuthorityHandler(traffic_client, expected_graph_hash=expected_graph_hash))  # type: ignore[arg-type]
        router.register(RenewAuthorityHandler(traffic_client))  # type: ignore[arg-type]
        router.register(RevokeAuthorityHandler(traffic_client))  # type: ignore[arg-type]
    return CommandStack(router=router, move=move, hold=hold, system_actions=system_actions)
