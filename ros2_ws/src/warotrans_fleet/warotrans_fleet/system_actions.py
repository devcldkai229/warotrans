"""SYSTEM_ACTION from the backend (RB-09): restart one stack of the robot, or the whole robot computer.

Actions (``payload.action``):

  RESTART_NAV           Nav2 lifecycle RESET then STARTUP (planner, controller, costmaps, BT navigator).
  RESTART_LOCALIZATION  map_server + AMCL lifecycle RESET then STARTUP, then the pose the robot had is published again
                        as /initialpose, so it does not wake up lost.
  RESTART_CAMERA        camera stream + evidence agent processes restarted.
  RESTART_STACK         every process of the robot stack stopped and started again (bringup, Nav2, gateway, camera).
                        The gateway itself goes down: the result is sent first, then the restart runs detached.
  REBOOT / SHUTDOWN     the robot computer reboots / powers off (sudoers rule, tools/install_system_actions.sh).

Only actions this robot can run are announced in the heartbeat (``capabilities``) and accepted. Every action needs the
robot to hold motion (STOP first) and to run no MOVE: restarting Nav2 under a goal, or rebooting a moving robot, is
never what an operator wants.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Callable, Mapping, Optional

from warotrans_fleet.command_contract import (
    OUTCOME_FAILED,
    OUTCOME_SUCCEEDED,
    REASON_BUSY,
    REASON_INVALID_PAYLOAD,
    REASON_UNSUPPORTED_COMMAND,
    CommandEnvelope,
    CommandRejected,
)
from warotrans_fleet.command_router import AckDecision, CommandSession, LogSink

SYSTEM_ACTION = "SYSTEM_ACTION"

RESTART_NAV = "RESTART_NAV"
RESTART_LOCALIZATION = "RESTART_LOCALIZATION"
RESTART_CAMERA = "RESTART_CAMERA"
RESTART_STACK = "RESTART_STACK"
REBOOT = "REBOOT"
SHUTDOWN = "SHUTDOWN"

ALL_ACTIONS = (RESTART_NAV, RESTART_LOCALIZATION, RESTART_CAMERA, RESTART_STACK, REBOOT, SHUTDOWN)

# These end the gateway process: the result must leave before they run.
DETACHED_ACTIONS = frozenset({RESTART_STACK, REBOOT, SHUTDOWN})

REASON_MOTION_NOT_HELD = "MOTION_NOT_HELD"
REASON_ACTION_NOT_PERMITTED = "ACTION_NOT_PERMITTED"


@dataclass(frozen=True)
class SystemAction:
    """One action: ``run`` returns None on success or an error text. ``precheck`` (optional) runs before the ACK and
    returns an error text when the action cannot run here at all (e.g. no sudo rule for reboot)."""

    run: Callable[[], Optional[str]]
    precheck: Optional[Callable[[], Optional[str]]] = None


def parse_action_list(text: str) -> list[str]:
    """``"restart_nav, REBOOT"`` → ``["RESTART_NAV", "REBOOT"]``; unknown names are dropped, order kept, no repeats."""
    out: list[str] = []
    for part in (text or "").split(","):
        name = part.strip().upper()
        if name in ALL_ACTIONS and name not in out:
            out.append(name)
    return out


class SystemActionHandler:
    command_type = SYSTEM_ACTION

    def __init__(
        self,
        actions: Mapping[str, SystemAction],
        *,
        motion_held: Callable[[], bool],
        busy: Callable[[], bool],
        log: LogSink,
        run_async: Optional[Callable[[Callable[[], None]], None]] = None,
    ) -> None:
        self._actions = dict(actions)
        self._held = motion_held
        self._busy = busy
        self._log = log
        self._run_async = run_async or _thread
        self._lock = threading.Lock()
        self._running: Optional[str] = None

    @property
    def announced(self) -> list[str]:
        """What the heartbeat announces: the actions this robot can run, in a stable order."""
        return [name for name in ALL_ACTIONS if name in self._actions]

    def handle(self, command: CommandEnvelope, session: CommandSession) -> AckDecision:
        action = str(command.payload.get("action") or "").strip().upper()
        if not action:
            raise CommandRejected(REASON_INVALID_PAYLOAD, "payload.action is required")
        spec = self._actions.get(action)
        if spec is None:
            raise CommandRejected(REASON_UNSUPPORTED_COMMAND, f"{action} is not available on this robot")
        if not self._held():
            raise CommandRejected(REASON_MOTION_NOT_HELD, "stop the robot first (it must hold motion)")
        if self._busy():
            raise CommandRejected(REASON_BUSY, "a MOVE is running; cancel it first")
        if spec.precheck is not None:
            problem = spec.precheck()
            if problem:
                raise CommandRejected(REASON_ACTION_NOT_PERMITTED, problem)
        with self._lock:
            if self._running is not None:
                raise CommandRejected(REASON_BUSY, f"{self._running} is still running")
            self._running = action

        def _execute() -> None:
            try:
                if action in DETACHED_ACTIONS:
                    # The process that answers is about to die: answer first, then schedule.
                    session.complete(OUTCOME_SUCCEEDED, result={"action": action, "scheduled": True})
                    self._log.warning(f"[SystemAction] {action} scheduled")
                    error = spec.run()
                    if error:
                        self._log.error(f"[SystemAction] {action} could not be scheduled: {error}")
                    return
                self._log.warning(f"[SystemAction] {action} started")
                try:
                    error = spec.run()
                except Exception as exc:  # noqa: BLE001 - a broken action must still answer
                    error = f"{type(exc).__name__}: {exc}"
                if error:
                    self._log.error(f"[SystemAction] {action} failed: {error}")
                    session.complete(
                        OUTCOME_FAILED, error_code="SYSTEM_ACTION_FAILED", error_message=error[:300],
                        result={"action": action},
                    )
                else:
                    self._log.warning(f"[SystemAction] {action} done")
                    session.complete(OUTCOME_SUCCEEDED, result={"action": action})
            finally:
                with self._lock:
                    self._running = None

        # Lifecycle transitions take seconds: never on the MQTT callback thread.
        return AckDecision.accept(after_ack=lambda: self._run_async(_execute))


def _thread(fn: Callable[[], None]) -> None:
    threading.Thread(target=fn, name="system-action", daemon=True).start()
