"""The ROS / OS side of SYSTEM_ACTION (see system_actions.py): lifecycle managers and shell commands.

Nav2 and localization are restarted through their lifecycle manager (``<manager>/manage_nodes``: RESET, then STARTUP),
inside the running processes, so nothing else of the stack is touched. Camera, whole stack, reboot and power-off are
shell commands (gateway parameters), run detached in their own session so they outlive the gateway when they stop it.
"""

from __future__ import annotations

import os
import shlex
import subprocess
import threading
import time
from typing import Callable, Optional

from warotrans_fleet.system_actions import (
    REBOOT,
    RESTART_CAMERA,
    RESTART_LOCALIZATION,
    RESTART_NAV,
    RESTART_STACK,
    SHUTDOWN,
    SystemAction,
)

# nav2_msgs/srv/ManageLifecycleNodes
STARTUP = 0
RESET = 3


def _wait(future, timeout: float) -> bool:
    deadline = time.monotonic() + timeout
    while not future.done():
        if time.monotonic() > deadline:
            return False
        time.sleep(0.05)
    return True


class LifecycleRestarter:
    """RESET then STARTUP of one lifecycle manager. The gateway spins on another thread, so waiting here is safe."""

    def __init__(self, node, manager: str, *, timeout_sec: float = 90.0) -> None:
        self._node = node
        self._manager = manager.rstrip("/")
        self._timeout = timeout_sec
        self._client = None
        self._lock = threading.Lock()

    def _call(self, command: int) -> Optional[str]:
        from nav2_msgs.srv import ManageLifecycleNodes

        if self._client is None:
            self._client = self._node.create_client(ManageLifecycleNodes, f"{self._manager}/manage_nodes")
        if not self._client.wait_for_service(timeout_sec=3.0):
            return f"{self._manager}/manage_nodes is not available (is the stack running?)"
        request = ManageLifecycleNodes.Request()
        request.command = command
        future = self._client.call_async(request)
        name = "RESET" if command == RESET else "STARTUP"
        if not _wait(future, self._timeout):
            return f"{self._manager} {name} timed out after {self._timeout:.0f} s"
        result = future.result()
        if result is None or not result.success:
            return f"{self._manager} {name} failed"
        return None

    def restart(self) -> Optional[str]:
        with self._lock:
            return self._call(RESET) or self._call(STARTUP)


def run_detached(command: str, *, delay_sec: float = 2.0, log_file: Optional[str] = None) -> Optional[str]:
    """Runs ``command`` in a new session after ``delay_sec`` (so an MQTT answer can leave first). None when started."""
    if not command.strip():
        return "no command configured"
    target = os.path.expanduser(log_file) if log_file else os.devnull
    script = f"sleep {max(0.0, delay_sec):.1f}; {command}"
    try:
        with open(target, "ab") as sink:
            subprocess.Popen(
                ["bash", "-c", script], stdout=sink, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                start_new_session=True, cwd=os.path.expanduser("~"),
            )
    except OSError as exc:
        return f"cannot start: {exc}"
    return None


def run_and_wait(command: str, *, timeout_sec: float = 60.0) -> Optional[str]:
    """Runs a shell command to the end; its last output line is the error when it fails."""
    if not command.strip():
        return "no command configured"
    try:
        done = subprocess.run(
            ["bash", "-c", command], capture_output=True, text=True, timeout=timeout_sec,
            stdin=subprocess.DEVNULL, cwd=os.path.expanduser("~"),
        )
    except subprocess.TimeoutExpired:
        return f"timed out after {timeout_sec:.0f} s"
    except OSError as exc:
        return f"cannot run: {exc}"
    if done.returncode != 0:
        tail = (done.stderr or done.stdout or "").strip().splitlines()
        return f"exit {done.returncode}: {tail[-1] if tail else 'no output'}"
    return None


def sudo_allowed(command: str) -> Optional[str]:
    """None when ``sudo -n`` may run the first word of ``command`` without a password, else why not."""
    words = shlex.split(command)
    if not words:
        return "no command configured"
    if words[0] != "sudo":
        return None
    rest = words[1:]
    while rest and rest[0].startswith("-"):
        rest = rest[1:]
    if not rest:
        return "no command after sudo"
    binary = rest[0]
    try:
        # With its arguments: a rule may allow "systemctl restart <unit>" and nothing else.
        done = subprocess.run(["sudo", "-n", "-l", *rest], capture_output=True, timeout=5, stdin=subprocess.DEVNULL)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return f"sudo check failed: {exc}"
    if done.returncode != 0:
        return f"{binary} needs a password: run tools/install_system_actions.sh on the robot once"
    return None


def build_actions(
    node,
    *,
    enabled: list[str],
    nav_manager: str,
    localization_manager: str,
    camera_cmd: str,
    stack_cmd: str,
    reboot_cmd: str,
    shutdown_cmd: str,
    current_pose: Callable[[], Optional[tuple[float, float, float]]],
    publish_initial_pose: Callable[[float, float, float], None],
    log_file: str,
    log,
) -> dict[str, SystemAction]:
    actions: dict[str, SystemAction] = {}

    if RESTART_NAV in enabled and nav_manager:
        actions[RESTART_NAV] = SystemAction(run=LifecycleRestarter(node, nav_manager).restart)

    if RESTART_LOCALIZATION in enabled and localization_manager:
        restarter = LifecycleRestarter(node, localization_manager)

        def _restart_localization() -> Optional[str]:
            pose = current_pose()
            error = restarter.restart()
            if error:
                return error
            if pose is None:
                return "localization restarted but the robot had no pose: localize it"
            # AMCL needs a moment after activation before it takes /initialpose.
            time.sleep(1.5)
            publish_initial_pose(*pose)
            log.info(f"[SystemAction] initial pose restored x={pose[0]:.2f} y={pose[1]:.2f} yaw={pose[2]:.2f}")
            return None

        actions[RESTART_LOCALIZATION] = SystemAction(run=_restart_localization)

    if RESTART_CAMERA in enabled and camera_cmd:
        actions[RESTART_CAMERA] = SystemAction(run=lambda: run_and_wait(camera_cmd, timeout_sec=60.0))

    if RESTART_STACK in enabled and stack_cmd:
        # RBT-002 restarts its systemd unit through sudo; RBT-001's script needs none (sudo_allowed passes it).
        actions[RESTART_STACK] = SystemAction(run=lambda: run_detached(stack_cmd, log_file=log_file),
                                              precheck=lambda: sudo_allowed(stack_cmd))

    if REBOOT in enabled and reboot_cmd:
        actions[REBOOT] = SystemAction(run=lambda: run_detached(reboot_cmd, log_file=log_file),
                                       precheck=lambda: sudo_allowed(reboot_cmd))

    if SHUTDOWN in enabled and shutdown_cmd:
        actions[SHUTDOWN] = SystemAction(run=lambda: run_detached(shutdown_cmd, log_file=log_file),
                                         precheck=lambda: sudo_allowed(shutdown_cmd))

    return actions
