"""Deterministic, motor-less robot client for the traffic E2E harness.

It runs the robot gateway's own logic — CommandRouter/MoveCommandHandler (ACK/RESULT contract), SemanticTrafficClient
(plan proposal, grant checks, reservation containment, verified exits) and AuthorityGateLogic (cmd_vel gate) —
against the real backend over the real broker, on its own topic prefix and robot code. Nothing here drives hardware:
"Nav2" is a straight-line planner and the pose advances along it only while the gate forwards velocity.

Recorded for assertions: every gate decision, every pose step taken (and whether the gate allowed it), every
authority message received.
"""

from __future__ import annotations

import logging
import math
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional

FLEET = Path(__file__).resolve().parents[2] / "ros2_ws" / "src" / "warotrans_fleet"
sys.path.insert(0, str(FLEET))

from warotrans_fleet.cmd_vel_authority_gate import AuthorityGateLogic  # noqa: E402
from warotrans_fleet.command_stack import build_command_stack  # noqa: E402
from warotrans_fleet.mqtt_publisher import ReconnectingMqttPublisher, default_paho_factory  # noqa: E402
from warotrans_fleet.navigator import (  # noqa: E402
    STATUS_ABORTED,
    STATUS_CANCELED,
    STATUS_SUCCEEDED,
    GoalCallbacks,
    GoalTicket,
    NavFeedback,
    NavGoal,
    NavResult,
)
from warotrans_fleet.payload_builder import build_heartbeat, build_telemetry  # noqa: E402
from warotrans_fleet.semantic_traffic_client import SemanticTrafficClient  # noqa: E402
from warotrans_fleet.sequence import SequenceClock  # noqa: E402

log = logging.getLogger("sim_robot")


@dataclass
class Step:
    t: float
    x: float
    y: float
    gate_open: bool


class StraightLineNav:
    """NavigatorPort: plans a straight line; the robot's pose only advances while its gate is open."""

    def __init__(self, robot: "SimRobot") -> None:
        self._robot = robot
        self.goal: Optional[NavGoal] = None
        self.callbacks: Optional[GoalCallbacks] = None
        self.path: list[tuple[float, float, float]] = []
        self.blocked_since: Optional[float] = None
        # A replan through this point (what Nav2 does around a new obstacle); None = straight to the goal.
        self.detour: Optional[tuple[float, float]] = None
        # Real planner output for this goal (fixture): followed point by point, never replanned to a straight line.
        self.route: list[tuple[float, float, float]] = []
        self.lock = threading.Lock()

    def is_ready(self) -> bool:
        return True

    def send_goal(self, goal: NavGoal, ticket: GoalTicket, callbacks: GoalCallbacks) -> None:
        with self.lock:
            ticket.handle = object()
            self.goal, self.callbacks = goal, callbacks
            self.path = self._plan(goal)
            self.blocked_since = None
        callbacks.on_response(True, None)
        self._robot.publish_plan()

    def cancel_goal(self, ticket: GoalTicket, on_response) -> None:
        on_response(True, "ERROR_NONE")
        self._finish(STATUS_CANCELED)

    def _plan(self, goal: NavGoal) -> list[tuple[float, float, float]]:
        if self._robot.path_provider is not None:
            if not self.route or self.goal is not goal:
                self.route = self._robot.path_provider(self._robot.x, self._robot.y, goal) or []
            if self.route:
                # From the route point nearest to the robot on: a real replan starts at the robot's pose.
                i = min(range(len(self.route)), key=lambda k: math.hypot(self.route[k][0] - self._robot.x,
                                                                         self.route[k][1] - self._robot.y))
                self.route = self.route[i:]
                return [(self._robot.x, self._robot.y, self.route[0][2])] + self.route
        points = [(self._robot.x, self._robot.y)]
        if self.detour is not None:
            points.append(self.detour)
        points.append((goal.x, goal.y))
        path: list[tuple[float, float, float]] = []
        for (x0, y0), (x1, y1) in zip(points, points[1:]):
            yaw = math.atan2(y1 - y0, x1 - x0)
            n = max(2, int(math.hypot(x1 - x0, y1 - y0) / 0.1) + 1)
            path.extend((x0 + (x1 - x0) * i / (n - 1), y0 + (y1 - y0) * i / (n - 1), yaw) for i in range(n))
        return path

    def replan(self) -> None:
        with self.lock:
            if self.goal is not None:
                self.path = self._plan(self.goal)

    def _finish(self, status: int) -> None:
        with self.lock:
            cb, self.callbacks, self.goal, self.path = self.callbacks, None, None, []
        if cb is not None:
            cb.on_result(NavResult(status=status, error_code=None, error_message=None))

    def step(self, dt: float, gate_open: bool, speed: float) -> None:
        with self.lock:
            if self.goal is None or self.callbacks is None:
                return
            goal, cb = self.goal, self.callbacks
        remaining = math.hypot(goal.x - self._robot.x, goal.y - self._robot.y)
        if self.detour is not None and math.hypot(self.detour[0] - self._robot.x, self.detour[1] - self._robot.y) < 0.02:
            self.detour = None
        tx, ty = self.detour if self.detour is not None else (goal.x, goal.y)
        if self.route:
            while len(self.route) > 1 and math.hypot(self.route[0][0] - self._robot.x, self.route[0][1] - self._robot.y) < 0.02:
                self.route.pop(0)
            tx, ty = self.route[0][0], self.route[0][1]
        leg = math.hypot(tx - self._robot.x, ty - self._robot.y)
        if remaining < 0.02 and self.detour is None:
            self.route = []
            self._finish(STATUS_SUCCEEDED)
            return
        if leg < 1e-6:
            return
        if not gate_open:
            self.blocked_since = self.blocked_since or time.monotonic()
            # Nav2's progress checker: no movement for movement_time_allowance → abort.
            if time.monotonic() - self.blocked_since > self._robot.progress_timeout_sec:
                self.blocked_since = None
                self._finish(STATUS_ABORTED)
            return
        self.blocked_since = None
        d = min(leg, speed * dt)
        self._robot.move_by((tx - self._robot.x) / leg * d, (ty - self._robot.y) / leg * d)
        cb.on_feedback(NavFeedback(distance_remaining=remaining, number_of_recoveries=0))


class SimRobot:
    def __init__(
        self,
        *,
        code: str,
        x: float,
        y: float,
        fingerprint: str,
        zones: list[dict[str, Any]],
        prefix: str,
        map_code: str,
        host: str = "127.0.0.1",
        port: int = 1883,
        speed: float = 0.4,
        progress_timeout_sec: float = 20.0,
        path_provider=None,
    ) -> None:
        self.path_provider = path_provider
        self.code, self.x, self.y, self.yaw = code, x, y, 0.0
        self.prefix, self.map_code = prefix.rstrip("/"), map_code
        self.speed, self.progress_timeout_sec = speed, progress_timeout_sec
        self.seq = SequenceClock()
        self.localized = True
        self.heartbeat_enabled = True
        self.steps: list[Step] = []
        self.gate_log: list[tuple[float, bool, str]] = []
        self.authority_msgs: list[dict[str, Any]] = []
        self.acks: list[dict[str, Any]] = []
        self.results: list[dict[str, Any]] = []
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._gate = AuthorityGateLogic(max_age_sec=0.3)
        self._tick_ok = False
        self._link_was_up = False
        self._last_replan = 0.0

        self.mqtt = ReconnectingMqttPublisher(
            host=host,
            port=port,
            client_id=f"e2e-sim-{code}",
            client_factory=default_paho_factory,
            subscribe_topic=[self.topic("command"), f"{self.prefix}/traffic/epoch"],
            on_message=self._on_message,
        )
        self.client = SemanticTrafficClient(
            robot_code=code,
            topic_prefix=self.prefix,
            fingerprint=fingerprint,
            publish_json=lambda t, b: self.mqtt.publish(t, b, qos=1),
            set_authority_ok=lambda ok: self._gate.on_authority_ok(ok, time.monotonic()),
            zones=zones,
        )
        self.nav = StraightLineNav(self)
        self.stack = build_command_stack(
            robot_code=code,
            publisher=self,
            navigator=self.nav,
            localization_provider=lambda: "LOCALIZED" if self.localized else "NOT_LOCALIZED",
            log=logging.getLogger(f"sim.{code}"),
            robot_map_version_code=map_code,
            traffic_mode="semantic_active",
            traffic_hold_active=lambda: self.client.held_for_traffic(),
        )
        self._threads: list[threading.Thread] = []

    # ------------------------------------------------------------------ lifecycle

    def topic(self, leaf: str) -> str:
        return f"{self.prefix}/robots/{self.code}/{leaf}"

    def start(self) -> None:
        self.mqtt.start()
        for target, period in ((self._heartbeat, 1.0), (self._telemetry, 0.2), (self._tick, 0.05)):
            t = threading.Thread(target=self._loop, args=(target, period), daemon=True, name=f"{self.code}-{target.__name__}")
            t.start()
            self._threads.append(t)

    def stop(self) -> None:
        self._stop.set()
        self.mqtt.stop()

    def disconnect(self) -> None:
        """Robot vanishes from the network (no heartbeat, no MQTT)."""
        self.heartbeat_enabled = False
        self.mqtt.stop()

    def reconnect(self) -> None:
        self.heartbeat_enabled = True
        self.mqtt.start()

    def _loop(self, fn, period: float) -> None:
        while not self._stop.wait(period):
            try:
                fn()
            except Exception:  # noqa: BLE001
                log.exception("%s %s failed", self.code, fn.__name__)

    # ------------------------------------------------------------------ CommandEventPublisher (router → MQTT)

    def publish_ack(self, payload: dict[str, Any]) -> None:
        self.acks.append(payload)
        self.mqtt.publish(self.topic("command_ack"), payload, qos=1)

    def publish_result(self, payload: dict[str, Any]) -> None:
        self.results.append(payload)
        self.mqtt.publish(self.topic("command_result"), payload, qos=1)

    # ------------------------------------------------------------------ downlink

    def _on_message(self, topic: str, body: dict[str, Any]) -> None:
        with self._lock:
            if topic.endswith("/traffic/epoch"):
                before = self.client.known_epoch
                self.client.on_epoch(body)
                if self.client.known_epoch != before:
                    self.publish_authority_state()
                return
            ack = self.client.handle_authority_command(body)
            if ack is not None:
                self.authority_msgs.append({"type": str(body.get("type")).upper(), **(body.get("payload") or {})})
                self.mqtt.publish(self.topic("command_ack"), ack, qos=1)
                return
            self.stack.router.handle_message(body)

    # ------------------------------------------------------------------ uplink

    def replan_through(self, point: Optional[tuple[float, float]]) -> None:
        """Nav2 replans now, through ``point`` (None: straight to the goal again)."""
        with self._lock:
            self.nav.detour = point
            self.nav.replan()
            self.publish_plan()

    def publish_plan(self) -> None:
        with self._lock:
            if self.nav.path:
                self.client.on_plan(self.nav.path, goal=self.nav.path[-1], command_id=self.stack.move.active_command_id)

    def publish_authority_state(self) -> None:
        body = {"schemaVersion": 1, "robotCode": self.code, "bootId": self.seq.boot_id, **self.client.state_report(),
                **({"poseX": self.x, "poseY": self.y, "localizationOk": True} if self.localized
                   else {"localizationOk": False})}
        self.mqtt.publish(self.topic("authority_state"), body, qos=1)

    def _heartbeat(self) -> None:
        if self.heartbeat_enabled:
            self.mqtt.publish(self.topic("heartbeat"), build_heartbeat(
                robot_code=self.code, boot_id=self.seq.boot_id, sequence=self.seq.next_heartbeat()))

    def _telemetry(self) -> None:
        if not self.heartbeat_enabled:
            return
        pose = {"x": self.x, "y": self.y, "yaw": self.yaw} if self.localized else None
        self.mqtt.publish(self.topic("telemetry"), build_telemetry(
            robot_code=self.code, boot_id=self.seq.boot_id, sequence=self.seq.next_telemetry(),
            map_version_code=self.map_code, pose=pose, battery_percent=90.0,
            navigation_status=self.stack.move.navigation_status,
            localization_status="LOCALIZED" if self.localized else "NOT_LOCALIZED",
            linear_velocity=0.0, angular_velocity=0.0, current_command_id=self.stack.move.active_command_id))

    def _tick(self) -> None:
        with self._lock:
            self.stack.router.tick()
            connected = self.mqtt.is_connected
            self.client.on_link(connected)
            if connected and not self._link_was_up:
                # Like the gateway: (re)connected → report what this robot holds so the backend can reconcile.
                self.publish_authority_state()
            self._link_was_up = connected
            if connected and self.client.state_report_due():
                self.publish_authority_state()
            active = self.stack.move.active_command_id
            if self.client.command_id is not None and active != self.client.command_id:
                self.client.on_command_finished(self.client.command_id)
            if active is not None and self.nav.path and time.monotonic() - self._last_replan >= 1.0:
                # Nav2 replans ~1 Hz (from the current pose); identical paths are debounced by the client.
                self._last_replan = time.monotonic()
                self.nav.replan()
                self.publish_plan()
            pose = (self.x, self.y, self.yaw) if self.localized else None
            if self.client.localization_regained(pose is not None) and connected:
                self.publish_authority_state()
            ok = self.client.tick(pose)
            self.gate_log.append((time.monotonic(), ok, self.client.last_block_reason))
            gate_open = self._gate.decide(time.monotonic()).forward
            self._tick_ok = ok
            self.nav.step(0.05, gate_open, self.speed)
            if self.client.grant is not None and self.localized:
                cleared = self.client.resources_cleared_at(self.x, self.y)
                if cleared:
                    self.client.publish_cleared_resources(cleared, x=self.x, y=self.y, yaw=self.yaw, localization_ok=True)

    def move_by(self, dx: float, dy: float) -> None:
        self.x += dx
        self.y += dy
        if dx or dy:
            self.yaw = math.atan2(dy, dx)
        self.steps.append(Step(time.monotonic(), self.x, self.y, self._tick_ok))

    # ------------------------------------------------------------------ assertions helpers

    def moved_while_gate_closed(self) -> bool:
        """True if the pose ever advanced in a tick whose traffic decision was 'blocked' (must never happen)."""
        return any(not s.gate_open for s in self.steps)
