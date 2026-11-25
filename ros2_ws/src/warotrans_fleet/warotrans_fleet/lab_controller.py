"""Fault Injection Lab inside the gateway (ROS side effects of lab_faults.LabFaults).

Created only when the gateway runs with lab_enabled (LAB_ENABLED=true on a test robot); otherwise nothing here exists and
lab messages are ignored. Side effects:
  IOT-01  ReconnectingMqttPublisher.cut_until(TTL end)
  IOT-02  the lidar driver hangs: sllidar_node is suspended (SIGSTOP) and resumed (SIGCONT) at the end, so /scan
          really goes silent (its /stop_motor service does not stop /scan: lab 2026-11-24)
  IOT-03  /lab/faults (latched JSON) tells the evidence agent the clock offset to report
  NAV-01  MoveCommandHandler.goal_rewrite -> unreachable goal for the next N Nav2 goals
  NAV-02  CommandRouter.interceptor accepts the next NAVIGATE_TO_POSE without driving; FAILED when the TTL ends
  MAP-01  the semantic traffic client proposes with a fake fingerprint, restored at the end
  OPS-04  CommandRouter.after_result: 2 s after the next result, the same command reported with the opposite outcome
"""

from __future__ import annotations

import json
import os
import signal
import subprocess
import threading
import time
from typing import Any, Callable, Optional

from warotrans_fleet.command_contract import NAVIGATE_TO_POSE, OUTCOME_FAILED, ERROR_NAVIGATION_FAILED
from warotrans_fleet.command_router import AckDecision
from warotrans_fleet.lab_faults import ActiveFault, LabFaults, LabMessageError, contradicting_outcome
from warotrans_fleet.payload_builder import new_message_id, utc_now_iso


class LabController:
    def __init__(self, node, *, mqtt, router, move, semantic_client: Callable[[], Any]) -> None:
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import Empty

        self._node = node
        self._log = node.get_logger()
        self._mqtt = mqtt
        self._router = router
        self._semantic_client = semantic_client
        self._faults = LabFaults(enabled=True)
        self._lock = threading.Lock()
        self._frozen: dict[str, Any] = {}  # injection id -> CommandSession held by NAV-02
        self._real_fingerprint: Optional[str] = None
        self._String = String
        self._Empty = Empty
        self._stop_motor = node.create_client(Empty, "stop_motor")
        self._start_motor = node.create_client(Empty, "start_motor")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._faults_pub = node.create_publisher(String, "/lab/faults", qos)
        router.interceptor = self._intercept
        router.after_result = self._after_result
        move.goal_rewrite = self._divert
        node.create_timer(1.0, self._tick)
        self._publish_state()
        self._log.warning("FAULT INJECTION LAB ENABLED on this robot (test bench only)")

    # ------------------------------------------------------------------ inbound
    def on_message(self, body: dict) -> None:
        try:
            with self._lock:
                kind, fault = self._faults.handle(body, time.monotonic())
        except LabMessageError as exc:
            self._log.warning(f"[Lab] refused: {exc}")
            return
        if fault is None:
            return
        self._log.warning(f"[Lab] {kind} {fault.scenario_id} injection={fault.injection_id} "
                          f"ttl_left={fault.expires_at - time.monotonic():.0f}s params={fault.parameters}")
        if kind == "inject":
            self._apply(fault)
        else:
            self._undo(fault)
        self._publish_state()

    def _tick(self) -> None:
        with self._lock:
            ended = self._faults.expire(time.monotonic())
        for fault in ended:
            self._log.warning(f"[Lab] expired {fault.scenario_id} injection={fault.injection_id}")
            self._undo(fault)
        if ended:
            self._publish_state()

    # ------------------------------------------------------------------ side effects
    def _apply(self, fault: ActiveFault) -> None:
        sid = fault.scenario_id
        if sid == "IOT-01":
            self._mqtt.cut_until(fault.expires_at)
        elif sid == "IOT-02":
            self._signal_lidar(signal.SIGSTOP)
        elif sid == "MAP-01":
            client = self._semantic_client()
            if client is not None:
                if self._real_fingerprint is None:
                    self._real_fingerprint = client.fingerprint
                client.fingerprint = self._faults.fingerprint(self._real_fingerprint)

    def _undo(self, fault: ActiveFault) -> None:
        sid = fault.scenario_id
        if sid == "IOT-01":
            self._mqtt.cut_until(0.0)
        elif sid == "IOT-02":
            self._signal_lidar(signal.SIGCONT)
        elif sid == "MAP-01":
            client = self._semantic_client()
            if client is not None and self._real_fingerprint is not None:
                client.fingerprint = self._real_fingerprint
            self._real_fingerprint = None
        elif sid == "NAV-02":
            session = self._frozen.pop(fault.injection_id, None)
            if session is not None and not session.is_completed:
                session.complete(OUTCOME_FAILED, error_code=ERROR_NAVIGATION_FAILED,
                                 error_message="navigation never started (lab freeze ended)")

    def _signal_lidar(self, sig: int) -> None:
        """Suspend / resume the lidar driver process of this robot (same user as the gateway)."""
        try:
            out = subprocess.run(["pgrep", "-f", "sllidar_node"], capture_output=True, text=True, timeout=5).stdout
        except (OSError, subprocess.SubprocessError) as exc:
            self._log.error(f"[Lab] cannot find the lidar driver: {exc}")
            return
        pids = [int(p) for p in out.split() if p.strip().isdigit()]
        for pid in pids:
            try:
                os.kill(pid, sig)
            except OSError as exc:
                self._log.error(f"[Lab] signal {sig} to lidar pid {pid}: {exc}")
        self._log.warning(f"[Lab] lidar driver {'suspended' if sig == signal.SIGSTOP else 'resumed'} (pids {pids})")

    def _call(self, client, name: str) -> None:
        if not client.wait_for_service(timeout_sec=1.0):
            self._log.error(f"[Lab] lidar service {name} not available")
            return
        client.call_async(self._Empty.Request())
        self._log.warning(f"[Lab] lidar {name} called")

    def _publish_state(self) -> None:
        with self._lock:
            state = {"clockOffsetSec": self._faults.clock_offset_s(), "active": self._faults.summary(),
                     "at": utc_now_iso()}
        self._faults_pub.publish(self._String(data=json.dumps(state)))

    # ------------------------------------------------------------------ hooks
    def _divert(self) -> Optional[tuple[float, float]]:
        with self._lock:
            return self._faults.divert_goal()

    def _intercept(self, command, session) -> Optional[AckDecision]:
        if command.command_type != NAVIGATE_TO_POSE:
            return None
        with self._lock:
            fault = self._faults.freeze_next_move()
        if fault is None:
            return None
        self._frozen[fault.injection_id] = session
        self._log.warning(f"[Lab] NAV-02 holds command {command.command_id} without driving")
        return AckDecision.accept()

    def _after_result(self, payload: dict[str, Any]) -> None:
        with self._lock:
            contradict = self._faults.contradict_result()
        if not contradict:
            return
        late = dict(payload, messageId=new_message_id(), outcome=contradicting_outcome(str(payload.get("outcome"))),
                    errorCode=None, errorMessage=None)

        def _send() -> None:
            late["sentAt"] = utc_now_iso()
            self._router.publish_raw_result(late)
            self._log.warning(f"[Lab] OPS-04 sent {late['outcome']} again for command {late.get('commandId')}")

        threading.Timer(2.0, _send).start()
