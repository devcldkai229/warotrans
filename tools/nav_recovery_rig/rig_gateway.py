#!/usr/bin/env python3
"""Stand-in for robot_gateway in the recovery / V004 rig. Named robot_gateway because the gate trusts only that publisher.

  --authority ok|closed     what traffic/authority_ok says, nothing else (recovery rig)
  --authority semantic      the REAL SemanticTrafficClient drives the gate, against a backend simulated in this process:
                            it reserves the same cells (collect_resources), renews, can answer late (--rtt-s), rejects a
                            plan through keep-out like TrafficState does, and on request withdraws the grant for a while
                            (--deny-from-s, --deny-for-s: REVOKE, then no answer until the end of the window).
  --recovery ok|<reason>    what traffic/authorize_backup and traffic/authorize_spin answer (ok|closed only)
  --recovery-backup <why>   override for backup only (e.g. OUTSIDE_RESERVATION)
  --events <file>           semantic: JSONL of every gate change and backend decision, read back by run_rig.analyse
"""

from __future__ import annotations

import argparse
import json
import math
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path as FsPath
from typing import Any, Optional

import rclpy
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from warotrans_fleet.semantic_traffic_client import SemanticGeometry, SemanticTrafficClient, collect_resources, disc_hits_ring

COMMAND_ID = "99999999-9999-9999-9999-999999999999"
ROBOT = "RBT-RIG"
TOPIC_PREFIX = "warotrans-rig/v1"
RENEW_PERIOD_S = 1.5
GRANT_TTL_S = 30.0
BACKUP_M = 0.10
# traffic.RealRobot: cell 0.25 m, horizon 4 m, path tolerance 0.15 m, footprint half 0.18 x 0.15 m.
FOOTPRINT_RADIUS_M = math.hypot(0.18, 0.15)
BODY_HALF_WIDTH_M = 0.15


def geometry_for(lane_half_width: float) -> dict[str, float]:
    """traffic.RealRobot, or with lane rectangles the settings of a two-robot run (0.1 m cells)."""
    cell = 0.1 if lane_half_width > 0 else 0.25
    return {
        "cellSizeMeters": cell,
        "horizonMeters": 4.0,
        "sampleStepMeters": cell / 4.0,
        "reserveRadiusMeters": FOOTPRINT_RADIUS_M + 0.15 + cell / 8.0,
        "footprintRadiusMeters": FOOTPRINT_RADIUS_M,
        "laneHalfWidthMeters": lane_half_width,
        "footprintHalfLengthMeters": 0.18,
        "footprintHalfWidthMeters": BODY_HALF_WIDTH_M,
    }


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _iso(ts: float) -> str:
    return datetime.fromtimestamp(ts, tz=timezone.utc).isoformat().replace("+00:00", "Z")


def _zones(bundle: FsPath, name: str) -> list[dict[str, Any]]:
    path = bundle / name
    return json.loads(path.read_text(encoding="utf-8")).get("zones", []) if path.is_file() else []


class SimulatedBackend:
    """What the coordinator does for one robot with the floor to itself. No queue: conflicts are --deny-* windows."""

    def __init__(self, zones: list[dict[str, Any]], keepout: list[dict[str, Any]], fingerprint: str, a: argparse.Namespace,
                 send, log) -> None:
        self.zones, self.keepout, self.fingerprint, self.a, self.send, self.log = zones, keepout, fingerprint, a, send, log
        self.payload = geometry_for(a.lane_half_width)
        self.geometry = SemanticGeometry.from_payload(self.payload)
        self.inbox: list[tuple[float, dict[str, Any]]] = []
        self.first_plan_at: Optional[float] = None
        self.authority: Optional[dict[str, Any]] = None
        self.seq = 0
        self.denied = False
        self.last_renew = 0.0
        self.latest: Optional[dict[str, Any]] = None

    def on_publish(self, topic: str, body: dict[str, Any]) -> None:
        if topic.endswith("/traffic_plan"):
            self.inbox.append((time.monotonic() + self.a.rtt_s, body))

    def _hits_keepout(self, poses: list[dict[str, Any]]) -> bool:
        for a, b in zip(poses, poses[1:]):
            steps = max(1, math.ceil(math.hypot(b["x"] - a["x"], b["y"] - a["y"]) / self.geometry.sample_step))
            for k in range(steps + 1):
                x, y = a["x"] + (b["x"] - a["x"]) * k / steps, a["y"] + (b["y"] - a["y"]) * k / steps
                for z in self.keepout:
                    for ring in z.get("rings") or []:
                        if disc_hits_ring(x, y, BODY_HALF_WIDTH_M, ring):
                            return True
        return False

    def _grant(self, plan: dict[str, Any]) -> None:
        poses = [(p["x"], p["y"], p["yaw"]) for p in plan["poses"]]
        self.seq += 1
        self.authority = {
            "targetCommandId": plan["commandId"],
            "authorityId": str(uuid.uuid4()),
            "epoch": 1,
            "seq": self.seq,
            "semanticMapFingerprint": self.fingerprint,
            "planRevision": plan["planRevision"],
            "planHash": plan["planHash"],
            "resourceIds": sorted(collect_resources(poses, self.zones, self.geometry)),
            "expiresAt": _iso(time.time() + GRANT_TTL_S),
            "geometry": self.payload,
        }
        self.send("EXTEND_AUTHORITY", self.authority)
        self.log("grant", rev=plan["planRevision"], resources=len(self.authority["resourceIds"]))

    def tick(self) -> None:
        now = time.monotonic()
        if self.first_plan_at is not None and self.a.deny_for_s > 0:
            age = now - self.first_plan_at
            denied = self.a.deny_from_s <= age < self.a.deny_from_s + self.a.deny_for_s
            if denied and not self.denied:
                self.denied = True
                self.log("deny_start")
                if self.authority is not None:
                    self.send("REVOKE_AUTHORITY", {"authorityId": self.authority["authorityId"]})
                    self.authority = None
            elif self.denied and not denied:
                self.denied = False
                self.log("deny_end")
                if self.latest is not None:
                    self._grant(self.latest)
        while self.inbox and self.inbox[0][0] <= now:
            _, plan = self.inbox.pop(0)
            if self.first_plan_at is None:
                self.first_plan_at = now
            self.log("propose", rev=plan["planRevision"])
            if self._hits_keepout(plan["poses"]):
                self.log("keepout_reject", rev=plan["planRevision"])
                self.authority = None
                self.latest = None
                continue
            self.latest = plan
            if self.denied:
                continue
            same = self.authority is not None and self.authority["planHash"] == plan["planHash"] \
                and self.authority["planRevision"] == plan["planRevision"]
            if same:
                self.seq += 1
                self.authority = {**self.authority, "seq": self.seq, "expiresAt": _iso(time.time() + GRANT_TTL_S)}
                self.send("EXTEND_AUTHORITY", self.authority)
            else:
                self._grant(plan)
        if self.authority is not None and now - self.last_renew >= RENEW_PERIOD_S:
            self.last_renew = now
            self.seq += 1
            self.authority = {**self.authority, "seq": self.seq, "expiresAt": _iso(time.time() + GRANT_TTL_S)}
            self.send("RENEW_AUTHORITY", {
                "authorityId": self.authority["authorityId"], "epoch": 1, "seq": self.seq,
                "expiresAt": self.authority["expiresAt"],
            })


class SemanticGateway(Node):
    def __init__(self, a: argparse.Namespace) -> None:
        super().__init__("robot_gateway")
        bundle = FsPath(a.bundle)
        manifest = json.loads((bundle / "semantic_manifest.json").read_text(encoding="utf-8"))
        zones = _zones(bundle, "single_occupancy.json")  # as robot_gateway_node: the zones the backend reserves
        self._events = open(a.events, "a", encoding="utf-8", buffering=1) if a.events else None
        self._ok_pub = self.create_publisher(Bool, "traffic/authority_ok", 10)
        self._reason_pub = self.create_publisher(String, "traffic/last_block_reason", 10)
        self._pose: Optional[tuple[float, float, float]] = None
        self._last_ok: Optional[bool] = None
        self._last_reason: Optional[str] = None
        self.backend = SimulatedBackend(zones, _zones(bundle, "keepout.json"), manifest["semanticMapFingerprint"], a,
                                        self._deliver, self._log_event)
        self.client = SemanticTrafficClient(
            robot_code=ROBOT,
            topic_prefix=TOPIC_PREFIX,
            fingerprint=manifest["semanticMapFingerprint"],
            publish_json=self.backend.on_publish,
            set_authority_ok=self._publish_ok,
            zones=zones,
        )
        self.create_subscription(Path, "plan", self._on_plan, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_timer(0.05, self._tick)
        self.create_service(Trigger, "traffic/authorize_backup", self._answer("backup", BACKUP_M))
        self.create_service(Trigger, "traffic/authorize_spin", self._answer("spin", 0.0))

    def _log_event(self, kind: str, **fields: Any) -> None:
        if self._events is not None:
            self._events.write(json.dumps({"wall": round(time.time(), 3), "e": kind, **fields}) + "\n")

    def _deliver(self, cmd_type: str, payload: dict[str, Any]) -> None:
        self.client.handle_authority_command({"type": cmd_type, "commandId": str(uuid.uuid4()), "payload": payload})

    def _publish_ok(self, ok: bool) -> None:
        self._ok_pub.publish(Bool(data=bool(ok)))

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        self._pose = (float(p.position.x), float(p.position.y), _yaw_from_quat(p.orientation))

    def _on_plan(self, msg: Path) -> None:
        poses = [(float(s.pose.position.x), float(s.pose.position.y), _yaw_from_quat(s.pose.orientation)) for s in msg.poses]
        if len(poses) >= 2:
            self.client.on_plan(poses, goal=poses[-1], command_id=COMMAND_ID)

    def _answer(self, kind: str, distance: float):
        def cb(_req, res):
            reason = self.client.recovery_block_reason(self._pose, kind, distance)
            res.success = reason is None
            res.message = reason or ""
            self._log_event("recovery", manoeuvre=kind, reason=reason)
            return res
        return cb

    def _tick(self) -> None:
        self.backend.tick()
        ok = self.client.tick(self._pose)
        reason = "OK" if ok else self.client.last_block_reason
        self._reason_pub.publish(String(data=reason))
        if ok != self._last_ok or reason != self._last_reason:
            self._log_event("gate", ok=ok, reason=reason, rev=self.client.plan_revision)
            self._last_ok, self._last_reason = ok, reason
        if self._pose is not None:
            cleared = self.client.resources_cleared_at(self._pose[0], self._pose[1])
            if cleared:
                self._log_event("cleared", n=len(cleared))


class SimpleGateway(Node):
    def __init__(self, a: argparse.Namespace) -> None:
        super().__init__("robot_gateway")
        self.a = a
        self.pub = self.create_publisher(Bool, "traffic/authority_ok", 10)
        self.reason_pub = self.create_publisher(String, "traffic/last_block_reason", 10)
        self.create_timer(0.05, self._tick)
        # A closed gate means no grant: the real client refuses recovery then too.
        closed = "NO_GRANT" if a.authority != "ok" else ""
        self.create_service(Trigger, "traffic/authorize_backup", self._answer(closed or a.recovery_backup or a.recovery))
        self.create_service(Trigger, "traffic/authorize_spin", self._answer(closed or a.recovery))

    def _tick(self) -> None:
        ok = self.a.authority == "ok"
        self.pub.publish(Bool(data=ok))
        self.reason_pub.publish(String(data="OK" if ok else "NO_GRANT"))

    @staticmethod
    def _answer(value: str):
        def cb(_req, res):
            res.success = value == "ok"
            res.message = "" if res.success else value
            return res
        return cb


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    p.add_argument("--authority", default="ok", choices=("ok", "closed", "semantic"))
    p.add_argument("--recovery", default="ok")
    p.add_argument("--recovery-backup", default="")
    p.add_argument("--bundle", default="", help="semantic: map bundle directory")
    p.add_argument("--lane-half-width", type=float, default=0.0, help="semantic: reserve lane rectangles (two-robot run)")
    p.add_argument("--events", default="", help="semantic: JSONL of gate changes and backend decisions")
    p.add_argument("--rtt-s", type=float, default=0.1, help="semantic: delay before the backend answers a proposal")
    p.add_argument("--deny-from-s", type=float, default=0.0, help="semantic: seconds after the first plan")
    p.add_argument("--deny-for-s", type=float, default=0.0, help="semantic: how long the backend grants nothing")
    return p


def main() -> None:
    a = parser().parse_args()
    if a.authority == "semantic" and not a.bundle:
        raise SystemExit("--authority semantic needs --bundle")
    rclpy.init()
    node = SemanticGateway(a) if a.authority == "semantic" else SimpleGateway(a)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
