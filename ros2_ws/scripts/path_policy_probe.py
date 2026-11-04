#!/usr/bin/env python3
"""Positive evidence for path_policy_validator (subscribe first, then publish).

Retries until expected reason matches so concurrent Nav2 /plan|/goal_pose traffic
does not produce false FAIL from racey topic echoes.
"""

from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as NavPath
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

QOS = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
)


class Probe(Node):
    def __init__(self) -> None:
        super().__init__(
            "path_policy_probe",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.ok = None
        self.reason = None
        self.create_subscription(Bool, "/path_policy/ok", self._on_ok, QOS)
        self.create_subscription(String, "/path_policy/reason", self._on_reason, QOS)
        # Dedicated topics — publishing /goal_pose would drive bt_navigator into KEEP_OUT/NO_STOP.
        self.goal_pub = self.create_publisher(PoseStamped, "/path_policy/check_goal", QOS)
        self.plan_pub = self.create_publisher(NavPath, "/path_policy/check_plan", QOS)

    def _on_ok(self, msg: Bool) -> None:
        self.ok = bool(msg.data)

    def _on_reason(self, msg: String) -> None:
        self.reason = str(msg.data)

    def spin_for(self, sec: float) -> None:
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def drain(self, sec: float = 0.4) -> None:
        self.ok = None
        self.reason = None
        self.spin_for(sec)
        self.ok = None
        self.reason = None

    def wait_reason(self, timeout: float = 4.0) -> tuple[bool | None, str | None]:
        self.ok = None
        self.reason = None
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.reason is not None:
                return self.ok, self.reason
        return self.ok, self.reason


def yaw_quat(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def ring_centroid(ring: list[list[float]]) -> tuple[float, float]:
    return sum(p[0] for p in ring) / len(ring), sum(p[1] for p in ring) / len(ring)


def main() -> int:
    base = Path(
        "/mnt/e/programming/my_project/warotrans/ros2_ws/src/warotrans_simulation/maps/map_one_v001"
    )
    ko_zones = json.loads((base / "keepout.json").read_text(encoding="utf-8"))["zones"]
    ns_zones = json.loads((base / "no_stop.json").read_text(encoding="utf-8"))["zones"]
    kx, ky = ring_centroid(ko_zones[0]["rings"][0])
    # Prefer ZONE-CENTER-NO-STOP (clearly interior, not shelf overlap)
    ns_zone = next((z for z in ns_zones if "CENTER" in str(z.get("code") or "")), ns_zones[0])
    nx, ny = ring_centroid(ns_zone["rings"][0])

    results: list[dict] = []
    rclpy.init()
    node = Probe()
    try:
        node.spin_for(2.0)

        def pub_goal(x: float, y: float, yaw: float = 0.0) -> None:
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.pose.position.x = x
            msg.pose.position.y = y
            z, w = yaw_quat(yaw)
            msg.pose.orientation.z = z
            msg.pose.orientation.w = w
            node.goal_pub.publish(msg)

        def pub_plan(poses: list[tuple[float, float, float]]) -> None:
            path = NavPath()
            path.header.frame_id = "map"
            path.header.stamp = node.get_clock().now().to_msg()
            for x, y, yaw in poses:
                p = PoseStamped()
                p.header = path.header
                p.pose.position.x = x
                p.pose.position.y = y
                z, w = yaw_quat(yaw)
                p.pose.orientation.z = z
                p.pose.orientation.w = w
                path.poses.append(p)
            node.plan_pub.publish(path)

        def expect_case(
            case: str,
            *,
            expect_ok: bool,
            expect_reason_substr: str,
            publish_fn,
            attempts: int = 4,
        ) -> None:
            last_ok, last_reason = None, None
            matched = False
            for _ in range(attempts):
                node.drain(0.35)
                publish_fn()
                ok, reason = node.wait_reason(timeout=3.5)
                last_ok, last_reason = ok, reason
                if reason is not None and expect_reason_substr in reason and bool(ok) == expect_ok:
                    matched = True
                    break
                time.sleep(0.2)
            results.append(
                {
                    "case": case,
                    "ok": last_ok,
                    "reason": last_reason,
                    "expect_ok": expect_ok,
                    "expect_reason_substr": expect_reason_substr,
                    "matched": matched,
                }
            )

        expect_case(
            "valid_goal",
            expect_ok=True,
            expect_reason_substr="goal_ok",
            publish_fn=lambda: pub_goal(-1.5, 0.2, 0.0),
        )
        expect_case(
            "no_stop_goal",
            expect_ok=False,
            expect_reason_substr="NO_STOP_GOAL",
            publish_fn=lambda: pub_goal(nx, ny, 0.0),
        )
        expect_case(
            "keepout_goal",
            expect_ok=False,
            expect_reason_substr="KEEP_OUT_GOAL",
            publish_fn=lambda: pub_goal(kx, ky, 0.0),
        )
        # Stay east of ZONE-CENTER-NO-STOP (x<=0.911) so plan end is not NO_STOP.
        expect_case(
            "oneway_reverse_plan",
            expect_ok=False,
            expect_reason_substr="ONE_WAY_VIOLATION",
            publish_fn=lambda: pub_plan([(1.55, -0.327, math.pi), (1.15, -0.327, math.pi)]),
        )
        expect_case(
            "oneway_forward_plan",
            expect_ok=True,
            expect_reason_substr="path_ok",
            publish_fn=lambda: pub_plan([(1.15, -0.327, 0.0), (1.55, -0.327, 0.0)]),
        )

        out = Path("/tmp/nav2_live_artifacts/path_policy_probe.json")
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(results, indent=2))
        return 0 if all(r.get("matched") for r in results) else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
