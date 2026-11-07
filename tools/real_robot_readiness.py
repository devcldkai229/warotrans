#!/usr/bin/env python3
"""Real-robot readiness for Semantic traffic dispatch. Run ON THE PI with the stack up. READ-ONLY.

It only subscribes, reads parameters and queries lifecycle state. It never publishes, never sends a goal and
never touches /cmd_vel. Each check prints PASS / FAIL; the exit code is 0 only when everything passes.

    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
    python3 ~/ros2_ws/../tools/real_robot_readiness.py \
        --manifest $(ros2 pkg prefix warotrans_navigation)/share/warotrans_navigation/maps/map_one_v002/semantic_manifest.json \
        --mqtt-host <broker LAN IP>

A robot is only ready for supervised motion when every line passes, INCLUDING the emergency stop, which no
software check can prove: it needs a hardware stop that cuts motor power and was tested by a person
(--estop-tested is that person's statement, not something this script measures).
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import subprocess
from email.utils import parsedate_to_datetime
import urllib.request
import sys
import time
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rcl_interfaces.srv import GetParameters
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class Check:
    name: str
    ok: bool
    detail: str


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("warotrans_readiness_probe")
        self.scan_stamps: list[float] = []
        self.cmd_vel: list[Twist] = []
        self.authority_ok: list[bool] = []
        self.amcl: PoseWithCovarianceStamped | None = None
        self.create_subscription(LaserScan, "/scan", lambda _m: self.scan_stamps.append(time.monotonic()),
                                 qos_profile_sensor_data)
        self.create_subscription(Twist, "/cmd_vel_motor", self.cmd_vel.append, 10)
        self.create_subscription(Bool, "/traffic/authority_ok", lambda m: self.authority_ok.append(bool(m.data)), 10)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, latched)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.amcl = msg

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def params(self, node: str, names: list[str], timeout: float = 3.0) -> dict | None:
        client = self.create_client(GetParameters, f"{node}/get_parameters")
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            return None
        out = {}
        for name, value in zip(names, future.result().values):
            out[name] = {1: value.bool_value, 2: value.integer_value, 3: value.double_value,
                         4: value.string_value}.get(value.type)
        return out

    def lifecycle_active(self, manager: str, timeout: float = 3.0) -> bool | None:
        client = self.create_client(Trigger, f"{manager}/is_active")
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        return None if future.result() is None else bool(future.result().success)


def run(args: argparse.Namespace) -> list[Check]:
    checks: list[Check] = []
    add = lambda name, ok, detail="": checks.append(Check(name, bool(ok), detail))  # noqa: E731

    manifest = json.loads(open(args.manifest, encoding="utf-8").read())
    bundle = os.path.dirname(os.path.abspath(args.manifest))
    fingerprint = manifest["semanticMapFingerprint"]

    for dev in ("/dev/warotrans", "/dev/rplidar"):
        add(f"device {dev}", os.path.exists(dev), os.path.realpath(dev) if os.path.exists(dev) else "missing")

    teleop = subprocess.run(["systemctl", "is-active", "warotrans-teleop"], capture_output=True, text=True).stdout.strip()
    add("warotrans-teleop.service stopped (fights Nav2 for /cmd_vel)", teleop != "active", teleop or "?")

    if args.backend_url:
        # No NTP on the hotspot network and no RTC battery: the clock that matters is the backend's.
        try:
            with urllib.request.urlopen(args.backend_url.rstrip("/") + "/api/fleet/ping", timeout=3.0) as resp:
                skew = time.time() - parsedate_to_datetime(resp.headers["Date"]).timestamp()
            add("clock within 2 s of the backend (grant expiresAt)", abs(skew) <= 2.0, f"{skew:+.1f} s")
        except (OSError, TypeError, ValueError) as exc:
            add("clock within 2 s of the backend (grant expiresAt)", False, str(exc))
    else:
        synced = subprocess.run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"],
                                capture_output=True, text=True).stdout.strip()
        add("clock NTP-synchronized (grant expiresAt)", synced == "yes", synced or "?")

    try:
        with socket.create_connection((args.mqtt_host, args.mqtt_port), timeout=3.0):
            add(f"MQTT broker {args.mqtt_host}:{args.mqtt_port} reachable", True)
    except OSError as exc:
        add(f"MQTT broker {args.mqtt_host}:{args.mqtt_port} reachable", False, str(exc))

    rclpy.init()
    probe = Probe()
    try:
        probe.spin_for(args.window)

        for manager in ("lifecycle_manager_localization", "lifecycle_manager_navigation"):
            active = probe.lifecycle_active(manager)
            add(f"{manager} active", active is True, "no service" if active is None else str(active))

        planner = probe.params("planner_server", ["GridBased.plugin", "GridBased.lanes_json"]) or {}
        add("planner = LaneAwareNavfn", planner.get("GridBased.plugin") == "warotrans_lane_planner::LaneAwareNavfn",
            str(planner.get("GridBased.plugin")))
        lanes_json = planner.get("GridBased.lanes_json") or ""
        same_lanes = bool(lanes_json) and os.path.realpath(lanes_json) == os.path.realpath(os.path.join(bundle, "lanes.json"))
        add("planner lanes_json = bundle lanes.json (ONE_WAY)", same_lanes, lanes_json or "empty")

        cm = probe.params("collision_monitor", ["cmd_vel_out_topic"]) or {}
        add("collision_monitor -> cmd_vel_safe (authority gate in path)", cm.get("cmd_vel_out_topic") == "cmd_vel_safe",
            str(cm.get("cmd_vel_out_topic")))
        gate = probe.params("cmd_vel_authority_gate", ["input_topic", "output_topic", "authority_ok_topic",
                                                       "required_authority_publisher", "estop_topic"])
        add("cmd_vel_authority_gate running", gate is not None, str(gate))
        gate = gate or {}
        add("gate forwards to cmd_vel_motor (motor-only topic)", gate.get("output_topic") == "cmd_vel_motor",
            str(gate.get("output_topic")))
        add("gate trusts authority_ok only from robot_gateway", gate.get("required_authority_publisher") == "robot_gateway",
            str(gate.get("required_authority_publisher")))
        bridge = probe.params("esp32_bridge_node", ["cmd_vel_topic", "required_cmd_vel_publisher", "motors_enabled"]) or {}
        if args.expect_motors_disabled:
            add("ESP32 bridge motors_enabled = false (non-motion check)", bridge.get("motors_enabled") is False,
                str(bridge.get("motors_enabled")))
        add("ESP32 bridge drives only from /cmd_vel_motor", bridge.get("cmd_vel_topic") == "/cmd_vel_motor",
            str(bridge.get("cmd_vel_topic")))
        add("ESP32 bridge requires the gate as sole publisher",
            bridge.get("required_cmd_vel_publisher") == "cmd_vel_authority_gate", str(bridge.get("required_cmd_vel_publisher")))
        motor_pubs = [i.node_name for i in probe.get_publishers_info_by_topic("/cmd_vel_motor")]
        add("/cmd_vel_motor published by the gate only (no bypass: teleop, CLI)", motor_pubs == ["cmd_vel_authority_gate"],
            str(motor_pubs))
        ok_pubs = [i.node_name for i in probe.get_publishers_info_by_topic("/traffic/authority_ok")]
        add("/traffic/authority_ok published by robot_gateway only", ok_pubs == ["robot_gateway"], str(ok_pubs))

        gw = probe.params("robot_gateway", ["traffic_mode", "semantic_map_fingerprint", "map_version_code",
                                            "teleop_enabled", "require_amcl_converged", "robot_code"]) or {}
        add("gateway traffic_mode = semantic_active", gw.get("traffic_mode") == "semantic_active", str(gw.get("traffic_mode")))
        add("gateway fingerprint = backend bundle", gw.get("semantic_map_fingerprint") == fingerprint,
            f"{str(gw.get('semantic_map_fingerprint'))[:12]} vs {fingerprint[:12]}")
        add("gateway map_version_code = bundle", gw.get("map_version_code") == manifest["mapCode"], str(gw.get("map_version_code")))
        add("gateway teleop disabled (bypasses gate)", gw.get("teleop_enabled") is False, str(gw.get("teleop_enabled")))
        add("gateway requires AMCL convergence", gw.get("require_amcl_converged") is True, str(gw.get("require_amcl_converged")))

        stamps = probe.scan_stamps
        hz = (len(stamps) - 1) / (stamps[-1] - stamps[0]) if len(stamps) > 2 and stamps[-1] > stamps[0] else 0.0
        add("/scan >= 5 Hz", hz >= 5.0, f"{hz:.1f} Hz")

        try:
            tf = probe.tf.lookup_transform("map", "base_link", Time(), timeout=Duration(seconds=1.0))
            age = (probe.get_clock().now() - Time.from_msg(tf.header.stamp)).nanoseconds / 1e9
            add("TF map->base_link fresh (< 0.5 s)", 0.0 <= age < 0.5, f"age {age:.2f}s")
        except TransformException as exc:
            add("TF map->base_link fresh (< 0.5 s)", False, str(exc))

        if probe.amcl is None:
            add("AMCL pose received", False, "no /amcl_pose")
        else:
            cov = probe.amcl.pose.covariance
            xy, yaw = math.sqrt(max(cov[0], cov[7])), math.sqrt(cov[35])
            p = probe.amcl.pose.pose.position
            near_boot = math.hypot(p.x, p.y) < 0.05
            add("AMCL covariance bounded (σxy <= 0.5 m, σyaw <= 0.5 rad)", xy <= 0.5 and yaw <= 0.5,
                f"σxy {xy:.3f} σyaw {yaw:.3f}")
            add("AMCL pose is not the provisional boot pose (0, 0)", not near_boot, f"({p.x:.2f}, {p.y:.2f})")
            if args.home_x is not None and args.home_y is not None:
                d = math.hypot(p.x - args.home_x, p.y - args.home_y)
                add(f"robot localized at its home endpoint (<= {args.home_tolerance} m)", d <= args.home_tolerance,
                    f"{d:.2f} m from ({args.home_x}, {args.home_y})")

        nodes = [name for name, _ in probe.get_node_names_and_namespaces()]
        add("recovery_guard running (BackUp/Spin only through it)", "recovery_guard" in nodes, "")
        ekf_on = "ekf_filter_node" in nodes
        amcl = probe.params("amcl", ["alpha1"]) or {}
        alpha1 = amcl.get("alpha1")
        add("AMCL motion noise matches the odometry source (EKF 0.15, wheel-only 0.3)",
            alpha1 is not None and math.isclose(alpha1, 0.15 if ekf_on else 0.3, abs_tol=1e-6),
            f"ekf={'on' if ekf_on else 'off'} alpha1={alpha1}")

        add("traffic/authority_ok published and False (no grant held)",
            bool(probe.authority_ok) and not any(probe.authority_ok),
            f"{len(probe.authority_ok)} samples, any True={any(probe.authority_ok)}")
        moving = [t for t in probe.cmd_vel if abs(t.linear.x) > 1e-6 or abs(t.angular.z) > 1e-6]
        add("/cmd_vel_motor carries no motion", not moving, f"{len(probe.cmd_vel)} msgs, {len(moving)} non-zero")
    finally:
        probe.destroy_node()
        rclpy.shutdown()

    add("hardware emergency stop (cuts motor power) present and tested by a person", args.estop_tested,
        "operator-stated" if args.estop_tested else
        "NONE: firmware has only a 300 ms cmd watchdog; /warotrans/estop is software only (defence in depth), "
        "no hardware E-stop")
    return checks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--manifest", required=True, help="semantic_manifest.json the backend also loads")
    parser.add_argument("--mqtt-host", required=True)
    parser.add_argument("--mqtt-port", type=int, default=1883)
    parser.add_argument("--backend-url", default="", help="compare the Pi clock with the backend (http://IP:5090)")
    parser.add_argument("--home-x", type=float, default=None, help="home endpoint x (map frame), e.g. EP-PARKING-A")
    parser.add_argument("--home-y", type=float, default=None)
    parser.add_argument("--home-tolerance", type=float, default=0.30)
    parser.add_argument("--expect-motors-disabled", action="store_true", help="non-motion commissioning run")
    parser.add_argument("--window", type=float, default=4.0, help="seconds to observe topics")
    parser.add_argument("--estop-tested", action="store_true",
                        help="a person confirms a hardware E-stop exists and was tested today")
    args = parser.parse_args()

    checks = run(args)
    width = max(len(c.name) for c in checks)
    for c in checks:
        print(f"{'PASS' if c.ok else 'FAIL'}  {c.name.ljust(width)}  {c.detail}")
    failed = [c for c in checks if not c.ok]
    print(f"\nREADINESS: {'PASS' if not failed else 'FAIL'} ({len(checks) - len(failed)}/{len(checks)} passed)")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
