#!/usr/bin/env python3
"""P0 ground-truth motion trace: Gazebo model pose vs /odom vs map TF vs cmd_vel pipeline.

Does NOT treat map→base alone as physical motion. Prefer /sim01/odom (GZ OdometryPublisher)
and optional gz dynamic_pose snapshots written by a sidecar.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def yaw_from_q(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def try_gz_model_pose(world: str, model: str) -> tuple[float, float, float] | None:
    """Best-effort parse of gz model pose via `gz model` / topic snapshot."""
    # Prefer gz model -m <name> -p if available
    for cmd in (
        ["gz", "model", "-m", model, "-p", "-w", world],
        ["gz", "model", "-m", model, "-p"],
    ):
        try:
            out = subprocess.check_output(cmd, stderr=subprocess.DEVNULL, timeout=2.0, text=True)
        except Exception:  # noqa: BLE001
            continue
        # Expect lines like: "Pose: [x y z]" and orientation elsewhere — formats vary.
        # Fallback: parse first three floats after "Pose".
        for line in out.splitlines():
            if "Pose" in line or "pose" in line:
                nums = []
                for tok in line.replace("[", " ").replace("]", " ").replace(",", " ").split():
                    try:
                        nums.append(float(tok))
                    except ValueError:
                        continue
                if len(nums) >= 2:
                    yaw = nums[5] if len(nums) >= 6 else 0.0
                    return nums[0], nums[1], yaw
    return None


class PhysicalMotionTrace(Node):
    def __init__(self, ns: str, out_csv: Path, world: str, model: str, hz: float) -> None:
        super().__init__(
            "gz_physical_motion_trace",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            cli_args=[
                "--ros-args",
                "-r",
                f"/tf:=/{ns}/tf",
                "-r",
                f"/tf_static:=/{ns}/tf_static",
            ],
        )
        self.ns = ns
        self.world = world
        self.model = model
        self.out_csv = out_csv
        self.lock = threading.Lock()
        self.odom = None
        self.map_pose = None
        self.auth = None
        self.auth_t = None
        self.cmd = {
            "nav": (0.0, 0.0),
            "smoothed": (0.0, 0.0),
            "safe": (0.0, 0.0),
            "gated": (0.0, 0.0),
        }
        self.tf = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self._tfl = TransformListener(self.tf, self, spin_thread=True)
        prefix = f"/{ns}"
        self.create_subscription(Odometry, f"{prefix}/odom", self._on_odom, 10)
        self.create_subscription(Bool, f"{prefix}/traffic/authority_ok", self._on_auth, 10)
        for key, topic in (
            ("nav", f"{prefix}/cmd_vel_nav"),
            ("smoothed", f"{prefix}/cmd_vel_smoothed"),
            ("safe", f"{prefix}/cmd_vel_safe"),
            ("gated", f"{prefix}/cmd_vel"),
        ):
            self.create_subscription(Twist, topic, self._mk_cmd(key), 10)
        self.wall0 = time.time()
        self.fields = [
            "wall_t",
            "sim_t",
            "gz_x",
            "gz_y",
            "gz_yaw",
            "odom_x",
            "odom_y",
            "odom_yaw",
            "map_x",
            "map_y",
            "map_yaw",
            "authority_ok",
            "authority_ok_age",
            "cmd_nav_vx",
            "cmd_nav_wz",
            "cmd_smoothed_vx",
            "cmd_smoothed_wz",
            "cmd_safe_vx",
            "cmd_safe_wz",
            "cmd_gated_vx",
            "cmd_gated_wz",
            "dz_gz_odom_m",
            "dz_gz_map_m",
        ]
        self.out_csv.parent.mkdir(parents=True, exist_ok=True)
        with self.out_csv.open("w", newline="", encoding="utf-8") as f:
            csv.DictWriter(f, fieldnames=self.fields).writeheader()
        self.create_timer(1.0 / max(1.0, hz), self._tick)
        self.get_logger().info(
            f"GT trace → {out_csv} ns={ns} world={world} model={model} hz={hz}"
        )

    def _mk_cmd(self, key: str):
        def _cb(msg: Twist) -> None:
            with self.lock:
                self.cmd[key] = (float(msg.linear.x), float(msg.angular.z))

        return _cb

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose
        with self.lock:
            self.odom = (float(p.position.x), float(p.position.y), yaw_from_q(p.orientation))

    def _on_auth(self, msg: Bool) -> None:
        with self.lock:
            self.auth = bool(msg.data)
            self.auth_t = time.monotonic()

    def _lookup_map(self) -> tuple[float, float, float] | None:
        try:
            t = self.tf.lookup_transform("map", f"{self.ns}/base_footprint", rclpy.time.Time())
            tr = t.transform.translation
            return float(tr.x), float(tr.y), yaw_from_q(t.transform.rotation)
        except Exception:  # noqa: BLE001
            return None

    def _tick(self) -> None:
        wall = time.time() - self.wall0
        sim_t = self.get_clock().now().nanoseconds / 1e9
        gz = try_gz_model_pose(self.world, self.model)
        map_pose = self._lookup_map()
        with self.lock:
            odom = self.odom
            auth = self.auth
            auth_age = None if self.auth_t is None else time.monotonic() - self.auth_t
            cmd = dict(self.cmd)
        def d2(a, b):
            if a is None or b is None:
                return ""
            return f"{math.hypot(a[0] - b[0], a[1] - b[1]):.4f}"

        row = {
            "wall_t": f"{wall:.3f}",
            "sim_t": f"{sim_t:.3f}",
            "gz_x": "" if gz is None else f"{gz[0]:.4f}",
            "gz_y": "" if gz is None else f"{gz[1]:.4f}",
            "gz_yaw": "" if gz is None else f"{gz[2]:.4f}",
            "odom_x": "" if odom is None else f"{odom[0]:.4f}",
            "odom_y": "" if odom is None else f"{odom[1]:.4f}",
            "odom_yaw": "" if odom is None else f"{odom[2]:.4f}",
            "map_x": "" if map_pose is None else f"{map_pose[0]:.4f}",
            "map_y": "" if map_pose is None else f"{map_pose[1]:.4f}",
            "map_yaw": "" if map_pose is None else f"{map_pose[2]:.4f}",
            "authority_ok": auth,
            "authority_ok_age": "" if auth_age is None else f"{auth_age:.3f}",
            "cmd_nav_vx": f"{cmd['nav'][0]:.4f}",
            "cmd_nav_wz": f"{cmd['nav'][1]:.4f}",
            "cmd_smoothed_vx": f"{cmd['smoothed'][0]:.4f}",
            "cmd_smoothed_wz": f"{cmd['smoothed'][1]:.4f}",
            "cmd_safe_vx": f"{cmd['safe'][0]:.4f}",
            "cmd_safe_wz": f"{cmd['safe'][1]:.4f}",
            "cmd_gated_vx": f"{cmd['gated'][0]:.4f}",
            "cmd_gated_wz": f"{cmd['gated'][1]:.4f}",
            "dz_gz_odom_m": d2(gz, odom),
            "dz_gz_map_m": d2(gz, map_pose),
        }
        with self.out_csv.open("a", newline="", encoding="utf-8") as f:
            csv.DictWriter(f, fieldnames=self.fields).writerow(row)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--ns", default="sim01")
    ap.add_argument("--world", default="map_one_v001")
    ap.add_argument("--model", default="warotrans_sim_01")
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--duration", type=float, default=0.0)
    args = ap.parse_args()
    rclpy.init()
    node = PhysicalMotionTrace(args.ns, Path(args.out), args.world, args.model, args.hz)
    t0 = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if args.duration > 0 and time.time() - t0 >= args.duration:
                break
    finally:
        meta = {
            "ns": args.ns,
            "world": args.world,
            "model": args.model,
            "out": args.out,
            "note": "gz_* from `gz model -p` when available; odom_* is GZ OdometryPublisher via bridge",
        }
        Path(args.out).with_suffix(".meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
