"""ROS 2 node of the evidence agent (one per robot, beside the gateway and the camera).

Inputs
  /fleet/active_command          std_msgs/String JSON (gateway): the backend command running now, or null
  /evidence/camera_segments      std_msgs/String JSON (camera): a closed AVI segment
  /evidence/camera_status        std_msgs/String JSON (camera): recording state, fps, last error
  TF map -> base_link            the path actually driven (5 Hz moving, 1 Hz still)
  /odom                          measured velocities
  /plan                          Nav2's global plan (every replan kept)
  /navigate_to_pose/_action/status  goal outcomes as events
Output
  /evidence/recording_control    std_msgs/String JSON to the camera

Uploading never blocks navigation: it runs in its own thread, rate limited, and stops nothing if the network is down
(the spool keeps everything). The robot's credential comes from the environment (WARO_EVIDENCE_TOKEN) or a file, never
from a parameter file in a repository.
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from typing import Optional

from .agent_core import AgentCore, CameraProfile
from .recorder_api import ApiError, RecorderApi
from .sampling import read_clock_quality
from .spool import Spool
from .uploader import Uploader


def _boot_id() -> str:
    try:
        with open("/proc/sys/kernel/random/boot_id", encoding="ascii") as f:
            return f.read().strip()
    except OSError:
        import uuid
        return str(uuid.uuid4())


def main(args=None) -> None:
    import rclpy
    from action_msgs.msg import GoalStatus, GoalStatusArray
    from nav_msgs.msg import Odometry, Path
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener

    class EvidenceAgentNode(Node):
        def __init__(self) -> None:
            super().__init__("evidence_agent")
            d = self.declare_parameter
            d("recorder_url", "http://192.168.137.1:5095")
            d("robot_code", os.environ.get("WARO_ROBOT_CODE", ""))
            d("token_file", "")
            d("spool_dir", os.path.expanduser("~/warotrans_evidence"))
            d("quota_gb", 20.0)
            d("upload_rate_limit_mbytes_per_s", 2.0)  # MB/s (a 30 s 720p segment is ~36 MB)
            d("delete_uploaded", True)
            d("map_frame", "map")
            d("base_frame", "base_link")
            d("map_version_code", "")
            d("camera_id", "front")
            d("camera_model", "Raspberry Pi Camera Module 3")
            d("nominal_fov_degrees", 0.0)
            d("width", 1280)
            d("height", 720)
            d("framerate", 15)
            g = lambda n: self.get_parameter(n).value  # noqa: E731

            token = os.environ.get("WARO_EVIDENCE_TOKEN", "")
            if not token and g("token_file"):
                with open(os.path.expanduser(str(g("token_file"))), encoding="utf-8") as f:
                    token = f.read().strip()
            code = str(g("robot_code")) or os.environ.get("ROBOT_CODE", "")
            if not token or not code:
                self.get_logger().error("evidence: no robot code or token (WARO_EVIDENCE_TOKEN): recording stays local only")
            self.api = RecorderApi(str(g("recorder_url")), code, token)
            spool_dir = os.path.expanduser(str(g("spool_dir")))
            self.spool = Spool(os.path.join(spool_dir, "evidence.db"))
            fov = float(g("nominal_fov_degrees"))
            profile = CameraProfile(str(g("camera_id")), str(g("camera_model")), fov if fov > 0 else None,
                                    int(g("width")), int(g("height")), int(g("framerate")))
            self._control = self.create_publisher(String, "/evidence/recording_control",
                                                  QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE))
            self.core = AgentCore(self.spool, spool_dir, profile, _boot_id(),
                                  lambda c: self._control.publish(String(data=json.dumps(c))),
                                  quota_bytes=int(float(g("quota_gb")) * 1024 ** 3),
                                  map_version=str(g("map_version_code")) or None, logger=self.get_logger())
            self.uploader = Uploader(self.spool, self.api, self.get_logger(),
                                     rate_limit_bps=float(g("upload_rate_limit_mbytes_per_s")) * 1_000_000,
                                     delete_uploaded=bool(g("delete_uploaded")))
            self._lock = threading.Lock()  # the core is fed from the executor and the poll thread
            self._map_frame, self._base_frame = str(g("map_frame")), str(g("base_frame"))
            self._v: Optional[float] = None
            self._w: Optional[float] = None
            self._camera_error = ""
            self._nav_status: dict[bytes, int] = {}

            self.tf = Buffer()
            self._tf_listener = TransformListener(self.tf, self)
            reliable = QoSProfile(depth=50, reliability=QoSReliabilityPolicy.RELIABLE)
            self.create_subscription(String, "/fleet/active_command", self._on_command, 10)
            self.create_subscription(String, "/evidence/camera_segments", self._on_segment, reliable)
            self.create_subscription(String, "/evidence/camera_status", self._on_camera_status, 5)
            self.create_subscription(Odometry, "/odom", self._on_odom, 10)
            self.create_subscription(Path, "/plan", self._on_plan, 5)
            self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status", self._on_nav_status, 10)
            self.create_timer(0.2, self._sample_pose)
            self.create_timer(1.0, self._tick)
            self.create_timer(60.0, self._anchor)
            self._goal_status = GoalStatus
            self._time = Time
            self._stop = threading.Event()
            threading.Thread(target=self._background, name="evidence_upload", daemon=True).start()
            self._anchor()
            self.get_logger().info(f"evidence agent for {code} -> {g('recorder_url')}, spool {spool_dir}")

        # ---------------------------------------------------------------- callbacks
        def _on_command(self, msg) -> None:
            try:
                command = json.loads(msg.data)
            except ValueError:
                return
            with self._lock:
                self.core.on_active_command(command if isinstance(command, dict) else None)

        def _on_segment(self, msg) -> None:
            try:
                info = json.loads(msg.data)
            except ValueError:
                return
            with self._lock:
                self.core.on_camera_segment(info)

        def _on_camera_status(self, msg) -> None:
            try:
                status = json.loads(msg.data)
            except ValueError:
                return
            error = status.get("lastError") or ""
            if error and error != self._camera_error:
                with self._lock:
                    self.core.add_event("camera.error", "CAMERA", "WARNING", status, message=f"Camera: {error[:200]}")
            self._camera_error = error

        def _on_odom(self, msg) -> None:
            self._v = msg.twist.twist.linear.x
            self._w = msg.twist.twist.angular.z

        def _on_plan(self, msg) -> None:
            points = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            with self._lock:
                self.core.on_plan(points)

        def _on_nav_status(self, msg) -> None:
            names = {4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
            for st in msg.status_list:
                gid = bytes(st.goal_info.goal_id.uuid)
                if self._nav_status.get(gid) == st.status:
                    continue
                self._nav_status[gid] = st.status
                if st.status in names:
                    severity = {4: "INFO", 5: "WARNING", 6: "ERROR"}[st.status]
                    with self._lock:
                        self.core.add_event(f"nav2.goal.{names[st.status].lower()}", "NAVIGATION", severity,
                                            {"goalId": gid.hex()}, message=f"Nav2 goal {names[st.status]}")
                        self.core.on_pose(*self._pose(), force=True)
            if len(self._nav_status) > 200:
                self._nav_status.clear()

        def _pose(self) -> tuple:
            try:
                t = self.tf.lookup_transform(self._map_frame, self._base_frame, self._time())
            except Exception:  # noqa: BLE001 - no transform: an invalid sample (a gap)
                return (math.nan, math.nan, math.nan, self._v, self._w, 1e9, False)
            q = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            stamp = Time.from_msg(t.header.stamp)
            age = (self.get_clock().now() - stamp).nanoseconds / 1e9 if stamp.nanoseconds else 0.0
            return (t.transform.translation.x, t.transform.translation.y, yaw, self._v, self._w, max(0.0, age), True)

        def _sample_pose(self) -> None:
            with self._lock:
                self.core.on_pose(*self._pose())

        def _tick(self) -> None:
            with self._lock:
                self.core.tick()

        def _anchor(self) -> None:
            quality = read_clock_quality()
            with self._lock:
                self.core.add_clock_anchor(time.monotonic_ns(), quality)

        # ---------------------------------------------------------------- background
        def _background(self) -> None:
            last_poll = 0.0
            while not self._stop.is_set():
                now = time.monotonic()
                if now - last_poll >= 2.0:
                    last_poll = now
                    try:
                        run = self.api.active_run()
                        with self._lock:
                            self.core.on_active_run(run)
                    except ApiError:
                        pass  # offline: recording follows the gateway's commands, the run binds later
                try:
                    self.uploader.pump()
                except Exception as exc:  # noqa: BLE001 - never let the upload thread die
                    self.get_logger().warning(f"evidence: upload pass failed: {exc}")
                self._stop.wait(1.0)

        def destroy_node(self) -> bool:
            self._stop.set()
            with self._lock:
                self.core.shutdown()
            return super().destroy_node()

    rclpy.init(args=args)
    node = None
    try:
        node = EvidenceAgentNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
