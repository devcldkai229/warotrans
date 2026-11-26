"""ROS side of map building (mapping_session.py): localization on/off, slam_toolbox, the map it builds.

Runs in the mapping worker thread, never in the executor: service calls are polled, so the gateway's
SingleThreadedExecutor keeps spinning meanwhile.

  localization   ``<manager>/manage_nodes`` (nav2_msgs/ManageLifecycleNodes) PAUSE / RESUME: map_server and AMCL
                 are deactivated while slam_toolbox owns map -> odom, then reactivated (map_server republishes the
                 operational map). Empty manager = the stack has no lifecycle-managed localization (nothing to pause).
  slam_toolbox   ``ros2 launch warotrans_slam slam.launch.py`` (mapping mode) as a child process group, with a copy of
                 its params where ``map_name`` is ``map_topic`` (default /mapping/map): the operational /map of
                 map_server, its costmaps and AMCL never see the map being built.
  map            subscription to ``map_topic`` (reliable, transient local), turned into mapping_session.Grid.
  pose graph     ``slam_toolbox/serialize_map`` into ``<local_dir>/<session>/posegraph`` (for slam_toolbox
                 localization later); optional: a missing service only logs.
"""

from __future__ import annotations

import os
import signal
import subprocess
import tempfile
import threading
import time
from typing import Any, Optional

from warotrans_fleet.mapping_session import Grid

PAUSE = 1
RESUME = 2


class MappingRos:
    def __init__(
        self,
        node: Any,
        *,
        tf_buffer: Any,
        map_frame: str,
        base_frame: str,
        map_topic: str = "/mapping/map",
        localization_manager: str = "lifecycle_manager_localization",
        slam_params_file: str = "",
        slam_package: str = "warotrans_slam",
        slam_launch: str = "slam.launch.py",
        use_sim_time: bool = False,
        local_dir: str = "~/maps/mapping_sessions",
        service_timeout_sec: float = 15.0,
    ) -> None:
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        self._node = node
        self._tf = tf_buffer
        self._map_frame = map_frame
        self._base_frame = base_frame
        self._map_topic = map_topic
        self._manager = localization_manager.strip()
        self._slam_params_file = slam_params_file.strip()
        self._slam_package = slam_package
        self._slam_launch = slam_launch
        self._use_sim_time = use_sim_time
        self._local_dir = os.path.expanduser(local_dir)
        self._service_timeout = service_timeout_sec
        self._lock = threading.Lock()
        self._grid: Optional[Grid] = None
        self._proc: Optional[subprocess.Popen] = None
        self._log_file: Any = None
        self._manage_client = None

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(OccupancyGrid, map_topic, self._on_map, qos)

    # ------------------------------------------------------------------ executor callback

    def _on_map(self, msg: Any) -> None:
        if self._proc is None:
            return  # a map left over from an earlier session (transient local) is not this session's
        info = msg.info
        from warotrans_fleet.transforms import quaternion_to_yaw

        q = info.origin.orientation
        grid = Grid(
            int(info.width), int(info.height), float(info.resolution),
            float(info.origin.position.x), float(info.origin.position.y), quaternion_to_yaw(q.x, q.y, q.z, q.w),
            msg.data, time.monotonic(),
        )
        with self._lock:
            self._grid = grid

    # ------------------------------------------------------------------ controller ports

    def latest_map(self) -> Optional[Grid]:
        with self._lock:
            return self._grid

    def current_pose(self) -> Optional[tuple[float, float, float]]:
        from rclpy.time import Time
        from tf2_ros import TransformException

        from warotrans_fleet.transforms import quaternion_to_yaw

        try:
            tf = self._tf.lookup_transform(self._map_frame, self._base_frame, Time()).transform
        except TransformException:
            return None
        q = tf.rotation
        return float(tf.translation.x), float(tf.translation.y), quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def pause_localization(self) -> Optional[str]:
        return self._manage(PAUSE)

    def resume_localization(self) -> Optional[str]:
        return self._manage(RESUME)

    def start_slam(self, session_id: str) -> Optional[str]:
        self.stop_slam()
        with self._lock:
            self._grid = None
        try:
            params = self._params_copy()
        except Exception as exc:  # noqa: BLE001
            return f"slam params: {exc}"
        session_dir = os.path.join(self._local_dir, _safe(session_id))
        os.makedirs(session_dir, exist_ok=True)
        cmd = [
            "ros2", "launch", self._slam_package, self._slam_launch,
            f"params_file:={params}",
            f"use_sim_time:={'true' if self._use_sim_time else 'false'}",
        ]
        try:
            self._log_file = open(os.path.join(session_dir, "slam_toolbox.log"), "ab")
            self._proc = subprocess.Popen(
                cmd, stdout=self._log_file, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True
            )
        except OSError as exc:
            self._proc = None
            return f"cannot run {' '.join(cmd[:4])}: {exc}"
        time.sleep(1.0)
        if self._proc.poll() is not None:
            code = self._proc.returncode
            self._proc = None
            return f"slam_toolbox exited at once (code {code}); see {session_dir}/slam_toolbox.log"
        return None

    def stop_slam(self) -> None:
        proc = self._proc
        if proc is None:
            return
        try:
            if proc.poll() is None:
                pgid = os.getpgid(proc.pid)
                for sig, wait in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0), (signal.SIGKILL, 2.0)):
                    os.killpg(pgid, sig)
                    try:
                        proc.wait(timeout=wait)
                        break
                    except subprocess.TimeoutExpired:
                        continue
                # `ros2 launch` exits before slam_toolbox does: a new slam_toolbox started meanwhile published no map
                # (lab 2026-11-25, RBT-001: START 0.7 s after the stop -> MAPPING_NO_MAP). Stop only when the whole
                # group is gone.
                if not wait_group_gone(pgid, 8.0):
                    try:
                        os.killpg(pgid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    wait_group_gone(pgid, 2.0)
        except (ProcessLookupError, PermissionError):
            pass
        finally:
            self._proc = None
            if self._log_file is not None:
                self._log_file.close()
                self._log_file = None

    def save_local(self, session_id: str, pgm: bytes, yaml_text: str) -> Optional[str]:
        session_dir = os.path.join(self._local_dir, _safe(session_id))
        os.makedirs(session_dir, exist_ok=True)
        with open(os.path.join(session_dir, "map.pgm"), "wb") as fh:
            fh.write(pgm)
        with open(os.path.join(session_dir, "map.yaml"), "w", encoding="utf-8") as fh:
            fh.write(yaml_text)
        return session_dir

    def save_posegraph(self, session_id: str) -> Optional[str]:
        try:
            from slam_toolbox.srv import SerializePoseGraph
        except ImportError:
            return "slam_toolbox.srv not available"
        client = self._node.create_client(SerializePoseGraph, "slam_toolbox/serialize_map")
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                return "slam_toolbox/serialize_map unavailable"
            req = SerializePoseGraph.Request()
            req.filename = os.path.join(self._local_dir, _safe(session_id), "posegraph")
            future = client.call_async(req)
            if not _wait(future, 20.0):
                return "serialize_map timed out"
            return None
        finally:
            self._node.destroy_client(client)

    # ------------------------------------------------------------------ helpers

    def _manage(self, command: int) -> Optional[str]:
        if not self._manager:
            return None
        from nav2_msgs.srv import ManageLifecycleNodes

        if self._manage_client is None:
            self._manage_client = self._node.create_client(ManageLifecycleNodes, f"{self._manager}/manage_nodes")
        client = self._manage_client
        if not client.wait_for_service(timeout_sec=3.0):
            return f"{self._manager}/manage_nodes unavailable"
        req = ManageLifecycleNodes.Request()
        req.command = command
        future = client.call_async(req)
        if not _wait(future, self._service_timeout):
            return f"{self._manager} {'PAUSE' if command == PAUSE else 'RESUME'} timed out"
        result = future.result()
        if result is None or not result.success:
            return f"{self._manager} {'PAUSE' if command == PAUSE else 'RESUME'} failed"
        return None

    def _params_copy(self) -> str:
        """slam_toolbox params with map_name = map_topic and mode = mapping, in a temp file."""
        import yaml

        path = self._slam_params_file
        if not path:
            from ament_index_python.packages import get_package_share_directory

            path = os.path.join(get_package_share_directory(self._slam_package), "config", "slam_toolbox.yaml")
        with open(os.path.expanduser(path), encoding="utf-8") as fh:
            doc = yaml.safe_load(fh) or {}
        params = doc.setdefault("slam_toolbox", {}).setdefault("ros__parameters", {})
        params["mode"] = "mapping"
        params["map_name"] = self._map_topic
        params["use_sim_time"] = self._use_sim_time
        fd, out = tempfile.mkstemp(prefix="warotrans_mapping_slam_", suffix=".yaml")
        with os.fdopen(fd, "w", encoding="utf-8") as fh:
            yaml.safe_dump(doc, fh, sort_keys=False)
        return out


def wait_group_gone(pgid: int, timeout_sec: float, *, kill=None, sleep=time.sleep, clock=time.monotonic) -> bool:
    """True once no process of the group is left (signal 0 finds none), False after timeout_sec."""
    kill = kill or os.killpg  # looked up at call time: os.killpg does not exist on Windows (tests)
    deadline = clock() + timeout_sec
    while True:
        try:
            kill(pgid, 0)
        except ProcessLookupError:
            return True
        except PermissionError:
            return False
        if clock() >= deadline:
            return False
        sleep(0.1)


def _wait(future: Any, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.05)
    return future.done()


def _safe(session_id: str) -> str:
    return "".join(c for c in session_id if c.isalnum() or c in "-_")[:64] or "session"
