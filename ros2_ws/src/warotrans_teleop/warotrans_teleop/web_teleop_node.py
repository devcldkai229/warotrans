"""Mobile web teleop node for WaroTrans commissioning."""

from __future__ import annotations

import json
import math
import mimetypes
import threading
import time
from dataclasses import dataclass, field
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Optional
from urllib.parse import urlparse

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from warotrans_hardware.motor_mixer import mix_to_pwm
from warotrans_msgs.msg import MotorCommandState


VALID_DIRECTIONS = {"STOP", "FORWARD", "BACKWARD", "LEFT", "RIGHT"}


@dataclass
class TeleopState:
    direction: str = "STOP"
    speed_percent: float = 20.0
    last_heartbeat_monotonic: float = 0.0
    command_active: bool = False
    odom_linear: float = 0.0
    odom_angular: float = 0.0
    lock: threading.Lock = field(default_factory=threading.Lock)


class WebTeleopNode(Node):
    """Serve a mobile web UI and publish /cmd_vel plus motor command status."""

    def __init__(self) -> None:
        super().__init__("web_teleop_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 8080)
        self.declare_parameter("default_speed_percent", 20.0)
        self.declare_parameter("min_speed_percent", 10.0)
        self.declare_parameter("max_speed_percent", 40.0)
        self.declare_parameter("speed_step_percent", 5.0)
        self.declare_parameter("commissioning_linear_command_max", 0.32)
        self.declare_parameter("commissioning_angular_command_max", 2.9317)
        self.declare_parameter("wheel_separation", 0.2183)
        self.declare_parameter("pwm_raw_max", 255)
        self.declare_parameter("pwm_limit_percent", 40.0)
        self.declare_parameter("deadman_timeout_ms", 250)
        self.declare_parameter("cmd_vel_publish_hz", 20.0)
        self.declare_parameter("motor_state_publish_hz", 10.0)

        self._host = str(self.get_parameter("host").value)
        self._port = int(self.get_parameter("port").value)
        self._min_speed = float(self.get_parameter("min_speed_percent").value)
        self._max_speed = float(self.get_parameter("max_speed_percent").value)
        self._speed_step = float(self.get_parameter("speed_step_percent").value)
        self._linear_max = float(
            self.get_parameter("commissioning_linear_command_max").value
        )
        self._angular_max = float(
            self.get_parameter("commissioning_angular_command_max").value
        )
        self._wheel_separation = float(self.get_parameter("wheel_separation").value)
        self._pwm_raw_max = int(self.get_parameter("pwm_raw_max").value)
        self._pwm_limit_percent = float(
            self.get_parameter("pwm_limit_percent").value
        )
        self._deadman_timeout_s = (
            float(self.get_parameter("deadman_timeout_ms").value) / 1000.0
        )
        self._cmd_vel_hz = float(self.get_parameter("cmd_vel_publish_hz").value)
        self._motor_state_hz = float(
            self.get_parameter("motor_state_publish_hz").value
        )

        default_speed = float(self.get_parameter("default_speed_percent").value)
        self._validate_parameters(default_speed)

        self._state = TeleopState(
            direction="STOP",
            speed_percent=default_speed,
        )

        share_dir = Path(get_package_share_directory("warotrans_teleop"))
        self._web_root = share_dir / "web"

        self._cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self._motor_state_publisher = self.create_publisher(
            MotorCommandState,
            "/motor_command_state",
            10,
        )
        self._odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self._on_odom,
            10,
        )

        cmd_period = 1.0 / self._cmd_vel_hz if self._cmd_vel_hz > 0.0 else 0.05
        motor_period = (
            1.0 / self._motor_state_hz if self._motor_state_hz > 0.0 else 0.1
        )
        self.create_timer(cmd_period, self._publish_cmd_vel)
        self.create_timer(motor_period, self._publish_motor_state)
        self.create_timer(0.02, self._check_deadman)

        handler_cls = self._make_handler()
        self._http_server = ThreadingHTTPServer((self._host, self._port), handler_cls)
        self._http_thread = threading.Thread(
            target=self._http_server.serve_forever,
            name="web_teleop_http",
            daemon=True,
        )
        self._http_thread.start()
        self.get_logger().info(
            f"Web teleop listening on http://{self._host}:{self._port}/"
        )

    def _validate_parameters(self, default_speed: float) -> None:
        if self._port <= 0 or self._port > 65535:
            raise ValueError("port must be between 1 and 65535")
        if self._min_speed <= 0.0 or self._max_speed <= 0.0:
            raise ValueError("speed percent bounds must be positive")
        if self._min_speed > self._max_speed:
            raise ValueError("min_speed_percent must not exceed max_speed_percent")
        if self._max_speed > self._pwm_limit_percent:
            raise ValueError(
                "max_speed_percent must not exceed firmware pwm_limit_percent"
            )
        if not self._min_speed <= default_speed <= self._max_speed:
            raise ValueError("default_speed_percent out of allowed range")
        if self._linear_max <= 0.0 or self._angular_max <= 0.0:
            raise ValueError("commissioning command max values must be positive")
        if self._wheel_separation <= 0.0:
            raise ValueError("wheel_separation must be positive")
        if self._pwm_raw_max <= 0:
            raise ValueError("pwm_raw_max must be positive")

    def _clamp_speed(self, speed_percent: float) -> float:
        return max(self._min_speed, min(self._max_speed, speed_percent))

    def _direction_to_twist(self, direction: str, speed_percent: float) -> Twist:
        """Map UI direction to /cmd_vel (ROS REP-103).

        FORWARD/BACKWARD -> ±linear.x
        LEFT/RIGHT       -> ±angular.z (positive = CCW / left turn)
        """
        twist = Twist()
        level = self._clamp_speed(speed_percent) / 100.0
        if direction == "FORWARD":
            twist.linear.x = level * self._linear_max
        elif direction == "BACKWARD":
            twist.linear.x = -level * self._linear_max
        elif direction == "LEFT":
            twist.angular.z = level * self._angular_max
        elif direction == "RIGHT":
            twist.angular.z = -level * self._angular_max
        return twist

    def _current_twist(self) -> Twist:
        with self._state.lock:
            direction = self._state.direction
            speed_percent = self._state.speed_percent
            active = self._state.command_active
            last_hb = self._state.last_heartbeat_monotonic

        if not active:
            return Twist()

        if time.monotonic() - last_hb > self._deadman_timeout_s:
            return Twist()

        return self._direction_to_twist(direction, speed_percent)

    def _publish_cmd_vel(self) -> None:
        self._cmd_vel_publisher.publish(self._current_twist())

    def _publish_motor_state(self) -> None:
        twist = self._current_twist()
        linear = float(twist.linear.x)
        angular = float(twist.angular.z)

        with self._state.lock:
            direction = self._state.direction
            speed_percent = self._state.speed_percent
            active = self._state.command_active
            last_hb = self._state.last_heartbeat_monotonic
            odom_linear = self._state.odom_linear
            odom_angular = self._state.odom_angular

        age_ms = 0
        if last_hb > 0.0:
            age_ms = int(max(0.0, (time.monotonic() - last_hb) * 1000.0))

        mix = mix_to_pwm(
            linear,
            angular,
            wheel_separation=self._wheel_separation,
            full_scale_linear=self._linear_max,
            pwm_raw_max=self._pwm_raw_max,
            pwm_limit_percent=self._pwm_limit_percent,
        )

        message = MotorCommandState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.direction = direction if active else "STOP"
        message.speed_level_percent = float(speed_percent)
        message.linear_x = linear
        message.angular_z = angular
        message.pwm_raw_max = self._pwm_raw_max
        message.pwm_limit_percent = float(self._pwm_limit_percent)
        message.command_active = active and age_ms <= int(
            self._deadman_timeout_s * 1000.0
        )
        message.last_command_age_ms = age_ms

        if mix is not None:
            message.left_pwm_percent = float(mix.left_pwm_percent)
            message.right_pwm_percent = float(mix.right_pwm_percent)
            message.left_pwm_raw = int(mix.left_pwm_raw)
            message.right_pwm_raw = int(mix.right_pwm_raw)

        self._motor_state_publisher.publish(message)

        # Keep odom values available for HTTP status via state (already stored).
        del odom_linear, odom_angular

    def _check_deadman(self) -> None:
        with self._state.lock:
            if not self._state.command_active:
                return
            if self._state.last_heartbeat_monotonic <= 0.0:
                return
            expired = (
                time.monotonic() - self._state.last_heartbeat_monotonic
                > self._deadman_timeout_s
            )
            if not expired:
                return
            self._state.command_active = False
            self._state.direction = "STOP"

        stop = Twist()
        self._cmd_vel_publisher.publish(stop)

    def _on_odom(self, msg: Odometry) -> None:
        with self._state.lock:
            self._state.odom_linear = float(msg.twist.twist.linear.x)
            self._state.odom_angular = float(msg.twist.twist.angular.z)

    def _handle_command(self, payload: dict) -> dict:
        direction = str(payload.get("direction", "STOP")).upper()
        if direction not in VALID_DIRECTIONS:
            raise ValueError(f"invalid direction: {direction}")

        speed_percent = payload.get("speed_percent")
        now = time.monotonic()

        with self._state.lock:
            if speed_percent is not None:
                self._state.speed_percent = self._clamp_speed(float(speed_percent))
            self._state.direction = direction
            self._state.command_active = direction != "STOP"
            if self._state.command_active:
                self._state.last_heartbeat_monotonic = now
            current_speed = self._state.speed_percent

        if direction == "STOP":
            self._cmd_vel_publisher.publish(Twist())
        else:
            self._cmd_vel_publisher.publish(
                self._direction_to_twist(direction, current_speed)
            )

        return self._build_status()

    def _handle_stop(self) -> dict:
        with self._state.lock:
            self._state.direction = "STOP"
            self._state.command_active = False
        self._cmd_vel_publisher.publish(Twist())
        return self._build_status()

    def _adjust_speed(self, delta: float) -> dict:
        with self._state.lock:
            self._state.speed_percent = self._clamp_speed(
                self._state.speed_percent + delta
            )
        return self._build_status()

    def _set_speed(self, speed_percent: float) -> dict:
        with self._state.lock:
            self._state.speed_percent = self._clamp_speed(speed_percent)
        twist = self._current_twist()
        if twist.linear.x != 0.0 or twist.angular.z != 0.0:
            self._cmd_vel_publisher.publish(twist)
        return self._build_status()

    def _build_status(self) -> dict:
        twist = self._current_twist()
        linear = float(twist.linear.x)
        angular = float(twist.angular.z)

        with self._state.lock:
            direction = self._state.direction
            speed_percent = self._state.speed_percent
            active = self._state.command_active
            last_hb = self._state.last_heartbeat_monotonic
            odom_linear = self._state.odom_linear
            odom_angular = self._state.odom_angular

        age_ms = 0
        if last_hb > 0.0:
            age_ms = int(max(0.0, (time.monotonic() - last_hb) * 1000.0))

        mix = mix_to_pwm(
            linear,
            angular,
            wheel_separation=self._wheel_separation,
            full_scale_linear=self._linear_max,
            pwm_raw_max=self._pwm_raw_max,
            pwm_limit_percent=self._pwm_limit_percent,
        )

        status = {
            "ros_connected": True,
            "direction": direction if active else "STOP",
            "speed_level_percent": speed_percent,
            "linear_cmd": linear,
            "angular_cmd": angular,
            "odom_linear": odom_linear,
            "odom_angular": odom_angular,
            "pwm_raw_max": self._pwm_raw_max,
            "pwm_limit_percent": self._pwm_limit_percent,
            "command_active": active and age_ms <= int(
                self._deadman_timeout_s * 1000.0
            ),
            "last_command_age_ms": age_ms,
            "left_pwm_percent": 0.0,
            "right_pwm_percent": 0.0,
            "left_pwm_raw": 0,
            "right_pwm_raw": 0,
        }
        if mix is not None:
            status["left_pwm_percent"] = mix.left_pwm_percent
            status["right_pwm_percent"] = mix.right_pwm_percent
            status["left_pwm_raw"] = mix.left_pwm_raw
            status["right_pwm_raw"] = mix.right_pwm_raw
        return status

    def _make_handler(self):
        node = self

        class TeleopRequestHandler(BaseHTTPRequestHandler):
            def log_message(self, format: str, *args) -> None:  # noqa: A003
                del format, args

            def _send_json(self, payload: dict, status: int = 200) -> None:
                body = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def _read_json(self) -> dict:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0:
                    return {}
                raw = self.rfile.read(length)
                return json.loads(raw.decode("utf-8"))

            def _serve_static(self, rel_path: str) -> None:
                safe_path = Path(rel_path).name
                file_path = node._web_root / safe_path
                if not file_path.is_file():
                    self.send_error(404)
                    return
                content = file_path.read_bytes()
                mime, _ = mimetypes.guess_type(str(file_path))
                self.send_response(200)
                self.send_header(
                    "Content-Type", mime or "application/octet-stream"
                )
                self.send_header("Content-Length", str(len(content)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(content)

            def do_GET(self) -> None:  # noqa: N802
                path = urlparse(self.path).path
                if path == "/api/status":
                    self._send_json(node._build_status())
                    return
                if path in ("/", "/index.html"):
                    self._serve_static("index.html")
                    return
                if path.startswith("/"):
                    candidate = path.lstrip("/")
                    if candidate in ("app.js", "style.css"):
                        self._serve_static(candidate)
                        return
                self.send_error(404)

            def do_POST(self) -> None:  # noqa: N802
                path = urlparse(self.path).path
                try:
                    if path == "/api/command":
                        payload = self._read_json()
                        self._send_json(node._handle_command(payload))
                        return
                    if path == "/api/stop":
                        self._send_json(node._handle_stop())
                        return
                    if path == "/api/speed_delta":
                        payload = self._read_json()
                        delta = float(payload.get("delta", 0.0))
                        self._send_json(node._adjust_speed(delta))
                        return
                    if path == "/api/speed_set":
                        payload = self._read_json()
                        speed = float(payload.get("speed_percent", 0.0))
                        self._send_json(node._set_speed(speed))
                        return
                    self.send_error(404)
                except (ValueError, json.JSONDecodeError) as exc:
                    self._send_json({"error": str(exc)}, status=400)

        return TeleopRequestHandler

    def destroy_node(self) -> bool:
        self._cmd_vel_publisher.publish(Twist())
        if hasattr(self, "_http_server"):
            self._http_server.shutdown()
            self._http_server.server_close()
        if hasattr(self, "_http_thread"):
            self._http_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[WebTeleopNode] = None
    try:
        node = WebTeleopNode()
        rclpy.spin(node)
    except (ValueError, KeyboardInterrupt, OSError) as exc:
        if node is None:
            print(f"web_teleop_node configuration error: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
