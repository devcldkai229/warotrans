"""ROS 2 bridge for ESP32 encoder telemetry and velocity commands."""

from __future__ import annotations

import math
import threading
import time
from typing import Optional

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from warotrans_msgs.msg import WheelTicks

from .serial_protocol import format_velocity_command, parse_encoder_line


class Esp32BridgeNode(Node):
    """Read cumulative encoder ticks and forward /cmd_vel to the ESP32."""

    def __init__(self) -> None:
        super().__init__("esp32_bridge_node")

        self.declare_parameter("serial_port", "/dev/warotrans")
        self.declare_parameter("baud_rate", 921600)

        self._serial_port = str(self.get_parameter("serial_port").value)
        self._baud_rate = int(self.get_parameter("baud_rate").value)
        if not self._serial_port:
            raise ValueError("serial_port must not be empty")
        if self._baud_rate <= 0:
            raise ValueError("baud_rate must be greater than zero")

        self._publisher = self.create_publisher(WheelTicks, "/wheel_ticks", 10)
        self._stop_event = threading.Event()
        self._connection_lock = threading.Lock()
        self._connection: Optional[serial.Serial] = None
        self._last_error_log = 0.0
        self._last_malformed_log = 0.0
        self._last_cmd_warn_log = 0.0

        self._cmd_vel_subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self._on_cmd_vel,
            10,
        )

        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name="esp32_serial_reader",
            daemon=True,
        )
        self._reader_thread.start()

    def _log_error_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log >= 5.0:
            self.get_logger().error(message)
            self._last_error_log = now

    def _log_malformed_throttled(self, raw_line: bytes) -> None:
        now = time.monotonic()
        if now - self._last_malformed_log >= 5.0:
            self.get_logger().warning(
                f"Ignoring malformed ESP32 telemetry: {raw_line!r}"
            )
            self._last_malformed_log = now

    def _log_cmd_warn_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_cmd_warn_log >= 5.0:
            self.get_logger().warning(message)
            self._last_cmd_warn_log = now

    def _send_velocity_command(self, linear_mps: float, angular_rps: float) -> None:
        try:
            payload = format_velocity_command(linear_mps, angular_rps).encode("ascii")
        except ValueError:
            self._log_cmd_warn_throttled(
                "Dropped non-finite /cmd_vel before serial write"
            )
            return

        with self._connection_lock:
            connection = self._connection
        if connection is None or not connection.is_open:
            return

        try:
            connection.write(payload)
            connection.flush()
        except (serial.SerialException, OSError) as exc:
            self._log_error_throttled(
                f"Failed to write velocity command on {self._serial_port}: {exc}"
            )

    def _on_cmd_vel(self, msg: Twist) -> None:
        # Both axes are required for skid-steer turns (Nav2 RPP).
        # Class B debug: if angular.z is significant but /odom.angular.z is not,
        # suspect open-loop PWM realization — not a missing bridge field.
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        if not math.isfinite(linear) or not math.isfinite(angular):
            self._log_cmd_warn_throttled("Ignoring non-finite /cmd_vel Twist")
            return
        self._send_velocity_command(linear, angular)

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            connection: Optional[serial.Serial] = None
            try:
                self.get_logger().info(
                    f"Connecting to ESP32 at {self._serial_port} "
                    f"(baud {self._baud_rate})"
                )
                connection = serial.Serial(
                    port=self._serial_port,
                    baudrate=self._baud_rate,
                    timeout=0.2,
                    dsrdtr=False,
                    rtscts=False,
                )
                connection.dtr = False
                connection.rts = False
                with self._connection_lock:
                    self._connection = connection
                self.get_logger().info("ESP32 serial connection established")

                while not self._stop_event.is_set():
                    raw_line = connection.readline()
                    if not raw_line:
                        continue

                    parsed = parse_encoder_line(raw_line)
                    if parsed is None:
                        self._log_malformed_throttled(raw_line)
                        continue

                    left_ticks, right_ticks, mcu_millis = parsed
                    message = WheelTicks()
                    message.left_ticks = left_ticks
                    message.right_ticks = right_ticks
                    message.mcu_millis = mcu_millis
                    self._publisher.publish(message)
            except (serial.SerialException, OSError) as exc:
                self._log_error_throttled(
                    f"ESP32 serial error on {self._serial_port}: {exc}"
                )
            finally:
                with self._connection_lock:
                    if self._connection is connection:
                        self._connection = None
                if connection is not None:
                    try:
                        connection.close()
                    except OSError:
                        pass

            if not self._stop_event.is_set():
                self.get_logger().warning(
                    "ESP32 serial disconnected; retrying in 1 second"
                )
                self._stop_event.wait(1.0)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._send_velocity_command(0.0, 0.0)
        with self._connection_lock:
            connection = self._connection
        if connection is not None:
            try:
                connection.close()
            except OSError:
                pass
        self._reader_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Esp32BridgeNode] = None
    try:
        node = Esp32BridgeNode()
        rclpy.spin(node)
    except (ValueError, KeyboardInterrupt) as exc:
        if node is None:
            print(f"esp32_bridge_node configuration error: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
