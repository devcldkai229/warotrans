"""Four MKE-S01 ultrasonic sensors on Raspberry Pi GPIO (BCM numbering).

One node, sequential trigger FRONT -> RIGHT -> REAR -> LEFT to avoid
acoustic cross-talk. Monitoring only — no motor stop / Nav2 integration.

Pi 5 / Ubuntu 24.04: uses python3-lgpio on /dev/gpiochip4 (pinctrl-rp1).

ECHO timing uses busy-poll gpio_read + monotonic timestamps. Short pulses
(near targets ~0.05 m ≈ 0.3 ms) are easily missed by lgpio edge callbacks.
"""

from __future__ import annotations

import math
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

try:
    import lgpio
except ImportError as exc:  # pragma: no cover - runtime on Pi only
    raise SystemExit(
        "python3-lgpio is required on Raspberry Pi 5. "
        "Install with: sudo apt install -y python3-lgpio"
    ) from exc


@dataclass(frozen=True)
class SensorSpec:
    name: str
    topic: str
    frame_id: str
    trig_param: str
    echo_param: str


# Polling order: FRONT -> RIGHT -> REAR -> LEFT
SENSOR_SPECS: tuple[SensorSpec, ...] = (
    SensorSpec(
        "FRONT",
        "/ultrasonic/front",
        "ultrasonic_front",
        "front_trig_gpio",
        "front_echo_gpio",
    ),
    SensorSpec(
        "RIGHT",
        "/ultrasonic/right",
        "ultrasonic_right",
        "right_trig_gpio",
        "right_echo_gpio",
    ),
    SensorSpec(
        "REAR",
        "/ultrasonic/rear",
        "ultrasonic_rear",
        "rear_trig_gpio",
        "rear_echo_gpio",
    ),
    SensorSpec(
        "LEFT",
        "/ultrasonic/left",
        "ultrasonic_left",
        "left_trig_gpio",
        "left_echo_gpio",
    ),
)


class UltrasonicNode(Node):
    """Read four ultrasonics sequentially and publish sensor_msgs/Range."""

    def __init__(self) -> None:
        super().__init__("ultrasonic_node")

        self.declare_parameter("front_trig_gpio", 17)
        self.declare_parameter("front_echo_gpio", 27)
        self.declare_parameter("rear_trig_gpio", 22)
        self.declare_parameter("rear_echo_gpio", 23)
        self.declare_parameter("left_trig_gpio", 24)
        self.declare_parameter("left_echo_gpio", 25)
        self.declare_parameter("right_trig_gpio", 5)
        self.declare_parameter("right_echo_gpio", 6)

        self.declare_parameter("min_range", 0.03)
        self.declare_parameter("max_range", 2.0)
        self.declare_parameter("speed_of_sound", 343.0)
        self.declare_parameter("trigger_pulse_us", 10)
        self.declare_parameter("inter_sensor_delay_ms", 60)
        self.declare_parameter("echo_timeout_ms", 35)
        self.declare_parameter("echo_recover_ms", 80)
        self.declare_parameter("median_window", 3)
        self.declare_parameter("field_of_view_deg", 15.0)
        # Pi 5 RP1 user GPIO is gpiochip4 (dialout). Not gpiochip0.
        self.declare_parameter("gpiochip", 4)
        self.declare_parameter("debug_log_period_s", 1.0)

        self._min_range = float(self.get_parameter("min_range").value)
        self._max_range = float(self.get_parameter("max_range").value)
        self._speed_of_sound = float(self.get_parameter("speed_of_sound").value)
        self._trigger_pulse_us = int(self.get_parameter("trigger_pulse_us").value)
        self._inter_sensor_delay_s = (
            float(self.get_parameter("inter_sensor_delay_ms").value) / 1000.0
        )
        self._echo_timeout_s = (
            float(self.get_parameter("echo_timeout_ms").value) / 1000.0
        )
        self._echo_recover_s = (
            float(self.get_parameter("echo_recover_ms").value) / 1000.0
        )
        self._median_window = max(1, int(self.get_parameter("median_window").value))
        self._fov = math.radians(
            float(self.get_parameter("field_of_view_deg").value)
        )
        self._gpiochip = int(self.get_parameter("gpiochip").value)
        self._debug_period_s = float(
            self.get_parameter("debug_log_period_s").value
        )
        self._warn_period = 2.0

        self._sensors: list[dict] = []
        for spec in SENSOR_SPECS:
            trig = int(self.get_parameter(spec.trig_param).value)
            echo = int(self.get_parameter(spec.echo_param).value)
            pub = self.create_publisher(Range, spec.topic, 10)
            self._sensors.append(
                {
                    "spec": spec,
                    "trig": trig,
                    "echo": echo,
                    "pub": pub,
                    "last_range": float("inf"),
                    "history": deque(maxlen=self._median_window),
                }
            )

        self._chip: Optional[int] = None
        self._running = True
        self._worker: Optional[threading.Thread] = None
        self._last_debug_log = 0.0

        self._open_gpio()
        self.get_logger().info(
            "ultrasonic_node ready on gpiochip"
            f"{self._gpiochip}: FRONT->RIGHT->REAR->LEFT "
            f"(delay={self._inter_sensor_delay_s * 1000:.0f} ms, "
            f"echo_timeout={self._echo_timeout_s * 1000:.0f} ms, "
            f"median={self._median_window}, measure=busy-poll)"
        )

        self._worker = threading.Thread(
            target=self._measure_loop, name="ultrasonic_measure", daemon=True
        )
        self._worker.start()

    def _open_gpio(self) -> None:
        try:
            self._chip = lgpio.gpiochip_open(self._gpiochip)
        except Exception as exc:
            self.get_logger().fatal(
                f"Failed to open gpiochip{self._gpiochip}: {exc}. "
                "On Pi 5 use gpiochip=4 (pinctrl-rp1). "
                "Ensure user is in group dialout and python3-lgpio is installed."
            )
            raise

        assert self._chip is not None
        try:
            for item in self._sensors:
                lgpio.gpio_claim_output(self._chip, item["trig"], 0)
                lgpio.gpio_claim_input(
                    self._chip, item["echo"], lgpio.SET_PULL_DOWN
                )
        except lgpio.error as exc:
            self._cleanup_gpio()
            raise RuntimeError(
                f"GPIO busy ({exc}). Another ultrasonic_node is likely still "
                "running. Stop it with: pkill -f ultrasonic_node"
            ) from exc

    def _wait_echo_level(
        self, echo: int, level: int, timeout_s: float
    ) -> Optional[int]:
        """Busy-wait until ECHO equals ``level``; return monotonic_ns."""
        assert self._chip is not None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if lgpio.gpio_read(self._chip, echo) == level:
                return time.monotonic_ns()
        return None

    def _ensure_echo_low(self, echo: int, name: str) -> bool:
        """Return True if ECHO is (or becomes) LOW before a new trigger."""
        assert self._chip is not None
        if lgpio.gpio_read(self._chip, echo) == 0:
            return True
        recovered = self._wait_echo_level(echo, 0, self._echo_recover_s)
        if recovered is None:
            self.get_logger().warning(
                f"{name}: ECHO stuck HIGH (recover failed)",
                throttle_duration_sec=self._warn_period,
            )
            return False
        return True

    def _median_range(self, history: Deque[float], sample: float) -> float:
        history.append(sample)
        valid = [v for v in history if not math.isinf(v)]
        if not valid:
            return float("inf")
        if len(valid) == 1:
            return valid[0]
        return float(statistics.median(valid))

    def _measure_loop(self) -> None:
        while self._running and rclpy.ok():
            for item in self._sensors:
                if not self._running:
                    break
                raw = self._measure(item)
                filtered = self._median_range(item["history"], raw)
                item["last_range"] = filtered
                self._publish_range(item, filtered)
                time.sleep(self._inter_sensor_delay_s)
            self._maybe_debug_log()

    def _measure(self, item: dict) -> float:
        assert self._chip is not None
        name = item["spec"].name
        trig = item["trig"]
        echo = item["echo"]

        if not self._ensure_echo_low(echo, name):
            return float("inf")

        lgpio.gpio_write(self._chip, trig, 0)
        time.sleep(0.000002)
        lgpio.gpio_write(self._chip, trig, 1)
        time.sleep(self._trigger_pulse_us * 1e-6)
        lgpio.gpio_write(self._chip, trig, 0)

        rise_ns = self._wait_echo_level(echo, 1, self._echo_timeout_s)
        if rise_ns is None:
            self.get_logger().warning(
                f"{name}: ECHO rising-edge timeout",
                throttle_duration_sec=self._warn_period,
            )
            return float("inf")

        fall_ns = self._wait_echo_level(echo, 0, self._echo_timeout_s)
        if fall_ns is None:
            self.get_logger().warning(
                f"{name}: ECHO falling-edge timeout",
                throttle_duration_sec=self._warn_period,
            )
            # Drain stuck HIGH so the next sensor/cycle starts clean.
            self._ensure_echo_low(echo, name)
            return float("inf")

        if fall_ns <= rise_ns:
            return float("inf")

        duration_s = (fall_ns - rise_ns) * 1e-9
        distance_m = duration_s * self._speed_of_sound / 2.0

        if distance_m < self._min_range or distance_m > self._max_range:
            self.get_logger().warning(
                f"{name}: out of range ({distance_m:.3f} m)",
                throttle_duration_sec=self._warn_period,
            )
            return float("inf")
        return distance_m

    def _publish_range(self, item: dict, distance_m: float) -> None:
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = item["spec"].frame_id
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self._fov
        msg.min_range = self._min_range
        msg.max_range = self._max_range
        msg.range = distance_m
        item["pub"].publish(msg)

    def _maybe_debug_log(self) -> None:
        now = time.monotonic()
        if now - self._last_debug_log < self._debug_period_s:
            return
        self._last_debug_log = now

        def fmt(value: float) -> str:
            if math.isinf(value):
                return "inf"
            return f"{value:.2f} m"

        by_name = {item["spec"].name: item["last_range"] for item in self._sensors}
        self.get_logger().info(
            f"FRONT: {fmt(by_name['FRONT'])}  "
            f"REAR : {fmt(by_name['REAR'])}  "
            f"LEFT : {fmt(by_name['LEFT'])}  "
            f"RIGHT: {fmt(by_name['RIGHT'])}"
        )

    def destroy_node(self) -> bool:
        self._running = False
        if self._worker is not None and self._worker.is_alive():
            self._worker.join(timeout=2.0)
        self._cleanup_gpio()
        return super().destroy_node()

    def _cleanup_gpio(self) -> None:
        if self._chip is None:
            return
        try:
            for item in self._sensors:
                try:
                    lgpio.gpio_write(self._chip, item["trig"], 0)
                except Exception:
                    pass
                try:
                    lgpio.gpio_free(self._chip, item["trig"])
                except Exception:
                    pass
                try:
                    lgpio.gpio_free(self._chip, item["echo"])
                except Exception:
                    pass
            lgpio.gpiochip_close(self._chip)
        except Exception as exc:
            self.get_logger().warning(f"GPIO cleanup error: {exc}")
        finally:
            self._chip = None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = UltrasonicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
