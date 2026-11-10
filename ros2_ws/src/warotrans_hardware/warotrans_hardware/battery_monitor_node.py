"""Battery SOC node: /battery/adc_sample (+ /cmd_vel, /odom as load proxy) -> /battery_state, /battery/estimate."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from warotrans_msgs.msg import BatteryAdcSample, BatteryEstimate

from .battery_soc import (
    BatteryConfig,
    VoltageSocEstimator,
    battery_state_fields,
    load_state,
    save_state,
)

# Scalar BatteryConfig fields exposed 1:1 as ROS parameters (defaults = BatteryConfig defaults).
_SCALAR_FIELDS = [
    name for name, f in BatteryConfig.__dataclass_fields__.items()
    if name not in ("ocv_table", "calibration_points")
]


class BatteryMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("battery_monitor_node")
        defaults = BatteryConfig()
        for name in _SCALAR_FIELDS:
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter("ocv_cell_voltages", [v for v, _ in defaults.ocv_table])
        self.declare_parameter("ocv_soc_percent", [s for _, s in defaults.ocv_table])
        # Multi-point calibration (pin volts -> pack volts). Empty = divider_gain / offset_v.
        self.declare_parameter("calibration_pin_volts", [0.0])
        self.declare_parameter("calibration_pack_volts", [0.0])
        self.declare_parameter("publish_rate_hz", 2.0)
        self.declare_parameter("state_file", "~/.local/state/warotrans/battery_soc.json")
        self.declare_parameter("persist_interval_s", 30.0)
        self.declare_parameter("persist_delta_pct", 1.0)
        self.declare_parameter("location", "base")
        self.declare_parameter("serial_number", "")

        self._config = self._load_config()
        self._estimator = VoltageSocEstimator(self._config)
        self._state_file = Path(str(self.get_parameter("state_file").value)).expanduser()
        self._persist_interval = float(self.get_parameter("persist_interval_s").value)
        self._persist_delta = float(self.get_parameter("persist_delta_pct").value)
        self._last_persist_t: Optional[float] = None
        self._last_persist_soc: Optional[float] = None
        saved = load_state(self._state_file)
        if saved is not None and self._estimator.restore(saved, time.time()):
            self.get_logger().info(f"Offered persisted SOC {saved.get('soc_pct')}% from {self._state_file}")

        self._location = str(self.get_parameter("location").value)
        self._serial_number = str(self.get_parameter("serial_number").value)
        self._state_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self._estimate_pub = self.create_publisher(BatteryEstimate, "/battery/estimate", 10)
        self.create_subscription(BatteryAdcSample, "/battery/adc_sample", self._on_sample, 20)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        rate = float(self.get_parameter("publish_rate_hz").value)
        if rate <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")
        self.create_timer(1.0 / rate, self._publish)

    def _load_config(self) -> BatteryConfig:
        values = {name: self.get_parameter(name).value for name in _SCALAR_FIELDS}
        values["cells_in_series"] = int(values["cells_in_series"])
        values["median_window"] = int(values["median_window"])
        volts = list(self.get_parameter("ocv_cell_voltages").value)
        socs = list(self.get_parameter("ocv_soc_percent").value)
        if len(volts) != len(socs):
            raise ValueError("ocv_cell_voltages and ocv_soc_percent must have the same length")
        values["ocv_table"] = list(zip(volts, socs))
        pins = list(self.get_parameter("calibration_pin_volts").value)
        packs = list(self.get_parameter("calibration_pack_volts").value)
        if len(pins) != len(packs):
            raise ValueError("calibration_pin_volts and calibration_pack_volts must have the same length")
        # [0.0] / [0.0] is the "no table" placeholder (ROS cannot declare an empty double array).
        points = [(p, v) for p, v in zip(pins, packs) if not (p == 0.0 and v == 0.0)]
        values["calibration_points"] = points
        return BatteryConfig.from_dict(values)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_sample(self, msg: BatteryAdcSample) -> None:
        self._estimator.add_sample(self._now(), float(msg.pin_millivolts))

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._estimator.update_motion(self._now(), float(msg.linear.x), float(msg.angular.z))

    def _on_odom(self, msg: Odometry) -> None:
        twist = msg.twist.twist
        self._estimator.update_motion(self._now(), float(twist.linear.x), float(twist.angular.z))

    def _publish(self) -> None:
        t = self._now()
        result = self._estimator.estimate(t)
        stamp = self.get_clock().now().to_msg()

        state = BatteryState()
        state.header.stamp = stamp
        for key, value in battery_state_fields(result, self._config).items():
            setattr(state, key, value)
        state.location = self._location
        state.serial_number = self._serial_number
        self._state_pub.publish(state)

        estimate = BatteryEstimate()
        estimate.header.stamp = stamp
        estimate.mode = result.mode
        estimate.confidence = result.confidence
        estimate.charging_state = result.charging_state
        nan = float("nan")
        estimate.percentage = result.percentage if result.percentage is not None else nan
        estimate.voltage = result.voltage if result.voltage is not None else nan
        estimate.ocv_estimate = result.ocv_estimate if result.ocv_estimate is not None else nan
        estimate.rested = result.rested
        estimate.fault = result.fault
        estimate.sample_age_s = result.sample_age_s if result.sample_age_s is not None else nan
        self._estimate_pub.publish(estimate)
        self._persist(t, result.percentage)

    def _persist(self, t: float, fraction: Optional[float]) -> None:
        if fraction is None:
            return
        soc = fraction * 100.0
        due = self._last_persist_t is None or t - self._last_persist_t >= self._persist_interval
        moved = self._last_persist_soc is None or abs(soc - self._last_persist_soc) >= self._persist_delta
        if not (due or moved):
            return
        snapshot = self._estimator.snapshot(time.time())
        if snapshot is None:
            return
        try:
            save_state(self._state_file, snapshot)
        except OSError as exc:
            self.get_logger().warning(f"Could not persist battery SOC to {self._state_file}: {exc}")
            return
        self._last_persist_t, self._last_persist_soc = t, soc


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[BatteryMonitorNode] = None
    try:
        node = BatteryMonitorNode()
        rclpy.spin(node)
    except (ValueError, KeyboardInterrupt) as exc:
        if node is None:
            print(f"battery_monitor_node configuration error: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
