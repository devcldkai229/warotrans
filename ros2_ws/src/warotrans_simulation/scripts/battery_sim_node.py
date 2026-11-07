#!/usr/bin/env python3
"""Emulate the battery voltage divider in Gazebo (simulation ONLY, launch arg battery_sim:=true).

Publishes /battery/adc_sample exactly like the real esp32_bridge does when the 0-25 V module is installed, so
the real warotrans_hardware battery_monitor_node (voltage-only SOC) runs unchanged in simulation:

    /odom (speed)  ->  pack model (OCV(SOC) - I*R, SOC drains with I)  ->  pin mV = pack / divider + noise

The model constants are illustrative, not measured: this exercises the pipeline, it is not a battery model
of the real robot.
"""

from __future__ import annotations

import random
from bisect import bisect_left

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from warotrans_hardware.battery_soc import DEFAULT_OCV_TABLE
from warotrans_msgs.msg import BatteryAdcSample


class BatterySimModel:
    def __init__(
        self,
        *,
        initial_soc_pct: float,
        capacity_ah: float,
        cells_in_series: int,
        idle_current_a: float,
        current_a_per_mps: float,
        current_a_per_radps: float,
        internal_resistance_ohm: float,
        divider_gain: float,
        noise_mv: float,
        seed: int,
    ) -> None:
        self.soc_pct = min(100.0, max(0.0, initial_soc_pct))
        self._capacity_ah = capacity_ah
        self._cells = cells_in_series
        self._idle = idle_current_a
        self._k_lin = current_a_per_mps
        self._k_ang = current_a_per_radps
        self._r = internal_resistance_ohm
        self._gain = divider_gain
        self._noise = noise_mv
        self._rng = random.Random(seed)
        self._socs = [s for _, s in DEFAULT_OCV_TABLE]
        self._volts = [v for v, _ in DEFAULT_OCV_TABLE]

    def cell_ocv(self, soc_pct: float) -> float:
        i = min(max(bisect_left(self._socs, soc_pct), 1), len(self._socs) - 1)
        s0, s1 = self._socs[i - 1], self._socs[i]
        v0, v1 = self._volts[i - 1], self._volts[i]
        return v0 if s1 == s0 else v0 + (v1 - v0) * (soc_pct - s0) / (s1 - s0)

    def step(self, dt_s: float, linear_mps: float, angular_radps: float) -> float:
        """Advance dt and return the pin millivolts the ESP32 would read."""
        current = self._idle + self._k_lin * abs(linear_mps) + self._k_ang * abs(angular_radps)
        self.soc_pct = max(0.0, self.soc_pct - current * dt_s / 3600.0 / self._capacity_ah * 100.0)
        pack_v = self.cell_ocv(self.soc_pct) * self._cells - current * self._r
        pin_mv = pack_v / self._gain * 1000.0 + self._rng.gauss(0.0, self._noise)
        return min(3300.0, max(0.0, pin_mv))


class BatterySimNode(Node):
    def __init__(self) -> None:
        super().__init__("battery_sim_node")
        self.declare_parameter("initial_soc_pct", 85.0)
        self.declare_parameter("capacity_ah", 10.0)
        self.declare_parameter("cells_in_series", 3)
        self.declare_parameter("idle_current_a", 1.2)
        self.declare_parameter("current_a_per_mps", 8.0)
        self.declare_parameter("current_a_per_radps", 3.0)
        self.declare_parameter("internal_resistance_ohm", 0.06)
        self.declare_parameter("divider_gain", 5.0)
        self.declare_parameter("noise_mv", 2.0)
        self.declare_parameter("seed", 7)
        self.declare_parameter("rate_hz", 5.0)
        p = lambda name: self.get_parameter(name).value  # noqa: E731
        self._model = BatterySimModel(
            initial_soc_pct=float(p("initial_soc_pct")),
            capacity_ah=float(p("capacity_ah")),
            cells_in_series=int(p("cells_in_series")),
            idle_current_a=float(p("idle_current_a")),
            current_a_per_mps=float(p("current_a_per_mps")),
            current_a_per_radps=float(p("current_a_per_radps")),
            internal_resistance_ohm=float(p("internal_resistance_ohm")),
            divider_gain=float(p("divider_gain")),
            noise_mv=float(p("noise_mv")),
            seed=int(p("seed")),
        )
        self._speed = (0.0, 0.0)
        self._pub = self.create_publisher(BatteryAdcSample, "/battery/adc_sample", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self._period = 1.0 / float(p("rate_hz"))
        self.create_timer(self._period, self._tick)

    def _on_odom(self, msg: Odometry) -> None:
        self._speed = (float(msg.twist.twist.linear.x), float(msg.twist.twist.angular.z))

    def _tick(self) -> None:
        sample = BatteryAdcSample()
        sample.header.stamp = self.get_clock().now().to_msg()
        sample.mcu_millis = (self.get_clock().now().nanoseconds // 1_000_000) & 0xFFFFFFFF
        sample.pin_millivolts = float(round(self._model.step(self._period, *self._speed)))
        sample.sample_count = 32
        self._pub.publish(sample)


def main() -> None:
    rclpy.init()
    node = BatterySimNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
