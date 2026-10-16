"""GY-BNO055 IMU driver on Raspberry Pi I2C (no apt Python libs required).

Publishes sensor_msgs/Imu on /imu/data with frame_id imu_link.
Verified chip: I2C-1 address 0x29, CHIP_ID 0xA0 (docs/calibration.md §8).
"""

from __future__ import annotations

import fcntl
import math
import os
import struct
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# Linux I2C
I2C_SLAVE = 0x0703

# BNO055 register map (Adafruit / Bosch)
REG_CHIP_ID = 0x00
REG_PAGE_ID = 0x07
REG_ACC_DATA = 0x08
REG_GYR_DATA = 0x14
REG_QUA_DATA = 0x20
REG_UNIT_SEL = 0x3B
REG_OPR_MODE = 0x3D
REG_PWR_MODE = 0x3E
REG_SYS_TRIGGER = 0x3F

CHIP_ID_EXPECTED = 0xA0
MODE_CONFIG = 0x00
MODE_NDOF = 0x0C

# Scale factors (Android orientation / m/s^2 / dps units after UNIT_SEL)
ACC_SCALE = 100.0  # 1 LSB = 0.01 m/s^2
GYR_SCALE = 16.0  # 1 LSB = 1/16 deg/s
QUA_SCALE = 16384.0  # 2^14


class Bno055Node(Node):
    """Read BNO055 over /dev/i2c-* and publish Imu."""

    def __init__(self) -> None:
        super().__init__("bno055_node")

        self.declare_parameter("i2c_bus", 1)
        self.declare_parameter("i2c_address", 0x29)
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("orientation_covariance", 0.01)
        self.declare_parameter("angular_velocity_covariance", 0.02)
        self.declare_parameter("linear_acceleration_covariance", 0.04)

        self._bus = int(self.get_parameter("i2c_bus").value)
        self._address = int(self.get_parameter("i2c_address").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        rate = float(self.get_parameter("publish_rate_hz").value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("publish_rate_hz must be > 0")

        self._ori_cov = float(self.get_parameter("orientation_covariance").value)
        self._gyr_cov = float(
            self.get_parameter("angular_velocity_covariance").value
        )
        self._acc_cov = float(
            self.get_parameter("linear_acceleration_covariance").value
        )

        self._device = f"/dev/i2c-{self._bus}"
        self._fd: Optional[int] = None
        self._publisher = self.create_publisher(Imu, "/imu/data", 10)

        self._open_bus()
        self._configure_sensor()

        period = 1.0 / rate
        self._timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f"BNO055 ready on {self._device} addr=0x{self._address:02X} "
            f"frame={self._frame_id} rate={rate:.1f} Hz"
        )

    def _open_bus(self) -> None:
        try:
            self._fd = os.open(self._device, os.O_RDWR)
            fcntl.ioctl(self._fd, I2C_SLAVE, self._address)
        except OSError as exc:
            raise RuntimeError(
                f"Cannot open {self._device} addr=0x{self._address:02X}: {exc}. "
                "Check wiring, dtparam=i2c_arm=on, and dialout group."
            ) from exc

    def _write_reg(self, register: int, value: int) -> None:
        assert self._fd is not None
        os.write(self._fd, bytes([register & 0xFF, value & 0xFF]))

    def _read_regs(self, register: int, length: int) -> bytes:
        assert self._fd is not None
        os.write(self._fd, bytes([register & 0xFF]))
        data = os.read(self._fd, length)
        if len(data) != length:
            raise OSError(
                f"I2C short read at 0x{register:02X}: got {len(data)}/{length}"
            )
        return data

    def _configure_sensor(self) -> None:
        chip_id = self._read_regs(REG_CHIP_ID, 1)[0]
        if chip_id != CHIP_ID_EXPECTED:
            raise RuntimeError(
                f"Unexpected CHIP_ID 0x{chip_id:02X} (want 0x{CHIP_ID_EXPECTED:02X}). "
                "Is GY-BNO055 at this address?"
            )

        # Enter CONFIG, select page 0, set units, then NDOF fusion mode.
        self._write_reg(REG_OPR_MODE, MODE_CONFIG)
        time.sleep(0.025)
        self._write_reg(REG_PAGE_ID, 0x00)
        self._write_reg(REG_PWR_MODE, 0x00)  # NORMAL
        # Unit select: Android orientation, deg/s gyro, m/s^2 accel (default-ish)
        self._write_reg(REG_UNIT_SEL, 0x00)
        self._write_reg(REG_SYS_TRIGGER, 0x00)
        time.sleep(0.010)
        self._write_reg(REG_OPR_MODE, MODE_NDOF)
        time.sleep(0.025)
        self.get_logger().info("BNO055 CHIP_ID OK, OPR_MODE=NDOF")

    def _on_timer(self) -> None:
        try:
            acc_raw = self._read_regs(REG_ACC_DATA, 6)
            gyr_raw = self._read_regs(REG_GYR_DATA, 6)
            qua_raw = self._read_regs(REG_QUA_DATA, 8)
        except OSError as exc:
            self.get_logger().warn(f"BNO055 read failed: {exc}", throttle_duration_sec=2.0)
            return

        ax, ay, az = struct.unpack("<hhh", acc_raw)
        gx, gy, gz = struct.unpack("<hhh", gyr_raw)
        qw, qx, qy, qz = struct.unpack("<hhhh", qua_raw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        msg.orientation.w = qw / QUA_SCALE
        msg.orientation.x = qx / QUA_SCALE
        msg.orientation.y = qy / QUA_SCALE
        msg.orientation.z = qz / QUA_SCALE
        msg.orientation_covariance[0] = self._ori_cov
        msg.orientation_covariance[4] = self._ori_cov
        msg.orientation_covariance[8] = self._ori_cov

        # deg/s -> rad/s
        msg.angular_velocity.x = math.radians(gx / GYR_SCALE)
        msg.angular_velocity.y = math.radians(gy / GYR_SCALE)
        msg.angular_velocity.z = math.radians(gz / GYR_SCALE)
        msg.angular_velocity_covariance[0] = self._gyr_cov
        msg.angular_velocity_covariance[4] = self._gyr_cov
        msg.angular_velocity_covariance[8] = self._gyr_cov

        msg.linear_acceleration.x = ax / ACC_SCALE
        msg.linear_acceleration.y = ay / ACC_SCALE
        msg.linear_acceleration.z = az / ACC_SCALE
        msg.linear_acceleration_covariance[0] = self._acc_cov
        msg.linear_acceleration_covariance[4] = self._acc_cov
        msg.linear_acceleration_covariance[8] = self._acc_cov

        self._publisher.publish(msg)

    def destroy_node(self) -> bool:
        try:
            if self._fd is not None:
                try:
                    self._write_reg(REG_OPR_MODE, MODE_CONFIG)
                except OSError:
                    pass
                os.close(self._fd)
                self._fd = None
        finally:
            return super().destroy_node()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[Bno055Node] = None
    try:
        node = Bno055Node()
        rclpy.spin(node)
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as exc:
        if node is None:
            print(f"bno055_node error: {exc}")
        else:
            node.get_logger().error(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
