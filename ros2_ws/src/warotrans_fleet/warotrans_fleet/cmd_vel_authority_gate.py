"""Gate /cmd_vel_safe → /cmd_vel theo ``/traffic/authority_ok`` (Active Mode).

Fail-safe: chỉ forward khi authority_ok == True VÀ tin nhắn authority_ok còn mới (<= max_age).
Mất tin nhắn (supervisor chết / node treo) → age tăng → tự chặn và gửi Twist 0.

Phần logic (``AuthorityGateLogic``, ``should_forward``) là pure Python; node ROS chỉ được định
nghĩa khi import được rclpy, để unit test chạy không cần ROS.

Gate này KHÔNG đổi TF, KHÔNG đụng firmware; chỉ nằm giữa controller và topic /cmd_vel hiện có.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

DEFAULT_MAX_AGE_SEC = 0.3


def should_forward(
    authority_ok: bool,
    authority_ok_age_sec: Optional[float],
    max_age: float = DEFAULT_MAX_AGE_SEC,
) -> bool:
    """True chỉ khi authority_ok là True và tuổi tin nhắn nằm trong [0, max_age]."""
    if authority_ok is not True:
        return False
    if authority_ok_age_sec is None or authority_ok_age_sec != authority_ok_age_sec:  # None / NaN
        return False
    if authority_ok_age_sec < 0.0:  # đồng hồ nhảy lùi → không tin
        return False
    return authority_ok_age_sec <= max_age


@dataclass(frozen=True)
class GateDecision:
    forward: bool
    publish_zero: bool  # True: gate đang chặn và cần gửi Twist 0 (dừng chủ động)


class AuthorityGateLogic:
    """Giữ trạng thái authority_ok mới nhất + thời điểm nhận (monotonic)."""

    def __init__(self, *, max_age_sec: float = DEFAULT_MAX_AGE_SEC, zero_hold_sec: float = 1.0) -> None:
        if max_age_sec <= 0:
            raise ValueError("max_age_sec must be > 0")
        self._max_age = max_age_sec
        self._zero_hold = zero_hold_sec
        self._authority_ok = False
        self._received_at: Optional[float] = None
        self._blocked_since: Optional[float] = None

    @property
    def authority_ok(self) -> bool:
        return self._authority_ok

    def on_authority_ok(self, value: bool, now: float) -> None:
        self._authority_ok = bool(value)
        self._received_at = now

    def age(self, now: float) -> Optional[float]:
        return None if self._received_at is None else now - self._received_at

    def decide(self, now: float) -> GateDecision:
        forward = should_forward(self._authority_ok, self.age(now), self._max_age)
        if forward:
            self._blocked_since = None
            return GateDecision(True, False)
        if self._blocked_since is None:
            self._blocked_since = now
        # Gửi 0 trong zero_hold_sec đầu sau khi bị chặn (đủ để motor nhận lệnh dừng), rồi im lặng
        # để không tranh quyền /cmd_vel với nguồn khác khi robot đã đứng yên.
        return GateDecision(False, now - self._blocked_since <= self._zero_hold)


# ----------------------------------------------------------------------------- ROS node (optional)

try:  # pragma: no cover - chỉ chạy khi có ROS
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import Bool

    _HAS_RCLPY = True
except ImportError:  # pragma: no cover
    _HAS_RCLPY = False


if _HAS_RCLPY:  # pragma: no cover - cần rclpy; logic đã được test qua AuthorityGateLogic

    class CmdVelAuthorityGate(Node):
        def __init__(self) -> None:
            super().__init__("cmd_vel_authority_gate")
            self.declare_parameter("input_topic", "/cmd_vel_safe")
            self.declare_parameter("output_topic", "/cmd_vel")
            self.declare_parameter("authority_ok_topic", "/traffic/authority_ok")
            self.declare_parameter("max_authority_age_sec", DEFAULT_MAX_AGE_SEC)
            self.declare_parameter("zero_hold_sec", 1.0)
            self.declare_parameter("gate_hz", 20.0)

            max_age = float(self.get_parameter("max_authority_age_sec").value)
            zero_hold = float(self.get_parameter("zero_hold_sec").value)
            gate_hz = float(self.get_parameter("gate_hz").value)
            if max_age <= 0 or gate_hz <= 0 or zero_hold < 0:
                raise ValueError("max_authority_age_sec > 0, gate_hz > 0, zero_hold_sec >= 0")

            self._logic = AuthorityGateLogic(max_age_sec=max_age, zero_hold_sec=zero_hold)
            self._pub = self.create_publisher(Twist, str(self.get_parameter("output_topic").value), 10)
            self.create_subscription(Twist, str(self.get_parameter("input_topic").value), self._on_cmd, 10)
            self.create_subscription(Bool, str(self.get_parameter("authority_ok_topic").value), self._on_ok, 10)
            self.create_timer(1.0 / gate_hz, self._on_timer)
            self._was_forwarding = False

        def _on_ok(self, msg: "Bool") -> None:
            self._logic.on_authority_ok(bool(msg.data), time.monotonic())

        def _on_cmd(self, msg: "Twist") -> None:
            decision = self._logic.decide(time.monotonic())
            if decision.forward:
                self._pub.publish(msg)
                self._was_forwarding = True
            else:
                self._publish_zero()

        def _on_timer(self) -> None:
            decision = self._logic.decide(time.monotonic())
            if not decision.forward and decision.publish_zero:
                self._publish_zero()
            if not decision.forward and self._was_forwarding:
                self.get_logger().warning("authority gate BLOCKING /cmd_vel_safe (authority_ok false/stale)")
            self._was_forwarding = decision.forward

        def _publish_zero(self) -> None:
            self._pub.publish(Twist())


def main(args=None) -> None:
    if not _HAS_RCLPY:
        raise RuntimeError("rclpy not available; cmd_vel_authority_gate node needs a ROS 2 environment")
    rclpy.init(args=args)
    node = CmdVelAuthorityGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
