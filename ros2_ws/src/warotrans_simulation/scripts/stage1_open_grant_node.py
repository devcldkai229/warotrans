#!/usr/bin/env python3
"""Stage-1 ONLY: publish /traffic/authority_ok=True so the Motion Authority Gate stays open.

Logged as STAGE1_OPEN_GRANT. Not a production traffic grant. TTL is soft (node lifetime);
kill the node or set enable:=false to close the gate.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class Stage1OpenGrant(Node):
    def __init__(self) -> None:
        super().__init__("stage1_open_grant")
        self.declare_parameter("topic", "/traffic/authority_ok")
        self.declare_parameter("hz", 10.0)
        self.declare_parameter("enable", True)
        topic = str(self.get_parameter("topic").value)
        hz = float(self.get_parameter("hz").value)
        self._enable = bool(self.get_parameter("enable").value)
        self._pub = self.create_publisher(Bool, topic, 10)
        period = 1.0 / max(hz, 1.0)
        self.create_timer(period, self._tick)
        self.get_logger().warn(
            "STAGE1_OPEN_GRANT active: publishing authority_ok=%s on %s at %.1f Hz "
            "(wall-clock; gate max_age is monotonic — not production traffic)"
            % (self._enable, topic, hz)
        )

    def _tick(self) -> None:
        msg = Bool()
        msg.data = bool(self._enable)
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = Stage1OpenGrant()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
