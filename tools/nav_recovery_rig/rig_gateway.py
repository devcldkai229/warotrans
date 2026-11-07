#!/usr/bin/env python3
"""Stand-in for robot_gateway in the rig: opens the authority gate, or not. Named robot_gateway because the gate
trusts only that publisher. Grant logic itself is covered by the real SemanticTrafficClient tests and the traffic
harness; this only drives the two answers the Nav2 stack can see.

  --authority ok|closed        what traffic/authority_ok says
  --recovery ok|<reason>       what traffic/authorize_backup and traffic/authorize_spin answer
  --recovery-backup <reason>   override for backup only (e.g. OUTSIDE_RESERVATION)
"""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class Gateway(Node):
    def __init__(self, a: argparse.Namespace) -> None:
        super().__init__("robot_gateway")
        self.a = a
        self.pub = self.create_publisher(Bool, "traffic/authority_ok", 10)
        self.create_timer(0.05, self._tick)
        # A closed gate means no grant: the real client refuses recovery then too.
        closed = "NO_GRANT" if a.authority != "ok" else ""
        self.create_service(Trigger, "traffic/authorize_backup", self._answer(closed or a.recovery_backup or a.recovery))
        self.create_service(Trigger, "traffic/authorize_spin", self._answer(closed or a.recovery))

    def _tick(self) -> None:
        self.pub.publish(Bool(data=self.a.authority == "ok"))

    @staticmethod
    def _answer(value: str):
        def cb(_req, res):
            res.success = value == "ok"
            res.message = "" if res.success else value
            return res
        return cb


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--authority", default="ok")
    p.add_argument("--recovery", default="ok")
    p.add_argument("--recovery-backup", default="")
    a = p.parse_args()
    rclpy.init()
    rclpy.spin(Gateway(a))


if __name__ == "__main__":
    main()
