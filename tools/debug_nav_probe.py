#!/usr/bin/env python3
# Debug probe (session f83311) — remove after the Nav2 issue is verified fixed.
import json, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped, PoseStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from tf2_ros import Buffer, TransformListener

LOG = sys.argv[1] if len(sys.argv) > 1 else "/tmp/debug-f83311.log"
RUN = sys.argv[2] if len(sys.argv) > 2 else "run2"


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


# #region agent log
def dlog(hyp, loc, msg, data):
    with open(LOG, "a") as f:
        f.write(json.dumps({"sessionId": "f83311", "runId": RUN, "hypothesisId": hyp, "location": loc,
                            "message": msg, "data": data, "timestamp": int(time.time() * 1000)}) + "\n")
# #endregion


class Probe(Node):
    def __init__(self):
        super().__init__("debug_nav_probe")
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.cmd = (0.0, 0.0)
        self.odom_w = 0.0
        self.goal = None
        self.gcm = None
        self.near_last_w = 0.0
        self.near_flips = 0
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.on_initialpose, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.on_gcm, latched)
        self.create_timer(0.2, self.tick)

    def robot(self):
        try:
            t = self.tf.lookup_transform("map", "base_footprint", rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation)
        except Exception:
            return None

    def cost_at(self, x, y):
        g = self.gcm
        if g is None:
            return None
        i = int((x - g.info.origin.position.x) / g.info.resolution)
        j = int((y - g.info.origin.position.y) / g.info.resolution)
        if 0 <= i < g.info.width and 0 <= j < g.info.height:
            return int(g.data[j * g.info.width + i])
        return "outside"

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.angular.z)

    def on_odom(self, m):
        self.odom_w = m.twist.twist.angular.z

    def on_gcm(self, m):
        self.gcm = m

    def on_initialpose(self, m):
        o = m.pose.pose.orientation
        # #region agent log
        dlog("K", "debug_nav_probe.py:on_initialpose", "initialpose received",
             {"frame": m.header.frame_id, "x": round(m.pose.pose.position.x, 3), "y": round(m.pose.pose.position.y, 3),
              "yaw_deg": round(math.degrees(yaw_of(o)), 1), "q": [o.x, o.y, o.z, o.w]})
        # #endregion

    def on_goal(self, m):
        self.goal = (m.pose.position.x, m.pose.position.y, yaw_of(m.pose.orientation))
        self.near_flips = 0
        r = self.robot()
        # #region agent log
        dlog("J", "debug_nav_probe.py:on_goal", "goal + costmap cost at start/goal",
             {"goal": [round(self.goal[0], 2), round(self.goal[1], 2), round(math.degrees(self.goal[2]), 1)],
              "robot": [round(r[0], 2), round(r[1], 2), round(math.degrees(r[2]), 1)] if r else None,
              "cost_goal": self.cost_at(self.goal[0], self.goal[1]),
              "cost_robot": self.cost_at(r[0], r[1]) if r else None})
        # #endregion

    def on_plan(self, m):
        p = m.poses
        if len(p) < 2:
            return
        length = sum(math.hypot(p[k + 1].pose.position.x - p[k].pose.position.x,
                                p[k + 1].pose.position.y - p[k].pose.position.y) for k in range(len(p) - 1))
        straight = math.hypot(p[-1].pose.position.x - p[0].pose.position.x, p[-1].pose.position.y - p[0].pose.position.y)
        costs = [self.cost_at(q.pose.position.x, q.pose.position.y) for q in p]
        unknown = sum(1 for c in costs if c == -1)
        outside = sum(1 for c in costs if c == "outside")
        num = [c for c in costs if isinstance(c, int) and c >= 0]
        # #region agent log
        dlog("I", "debug_nav_probe.py:on_plan", "plan stats",
             {"n": len(p), "len_m": round(length, 2), "straight_m": round(straight, 2),
              "ratio": round(length / straight, 2) if straight > 0.01 else None,
              "unknown_cells": unknown, "outside_cells": outside,
              "max_cost": max(num) if num else None, "mean_cost": round(sum(num) / len(num), 1) if num else None,
              "start": [round(p[0].pose.position.x, 2), round(p[0].pose.position.y, 2)],
              "mid": [round(p[len(p) // 2].pose.position.x, 2), round(p[len(p) // 2].pose.position.y, 2)],
              "end": [round(p[-1].pose.position.x, 2), round(p[-1].pose.position.y, 2)]})
        # #endregion

    def tick(self):
        if self.goal is None:
            return
        r = self.robot()
        if r is None:
            return
        d = math.hypot(self.goal[0] - r[0], self.goal[1] - r[1])
        cv, cw = self.cmd
        if d < 0.35 and (abs(cw) > 0.01 or abs(self.odom_w) > 0.05):
            if cw * self.near_last_w < 0:
                self.near_flips += 1
            if abs(cw) > 0.01:
                self.near_last_w = cw
            # #region agent log
            dlog("G,H", "debug_nav_probe.py:tick", "near-goal rotation",
                 {"dist": round(d, 3), "yaw_err_deg": round(math.degrees(wrap(self.goal[2] - r[2])), 1),
                  "cmd_v": round(cv, 3), "cmd_w": round(cw, 3), "odom_w": round(self.odom_w, 3),
                  "w_ratio": round(self.odom_w / cw, 2) if abs(cw) > 0.05 else None,
                  "sign_flips": self.near_flips, "robot_yaw_deg": round(math.degrees(r[2]), 1)})
            # #endregion


def main():
    rclpy.init()
    n = Probe()
    # #region agent log
    dlog("ALL", "debug_nav_probe.py:main", "probe started", {"log": LOG})
    # #endregion
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
