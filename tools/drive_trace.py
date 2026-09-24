#!/usr/bin/python3
"""Log commanded vs achieved motion, so a stopped robot can be diagnosed.

"The robot is stuck" has two completely different causes and they look the
same from the outside:

  * the stack is commanding zero - the controller found no feasible rollout,
    the collision monitor is holding the base, the costmap thinks the robot is
    in collision, or the command stream went stale; or
  * the stack is commanding motion and the base is not delivering it - the
    wheels are slipping, something is physically in the way, or the plant does
    not match the controller's model.

This records both, at 10 Hz, per robot:

  cmd_*      /<robot>/cmd_vel_smoothed  - what Nav2 asked for
  post_*     /<robot>/cmd_vel           - what survived the collision monitor
  x y yaw    ground-truth odometry

Usage (against a live simulation):
    python3 tools/drive_trace.py --robot omni_1 --seconds 900 --out /tmp/omni.csv
"""

import argparse
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


class DriveTrace(Node):
    def __init__(self, robot, sink):
        super().__init__("drive_trace")
        self.set_parameters([
            rclpy.parameter.Parameter(
                "use_sim_time", rclpy.Parameter.Type.BOOL, True
            )
        ])
        self.robot = robot
        self.sink = sink
        self.cmd = (0.0, 0.0, 0.0)
        self.post = (0.0, 0.0, 0.0)
        self.pose = None
        self.create_subscription(
            Twist, f"/{robot}/cmd_vel_smoothed", self.on_cmd, 10)
        self.create_subscription(
            Twist, f"/{robot}/cmd_vel", self.on_post, 10)
        self.create_subscription(
            Odometry, f"/{robot}/odom", self.on_odom, 10)

    def on_cmd(self, m):
        self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def on_post(self, m):
        self.post = (m.linear.x, m.linear.y, m.angular.z)

    def on_odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def write(self):
        if self.pose is None:
            return
        stamp = self.get_clock().now().nanoseconds * 1e-9
        self.sink.write(
            f"{stamp:.3f},{self.pose[0]:.4f},{self.pose[1]:.4f},"
            f"{self.pose[2]:.4f},{self.cmd[0]:.4f},{self.cmd[1]:.4f},"
            f"{self.cmd[2]:.4f},{self.post[0]:.4f},{self.post[1]:.4f},"
            f"{self.post[2]:.4f}\n"
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="omni_1")
    ap.add_argument("--seconds", type=float, default=900.0)
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument(
        "--out",
        default=os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "generated",
            "drive_trace.csv",
        ),
    )
    a = ap.parse_args()

    rclpy.init()
    with open(a.out, "w") as sink:
        sink.write("t,x,y,yaw,cmd_vx,cmd_vy,cmd_wz,post_vx,post_vy,post_wz\n")
        node = DriveTrace(a.robot, sink)
        period = 1.0 / a.rate
        end = time.time() + a.seconds
        nxt = time.time()
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=period)
            if time.time() >= nxt:
                node.write()
                nxt += period
        node.destroy_node()
    rclpy.shutdown()
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
