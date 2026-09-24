#!/usr/bin/env python3
"""Watch one robot's whole command chain at once, one line per second.

Nav2 on this robot is a chain of four topics, and "the robot does not move"
always means one of them is empty or zero:

    controller_server ---> /<robot>/cmd_vel            (raw MPPI output)
    velocity_smoother ---> /<robot>/cmd_vel_smoothed   (rate-limited)
    collision_monitor ---> /<robot>/cmd_vel            (same topic, safety layer)
    cmd_vel_stamper   ---> /<robot>/<controller>/cmd_vel  (TwistStamped)

Instead of echoing them one at a time (and wondering whether the gap between
two echoes hid the interesting bit), this prints them side by side together
with the ground-truth odom and the collision monitor's stop/slowdown flags.

Usage:
    /usr/bin/python3 tools/chain_probe.py light_1 --seconds 20
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE)


class ChainProbe(Node):
    def __init__(self, robot, controller):
        super().__init__("chain_probe")
        self.data = {}
        self.subs = [
            ("raw", Twist, f"/{robot}/cmd_vel", self.on_twist("raw")),
            ("smoothed", Twist, f"/{robot}/cmd_vel_smoothed", self.on_twist("smoothed")),
            ("ctrl", TwistStamped, f"/{robot}/{controller}/cmd_vel",
             self.on_twist_stamped("ctrl")),
            ("odom", Odometry, f"/{robot}/odom", self.on_odom),
            ("stop", Bool, f"/{robot}/polygon_stop", self.on_flag("stop")),
            ("slow", Bool, f"/{robot}/polygon_slowdown", self.on_flag("slow")),
        ]
        for _, cls, topic, cb in self.subs:
            self.create_subscription(cls, topic, cb, QOS)

    def on_twist(self, key):
        def cb(msg):
            self.data[key] = (time.time(), msg.linear.x, msg.angular.z)
        return cb

    def on_twist_stamped(self, key):
        def cb(msg):
            self.data[key] = (time.time(), msg.twist.linear.x, msg.twist.angular.z)
        return cb

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        p = msg.pose.pose.position
        self.data["odom"] = (time.time(), p.x, p.y, 2 * math.atan2(q.z, q.w))

    def on_flag(self, key):
        def cb(msg):
            self.data[key] = (time.time(), msg.data)
        return cb

    def line(self):
        def tw(k):
            v = self.data.get(k)
            if not v or time.time() - v[0] > 3.0:
                return "      --      "
            return f"x={v[1]:+.3f} w={v[2]:+.3f}"

        od = self.data.get("odom")
        odom = (f"x={od[1]:+6.2f} y={od[2]:+6.2f} yaw={od[3]:+.2f}"
                if od and time.time() - od[0] < 3.0 else "   odom stale ")

        def flag(k):
            v = self.data.get(k)
            return "-" if not v else ("STOP" if v[1] else "ok")

        return (f"  {tw('raw')} | {tw('smoothed')} | {tw('ctrl')} | {odom} "
                f"| {flag('stop')}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("robot")
    ap.add_argument("--controller", default=None,
                    help="drive controller name; guessed from the topic list if omitted")
    ap.add_argument("--seconds", type=float, default=20.0)
    a = ap.parse_args()

    controller = a.controller
    if controller is None:
        # Try both, the unused one simply never fires.
        controller = "diff_drive_controller"

    rclpy.init()
    node = ChainProbe(a.robot, controller)
    print(f"robot {a.robot}, controller {controller}")
    print("         cmd_vel(raw)    |  cmd_vel_smoothed | controller cmd_vel "
          "| ground-truth odom       | stop")
    end = time.time() + a.seconds
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        print(node.line(), flush=True)
        time.sleep(1.0)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
