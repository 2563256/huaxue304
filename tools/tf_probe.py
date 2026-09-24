#!/usr/bin/env python3
"""Print where the TF tree says a robot is, next to where it really is.

Nav2 plans in the `map` frame and gets there through the chain
`map -> <robot>/odom -> <robot>/base_footprint`. Two parts of that chain are
written by two different nodes:

  map -> odom                bridge.launch.py, a static transform at the spawn
  odom -> base_footprint     the ground-truth odom relay (spawn-relative)

so the composition must equal the robot's real world pose. If it does not, the
robot drives to a goal that is systematically offset - the classic symptom
being a robot that sails past its goal and then reports "failed to make
progress" while sitting in completely open floor.

Usage:
    /usr/bin/python3 tools/tf_probe.py light_1 --seconds 5
"""

import argparse
import math
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class TfProbe(Node):
    def __init__(self, robot):
        super().__init__("tf_probe")
        self.robot = robot
        self.tf = {}
        static_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 100)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf, static_qos)
        self.odom = None
        self.create_subscription(Odometry, f"/{robot}/odom", self.on_odom, 10)

    def on_tf(self, msg):
        for t in msg.transforms:
            self.tf[(t.header.frame_id, t.child_frame_id)] = t

    def on_odom(self, msg):
        self.odom = msg

    def compose(self):
        odom = self.robot + "/odom"
        base = self.robot + "/base_footprint"
        m2o = self.tf.get(("map", odom))
        o2b = self.tf.get((odom, base))
        if m2o is None or o2b is None:
            return None, (m2o is not None), (o2b is not None)
        x = m2o.transform.translation.x + o2b.transform.translation.x
        y = m2o.transform.translation.y + o2b.transform.translation.y
        # Both rotations are pure yaw here, so angles add.
        q1, q2 = m2o.transform.rotation, o2b.transform.rotation
        yaw = (2 * math.atan2(q1.z, q1.w)) + (2 * math.atan2(q2.z, q2.w))
        return (x, y, math.atan2(math.sin(yaw), math.cos(yaw))), True, True

    def report(self):
        pose, has_m2o, has_o2b = self.compose()
        if pose is None:
            return (f"TF incomplete: map->{self.robot}/odom present={has_m2o}, "
                    f"{self.robot}/odom->base present={has_o2b}")
        x, y, yaw = pose
        out = f"TF says map->base_footprint = ({x:+.3f}, {y:+.3f}, yaw {yaw:+.3f})"
        if self.odom is not None:
            p = self.odom.pose.pose.position
            out += f"   [odom (spawn-relative) = ({p.x:+.3f}, {p.y:+.3f})]"
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("robot")
    ap.add_argument("--seconds", type=float, default=5.0)
    a = ap.parse_args()

    rclpy.init()
    node = TfProbe(a.robot)
    end = time.time() + a.seconds
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    print(node.report())
    frames = sorted({f for pair in node.tf for f in pair if a.robot in f or f == "map"})
    print(f"frames involving map/{a.robot}: {frames}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
