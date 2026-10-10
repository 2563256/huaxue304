#!/usr/bin/python3
"""Measure what the mecanum base actually does with a commanded twist.

Publishes a TwistStamped straight to the controller's `reference` topic - the
last hop before ros2_control - and integrates the ground-truth odometry over
each phase. Going in there rather than through Nav2 is the point: it isolates
the plant from MPPI, the smoother and the collision monitor, so a shortfall
here is a modelling problem and a shortfall anywhere else is a controller one.

Run it against a live simulation (see RUNBOOK.md for the environment):

    python3 tools/omni_probe.py --robot omni_1
"""

import argparse
import math
import threading
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Probe(Node):
    def __init__(self, robot):
        super().__init__("omni_probe")
        # The controller runs on the simulation clock and its reference_timeout
        # is measured in that clock, so a stamp taken from the wall clock makes
        # every command look stale and the base simply does not move.
        self.set_parameters([
            rclpy.parameter.Parameter(
                "use_sim_time", rclpy.Parameter.Type.BOOL, True
            )
        ])
        self.robot = robot
        self.pose = None
        self.ctrl_pose = None
        self.pub = self.create_publisher(
            TwistStamped, f"/{robot}/mecanum_drive_controller/reference", 10
        )
        self.create_subscription(
            Odometry, f"/{robot}/odom", self.on_odom, 10
        )
        # The controller's OWN odometry, integrated from wheel positions with
        # its kinematic model. Comparing it with the ground truth is the whole
        # test: if the controller believes it turned 1.2 rad while Gazebo
        # turned 2.4, the model does not describe the base.
        self.create_subscription(
            Odometry, f"/{robot}/mecanum_drive_controller/odometry",
            self.on_ctrl_odom, 10,
        )
        self._lock = threading.Lock()

    def on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def get_pose(self):
        with self._lock:
            return self.pose

    def on_ctrl_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.ctrl_pose = (msg.pose.pose.position.x,
                              msg.pose.pose.position.y, yaw)

    def get_ctrl_pose(self):
        with self._lock:
            return self.ctrl_pose

    def command(self, vx, vy, wz, duration):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        start_sim = self.get_clock().now().nanoseconds / 1e9
        end = time.time() + duration
        while time.time() < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)
            # Spin *while* driving: without this the odometry callback never
            # runs during a phase and every measurement comes back as a flat
            # zero, which looks exactly like a robot that cannot move.
            rclpy.spin_once(self, timeout_sec=0.05)
        elapsed_sim = self.get_clock().now().nanoseconds / 1e9 - start_sim
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(stop)
        settle = time.time() + 0.6
        while time.time() < settle:
            rclpy.spin_once(self, timeout_sec=0.05)
        return elapsed_sim

    def phase(self, label, vx, vy, wz, duration):
        before = self.get_pose()
        before_c = self.get_ctrl_pose()
        elapsed_sim = self.command(vx, vy, wz, duration)
        after = self.get_pose()
        after_c = self.get_ctrl_pose()
        if before is None or after is None:
            print(f"  {label:22s} no odometry")
            return
        dx_w, dy_w = after[0] - before[0], after[1] - before[1]
        c, s = math.cos(before[2]), math.sin(before[2])
        dx_b = c * dx_w + s * dy_w
        dy_b = -s * dx_w + c * dy_w
        dyaw = math.atan2(math.sin(after[2] - before[2]),
                          math.cos(after[2] - before[2]))
        if elapsed_sim <= 0:
            print(f"  {label:22s} simulation clock did not advance; no measurement")
            return
        gain_x = dx_b / (vx * elapsed_sim) if vx else float("nan")
        gain_y = dy_b / (vy * elapsed_sim) if vy else float("nan")
        gain_w = dyaw / (wz * elapsed_sim) if wz else float("nan")
        extra = ""
        if before_c is not None and after_c is not None:
            c_dyaw = math.atan2(math.sin(after_c[2] - before_c[2]),
                                math.cos(after_c[2] - before_c[2]))
            extra = (f"   [controller thinks dyaw={c_dyaw:+.3f}"
                     f" gain={c_dyaw / (wz * elapsed_sim):.2f}]" if wz else
                     f"   [controller thinks dy={after_c[1] - before_c[1]:+.3f}]")
        print(f"  {label:22s} commanded ({vx:+.2f},{vy:+.2f},{wz:+.2f}) for "
              f"{duration:.1f}s wall / {elapsed_sim:.2f}s sim -> ground truth ({dx_b:+.3f},{dy_b:+.3f}) m, "
              f"dyaw {dyaw:+.3f}   gain=({gain_x:.2f},{gain_y:.2f},{gain_w:.2f})"
              f"{extra}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="omni_1")
    ap.add_argument("--seconds", type=float, default=2.0)
    a = ap.parse_args()

    rclpy.init()
    node = Probe(a.robot)
    deadline = time.time() + 15.0
    while node.get_pose() is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.get_pose() is None:
        raise SystemExit("no odometry - is the simulation running?")

    print(f"{a.robot} plant response (gain 1.00 means the base does exactly "
          f"what it is told):")
    node.phase("forward", 0.25, 0.0, 0.0, a.seconds)
    node.phase("reverse", -0.20, 0.0, 0.0, a.seconds)
    node.phase("strafe +y (raw)", 0.0, 0.25, 0.0, a.seconds)
    node.phase("strafe -y (raw)", 0.0, -0.25, 0.0, a.seconds)
    node.phase("spin +z", 0.0, 0.0, 0.60, a.seconds)
    node.phase("spin -z", 0.0, 0.0, -0.60, a.seconds)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
