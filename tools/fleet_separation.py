#!/usr/bin/python3
"""Measure how close the robots actually get to each other.

"The avoidance is not very good" is not something you can tune against. This
turns it into numbers: it samples every robot's ground-truth odometry (the same
source the local costmaps and the collision monitor use) and reports, over the
whole run, the distribution of the closest inter-robot distance.

Read it like this:

  * min / p1  - how close the fleet ever got. Below ~0.35 m means the two
    robots were touching bumpers: the collision monitor is doing the avoiding,
    not the planner.
  * time under 0.6 m - how long the fleet spent in the "neither robot wants to
    move and neither wants to give way" band. This is the number that should
    fall when avoidance improves.
  * the worst pair - a pair that dominates the low-distance samples is a
    bottleneck in the map, not a controller problem.

Run it alongside a simulation (see RUNBOOK.md for the environment):

    python3 tools/fleet_separation.py --seconds 600
"""

import argparse
import itertools
import math
import os
import time

import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node


def load_spawns(path):
    """Where each robot was created, straight from the profiles file.

    This is not optional: /<robot>/odom is published by the odometry relay as
    the displacement from THAT robot's own spawn (world = spawn + R(odom)), so
    two robots both sitting on their bays both read (0, 0) and comparing the
    raw odometry says the fleet is stacked on one spot. The spawn offset has to
    be added back before any distance means anything.
    """
    with open(path) as f:
        profiles = yaml.safe_load(f)
    out = {}
    for entry in profiles["fleet"]:
        sx, sy, syaw = entry["spawn"]
        out[entry["name"]] = (float(sx), float(sy), float(syaw))
    return out


class Separation(Node):
    def __init__(self, robots, spawns):
        super().__init__("fleet_separation")
        self.set_parameters([
            rclpy.parameter.Parameter(
                "use_sim_time", rclpy.Parameter.Type.BOOL, True
            )
        ])
        self.robots = robots
        self.spawns = spawns
        self.poses = {}
        for name in robots:
            self.create_subscription(
                Odometry, f"/{name}/odom",
                lambda msg, n=name: self.on_odom(n, msg), 10,
            )
        self.samples = []
        self.worst = {}

    def on_odom(self, name, msg):
        sx, sy, syaw = self.spawns[name]
        dx = msg.pose.pose.position.x
        dy = msg.pose.pose.position.y
        c, s = math.cos(syaw), math.sin(syaw)
        self.poses[name] = (sx + c * dx - s * dy, sy + s * dx + c * dy)

    def sample(self):
        if len(self.poses) < len(self.robots):
            return
        for a, b in itertools.combinations(sorted(self.robots), 2):
            (ax, ay), (bx, by) = self.poses[a], self.poses[b]
            d = math.hypot(ax - bx, ay - by)
            self.samples.append((d, f"{a}-{b}"))
            key = f"{a}-{b}"
            if key not in self.worst or d < self.worst[key]:
                self.worst[key] = d

    def report(self):
        if not self.samples:
            print("no samples - is the fleet running?")
            return
        moving = sorted(d for d, _ in self.samples)
        n = len(moving)
        m = len(moving)
        print(f"closest inter-robot distance over {n} samples "
              f"(map frame, spawn offsets applied):")
        print(f"   min {moving[0]:.2f} m   p1 {moving[int(0.01 * m)]:.2f} m   "
              f"p5 {moving[int(0.05 * m)]:.2f} m   median "
              f"{moving[m // 2]:.2f} m")
        for threshold in (0.4, 0.6, 0.8, 1.0):
            under = sum(1 for d in moving if d < threshold)
            print(f"   {under * 100.0 / m:5.1f}% of time closer than "
                  f"{threshold:.1f} m")
        print("   closest approach per pair:")
        for key, d in sorted(self.worst.items(), key=lambda kv: kv[1]):
            print(f"     {key:20s} {d:.2f} m")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", nargs="+",
                    default=["light_1", "light_2", "heavy_1", "omni_1"])
    ap.add_argument("--seconds", type=float, default=600.0)
    ap.add_argument("--rate", type=float, default=10.0)
    ap.add_argument("--profiles", default=None,
                    help="robot_profiles.yaml (default: the installed one)")
    a = ap.parse_args()

    profiles = a.profiles
    if profiles is None:
        from ament_index_python.packages import get_package_share_directory
        profiles = os.path.join(
            get_package_share_directory("hetero_fleet_description"),
            "config", "robot_profiles.yaml",
        )
    spawns = load_spawns(profiles)

    rclpy.init()
    node = Separation(a.robots, spawns)
    period = 1.0 / a.rate
    end = time.time() + a.seconds
    next_sample = time.time()
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=period)
        if time.time() >= next_sample:
            node.sample()
            next_sample += period
    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
