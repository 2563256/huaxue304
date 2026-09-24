#!/usr/bin/python3
"""Publish each robot's scan, plus a separate "where the other robots are" scan.

Two topics per robot, and the SPLIT between them is the whole point:

  /<robot>/scan_fixed       the real lidar, only with the frame corrected.
  /<robot>/robot_obstacles  synthetic returns for the other two vehicles.

FRAME (applies to both). Gazebo stamps the scans with a bare "lidar_link",
which is not in the TF tree - the real frame is "<robot>/lidar_link". Without
the rewrite the costmap's message filter cannot transform anything and silently
drops every scan.

WHY THE OTHER ROBOTS NEED SYNTHESISING. The fleet is deliberately
heterogeneous and that breaks robot-to-robot perception: after the 0.31-0.42
rescale the 2D lidars sit at (heights above the floor, measured off the URDFs)

    light_1             0.20 m
    omni_1              0.23 m
    heavy_1             0.24 m

while the bodies are 0.10-0.20 m tall. A 2D lidar is a single slice at its own
height, so anything shorter than it is not "hard to see" - it is invisible.
Each robot's ground-truth odometry is therefore written into the others'
robot_obstacles scan as a short run of returns at the near face of its
footprint.

WHY IT IS A SEPARATE TOPIC. The first version injected these returns straight
into scan_fixed, which every consumer reads - including each robot's GLOBAL
costmap. That made a parked vehicle a permanent, lethal obstacle in the
planning grid, so a robot that came back to a shared pickup/dropoff zone while
another was still standing there got

    GridBased plugin failed to plan from (-4.05, -7.49) to (-4.00, -6.00):
    "no valid path found"

for a 1.5 m move, and the task died after its single requeue. Only the LOCAL
costmap and the collision monitor subscribe to robot_obstacles; the global
costmap sees pure lidar, so planning is never blocked by a stationary
neighbour, while reactive avoidance and the emergency stop still work.
"""

import math
import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanFrameRelay(Node):
    def __init__(self):
        super().__init__("scan_frame_relay")
        share = get_package_share_directory("hetero_fleet_description")
        with open(os.path.join(share, "config", "robot_profiles.yaml")) as f:
            profiles = yaml.safe_load(f)
        types = profiles["robot_types"]

        # How far each robot's lidar sits AHEAD OF ITS BASE, in metres. This
        # used to be one constant (0.20) for the whole fleet, which was wrong
        # for every type: the real offsets are 0.057 (light), 0.087 (heavy) and
        # 0.076 (omni), so every synthetic teammate blob was drawn 11-14 cm too
        # far forward - i.e. up to a third of a robot's length of error in the
        # very signal the local costmaps and the collision monitor use to avoid
        # each other.
        #
        # Values are read off the URDFs (the lidar_link -> base_link chain): if
        # a lidar_mount_joint origin changes, change it here too. The parameter
        # stays as the fallback for a type that is not listed.
        self.default_lidar_dx = float(
            self.declare_parameter("lidar_dx", 0.20).value
        )
        self.lidar_dx_by_type = {
            "light_diff_amr": 0.057,
            "heavy_diff_amr": 0.087,
            "omni_mecanum_amr": 0.076,
        }

        self.robots = {}
        for entry in profiles["fleet"]:
            name = entry["name"]
            sx, sy, syaw = entry["spawn"]
            length, width = types[entry["type"]]["footprint"]
            self.robots[name] = {
                "spawn": (float(sx), float(sy), float(syaw)),
                "lidar_dx": self.lidar_dx_by_type.get(
                    entry["type"], self.default_lidar_dx
                ),
                # Inscribed radius, not the half-diagonal: the injected blob
                # is a keep-out marker, and using the circumscribed radius made
                # every teammate ~40% fatter than it is. With four robots in a
                # 26 x 10 m hall that inflated each local costmap into a sea of
                # lethal cells whenever two of them worked near each other, and
                # MPPI then found no feasible rollout at all ("Optimizer reset")
                # - both robots stood still waiting for the other one.
                "half_diag": 0.5 * min(length, width),
                "pose": None,          # (x, y, yaw) on the map
            }

        self.max_range = float(self.declare_parameter("max_range", 12.0).value)

        self.pubs = {}
        self.obstacle_pubs = {}
        for name in self.robots:
            self.pubs[name] = self.create_publisher(LaserScan, f"/{name}/scan_fixed", 10)
            self.obstacle_pubs[name] = self.create_publisher(
                LaserScan, f"/{name}/robot_obstacles", 10
            )
            self.create_subscription(
                LaserScan, f"/{name}/scan", lambda m, n=name: self.on_scan(m, n), 10
            )
            self.create_subscription(
                Odometry, f"/{name}/odom", lambda m, n=name: self.on_odom(n, m), 10
            )

    # ------------------------------------------------------------------ poses
    def on_odom(self, name, msg):
        """Ground-truth pose on the map: spawn + R(spawn_yaw) * odom."""
        sx, sy, syaw = self.robots[name]["spawn"]
        dx = msg.pose.pose.position.x
        dy = msg.pose.pose.position.y
        c0, s0 = math.cos(syaw), math.sin(syaw)
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self.robots[name]["pose"] = (
            sx + c0 * dx - s0 * dy,
            sy + s0 * dx + c0 * dy,
            yaw + syaw,
        )

    # ------------------------------------------------------------------ scan
    def on_scan(self, msg, name):
        msg.header.frame_id = f"{name}/lidar_link"
        # The real lidar goes out untouched - the global costmap plans from it.
        self.pubs[name].publish(msg)
        # Four local costmaps and four collision monitors depend on the
        # synthetic topic; one bad message must not take them all down.
        try:
            self.publish_robot_obstacles(msg, name)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"{name}: robot-obstacle scan skipped ({exc!r})",
                throttle_duration_sec=5.0,
            )

    def publish_robot_obstacles(self, scan, name):
        """Synthetic scan with one blob per other robot, in the lidar frame."""
        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = [float("inf")] * len(scan.ranges)

        ego = self.robots[name]["pose"]
        n = len(out.ranges)
        if ego is not None and n:
            ex, ey, eyaw = ego
            inc = out.angle_increment
            if inc > 0.0:
                c, s = math.cos(eyaw), math.sin(eyaw)
                for other, state in self.robots.items():
                    if other == name or state["pose"] is None:
                        continue
                    ox, oy, _ = state["pose"]
                    wx, wy = ox - ex, oy - ey
                    # The blob is drawn in the EGO robot's lidar frame, so the
                    # offset that comes off is the EGO's, not the teammate's.
                    bx = c * wx + s * wy - self.robots[name]["lidar_dx"]
                    by = -s * wx + c * wy
                    rng = math.hypot(bx, by)
                    half = state["half_diag"]
                    surface = rng - half
                    if surface > min(self.max_range, out.range_max):
                        continue
                    surface = max(surface, out.range_min)
                    bearing = math.atan2(by, bx)
                    spread = math.atan2(half, max(rng, half))
                    i0 = int(round((bearing - spread - out.angle_min) / inc))
                    i1 = int(round((bearing + spread - out.angle_min) / inc))
                    spans = [(i0, i1)] if i0 <= i1 else [(i0, n - 1), (0, i1)]
                    for a, b in spans:
                        for i in range(max(0, a), min(n - 1, b) + 1):
                            if out.ranges[i] > surface:
                                out.ranges[i] = surface
        self.obstacle_pubs[name].publish(out)


def main():
    rclpy.init()
    rclpy.spin(ScanFrameRelay())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
