#!/usr/bin/env python3
"""Print what a topic is actually carrying, without the `ros2 topic echo` CLI.

Why this exists: with ~90 nodes on this machine the rclpy-based shell commands
sometimes fail to discover a publisher inside their timeout, so `ros2 topic
echo --once` prints nothing even when the topic is healthy. A plain rclpy node
started inside the same domain sees the data reliably, which is what debugging
a "the robot does not move" report needs.

Usage:
    python3 tools/topic_probe.py /light_1/odom
    python3 tools/topic_probe.py /light_1/scan --seconds 3 --count
    python3 tools/topic_probe.py /light_1/diff_drive_controller/cmd_vel
"""

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage

TYPES = {
    "/odom": Odometry,
    "/cmd_vel": Twist,
    "/scan": LaserScan,
    "amcl_pose": PoseWithCovarianceStamped,
    "/plan": Path,
    "costmap": OccupancyGrid,
    "/joint_states": JointState,
    "pose_stamped": PoseStamped,
    "twist_stamped": TwistStamped,
    "tf": TFMessage,
}


def pick_type(topic):
    for suffix, cls in TYPES.items():
        if suffix.startswith("/") and topic.endswith(suffix):
            return cls
    if topic.endswith("/cmd_vel"):
        return Twist
    if topic.endswith("/cmd_vel_stamped") or "controller" in topic and topic.endswith("cmd_vel"):
        return TwistStamped
    if topic.endswith("/gz_pose"):
        return PoseStamped
    if topic == "/gz/dynamic_pose":
        return TFMessage
    return Odometry


def describe(msg):
    if isinstance(msg, JointState):
        parts = [f"{n}={v:+.3f}" for n, v in zip(msg.name, msg.velocity)]
        return "joint velocities: " + ", ".join(parts)
    if isinstance(msg, Path):
        pts = msg.poses
        if not pts:
            return "path with 0 poses"
        a, b = pts[0].pose.position, pts[-1].pose.position
        return (f"path frame={msg.header.frame_id} n={len(pts)} "
                f"start=({a.x:+.2f},{a.y:+.2f}) goal=({b.x:+.2f},{b.y:+.2f})")
    if isinstance(msg, OccupancyGrid):
        info = msg.info
        data = __import__("numpy").array(msg.data, dtype=int).reshape(info.height, info.width)
        return (f"costmap frame={msg.header.frame_id} {info.width}x{info.height} @"
                f"{info.resolution:.2f} origin=({info.origin.position.x:.2f},"
                f"{info.origin.position.y:.2f}) lethal={(data >= 100).sum()} "
                f"free={(data == 0).sum()} unknown={(data < 0).sum()}")
    if isinstance(msg, PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return (f"pose frame={msg.header.frame_id} x={p.x:+.3f} y={p.y:+.3f} "
                f"yaw={2 * __import__('math').atan2(q.z, q.w):+.3f}")
    if isinstance(msg, Odometry):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        t = msg.twist.twist
        return (f"odom frame={msg.header.frame_id} child={msg.child_frame_id} "
                f"x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f} "
                f"yaw={2 * __import__('math').atan2(q.z, q.w):+.3f} "
                f"vx={t.linear.x:+.3f} wz={t.angular.z:+.3f}")
    if isinstance(msg, (Twist, TwistStamped)):
        tw = msg.twist if isinstance(msg, TwistStamped) else msg
        return (f"twist x={tw.linear.x:+.3f} y={tw.linear.y:+.3f} "
                f"z={tw.angular.z:+.3f}")
    if isinstance(msg, LaserScan):
        finite = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        return (f"scan frame={msg.header.frame_id} n={len(msg.ranges)} "
                f"valid={len(finite)} min={min(finite) if finite else float('nan'):.2f}")
    if isinstance(msg, PoseStamped):
        p = msg.pose.position
        return f"pose frame={msg.header.frame_id} x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f}"
    if isinstance(msg, TFMessage):
        return f"tf with {len(msg.transforms)} transforms"
    return repr(msg)[:120]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("topic")
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--count", action="store_true", help="also print the rate")
    ap.add_argument("--info", action="store_true",
                    help="also list who publishes/subscribes this topic")
    ap.add_argument("--every", type=float, default=0.0,
                    help="print every message, at most this often (seconds)")
    a = ap.parse_args()

    rclpy.init()
    node = Node("topic_probe")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    if "joint_states" in a.topic or a.topic.endswith("/scan"):
        # These are published with SensorDataQoS (best effort); a RELIABLE
        # subscriber never matches them and would report "no messages" on a
        # perfectly healthy topic.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
    seen = {"n": 0, "last": None}
    last_print = {"t": 0.0}

    def cb(msg, _topic=a.topic):
        seen["n"] += 1
        seen["last"] = msg
        if a.every > 0.0 and time.time() - last_print["t"] >= a.every:
            last_print["t"] = time.time()
            print(f"   t+{time.time()-start:5.1f}s  {describe(msg)}", flush=True)

    start = time.time()
    if a.info:
        msg_type = pick_type(a.topic)
        type_name = f"{msg_type.__module__.rsplit('.', 1)[0]}/msg/{msg_type.__name__}"
        for kind, infos in (("publishers", node.get_publishers_info_by_topic(a.topic)),
                            ("subscribers", node.get_subscribers_info_by_topic(a.topic))):
            print(f"{a.topic} {kind}: {len(infos)}")
            for i in infos:
                print(f"   - {i.node_name} ({i.node_namespace})")

    node.create_subscription(pick_type(a.topic), a.topic, cb, qos)
    deadline = time.time() + a.seconds
    while rclpy.ok() and time.time() < deadline and seen["n"] == 0:
        rclpy.spin_once(node, timeout_sec=0.2)
    while rclpy.ok() and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    if seen["n"] == 0:
        print(f"{a.topic}: NO MESSAGES in {a.seconds:.0f} s")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    print(f"{a.topic}: {describe(seen['last'])}")
    if a.count:
        print(f"   received {seen['n']} messages in {a.seconds:.0f} s "
              f"(~{seen['n']/a.seconds:.1f} Hz)")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
