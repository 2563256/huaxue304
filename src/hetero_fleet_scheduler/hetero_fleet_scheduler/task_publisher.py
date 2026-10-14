#!/usr/bin/env python3
"""
task_publisher - inject a task stream into a running fleet_manager.

Two ways to use it:

  1. Replay a scenario file, one task at a time:
         ros2 run hetero_fleet_scheduler task_publisher \
             --scenario config/tasks/demo_10.yaml --interval 8

  2. Send a single ad-hoc task from the command line:
         ros2 run hetero_fleet_scheduler task_publisher \
             --single --capability heavy_payload --payload 120 \
             --pickup -4.16 5.10 --dropoff 7.44 -1.15

   (Those coordinates are pickup_A and dropoff_C of the scanned map; the zone
   list is in this workspace's MAP_SWAP.md.)

Task ids are generated automatically for --single so repeated invocations do
not collide. Scenario tasks keep the ids written in the file, which makes the
logs line up with the YAML.
"""

from __future__ import annotations

import argparse
import random
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from hetero_fleet_msgs.msg import Task as TaskMsg

import os

# The capability vocabulary, mirrored from robot_profiles.yaml. Used only to
# sanity-check that a scenario's task can be served by *some* robot.
KNOWN_CAPABILITIES = {
    "humanoid_transport",
    "manual_handling",
    "light_payload",
    "heavy_payload",
    "narrow_aisle",
    "precise_docking",
    "omnidirectional",
    "",
}


class TaskPublisher(Node):
    def __init__(self):
        super().__init__("task_publisher")
        self.pub = self.create_publisher(TaskMsg, "/task_request", 10)

    def send(self, task: TaskMsg, delay: float = 0.5) -> None:
        """Publish a task, repeatedly, until the manager has really seen it.

        A single publish is not reliable here: on a loaded machine DDS
        discovery can take seconds, and a task that goes out before the
        subscription is matched is simply dropped - the robot then never moves
        and nothing anywhere reports an error. Waiting for the subscriber and
        then sending the same message a few times fixes that; fleet_manager
        de-duplicates by task id, so the repeats are harmless.
        """
        deadline = time.time() + 10.0
        while self.pub.get_subscription_count() == 0 and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        for _ in range(4):
            self.pub.publish(task)
            time.sleep(0.4)
        time.sleep(delay)


def build_msg(spec: dict) -> TaskMsg:
    m = TaskMsg()
    m.id = str(spec["id"])
    m.type = str(spec.get("type", "transport"))
    p = spec["pickup"]
    d = spec["dropoff"]
    m.pickup.x, m.pickup.y = float(p[0]), float(p[1])
    m.pickup.theta = float(p[2]) if len(p) > 2 else 0.0
    m.dropoff.x, m.dropoff.y = float(d[0]), float(d[1])
    m.dropoff.theta = float(d[2]) if len(d) > 2 else 0.0
    m.payload_kg = float(spec.get("payload_kg", 0.0))
    m.required_capability = str(spec.get("required_capability", ""))
    m.priority = int(spec.get("priority", 1))
    m.deadline_sec = float(spec.get("deadline_sec", 0.0))
    return m


def load_scenario(node: Node, path: str) -> list:
    # Allow a bare filename to resolve against the installed config/tasks dir.
    if not os.path.exists(path):
        try:
            cand = os.path.join(
                get_package_share_directory("hetero_fleet_scheduler"),
                "config", "tasks", os.path.basename(path),
            )
            if os.path.exists(cand):
                path = cand
        except Exception:  # noqa: BLE001 - package index may be unavailable
            pass

    with open(path) as f:
        raw = yaml.safe_load(f)

    tasks = raw.get("tasks", [])
    node.get_logger().info(f"loaded {len(tasks)} tasks from {path}")

    for t in tasks:
        cap = str(t.get("required_capability", ""))
        if cap not in KNOWN_CAPABILITIES:
            node.get_logger().warn(
                f"task {t.get('id')}: unknown capability '{cap}' - no robot "
                f"will be able to serve it"
            )
    return tasks


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", help="YAML scenario file to replay")
    parser.add_argument("--interval", type=float, default=8.0,
                        help="seconds between scenario tasks (default 8)")
    parser.add_argument("--single", action="store_true",
                        help="send one ad-hoc task and exit")
    parser.add_argument("--id", default=None, help="task id for --single")
    parser.add_argument("--payload", type=float, default=0.0)
    parser.add_argument("--capability", default="")
    parser.add_argument("--priority", type=int, default=1)
    parser.add_argument("--deadline", type=float, default=0.0)
    parser.add_argument("--pickup", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--dropoff", nargs=2, type=float, metavar=("X", "Y"))
    parser.add_argument("--shuffle", action="store_true",
                        help="shuffle scenario order before replaying")
    args = parser.parse_args(argv)

    if not args.scenario and not args.single:
        parser.error("give either --scenario or --single")

    rclpy.init()
    node = TaskPublisher()

    try:
        if args.single:
            if not args.pickup or not args.dropoff:
                parser.error("--single needs --pickup X Y and --dropoff X Y")
            tid = args.id or f"adhoc_{int(time.time()) % 100000}"
            spec = {
                "id": tid,
                "type": "transport",
                "pickup": args.pickup,
                "dropoff": args.dropoff,
                "payload_kg": args.payload,
                "required_capability": args.capability,
                "priority": args.priority,
                "deadline_sec": args.deadline,
            }
            node.send(build_msg(spec))
            node.get_logger().info(
                f"sent {tid}: payload={args.payload}kg cap={args.capability or '-'}"
            )
            time.sleep(0.5)
            return 0

        specs = load_scenario(node, args.scenario)
        if args.shuffle:
            random.shuffle(specs)

        for i, spec in enumerate(specs, 1):
            node.send(build_msg(spec))
            node.get_logger().info(
                f"[{i}/{len(specs)}] sent {spec['id']} "
                f"payload={spec.get('payload_kg', 0)}kg "
                f"cap={spec.get('required_capability', '-') or '-'} "
                f"prio={spec.get('priority', 1)}"
            )
            if i < len(specs):
                time.sleep(args.interval)

        node.get_logger().info("scenario exhausted; waiting briefly before exit")
        time.sleep(2.0)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
