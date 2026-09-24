#!/usr/bin/python3
"""Make sure every robot's whole Nav2 stack actually reaches ACTIVE.

The failure this exists for, seen twice now on this machine:

    [omni_1.lifecycle_manager_navigation]: Activating map_server
    [ERROR] Failed to change state for node: map_server.
            Exception: map_server/get_state service client: async_send_request
            failed.
    [ERROR] Failed to bring up all requested nodes. Aborting bringup.

A single lifecycle service call getting dropped by DDS makes the manager give
up on that robot *entirely* - all nine of its nodes are left `inactive`. The
stack still looks alive (processes running, topics present), but bt_navigator
answers every goal with

    [omni_1.bt_navigator]: Action server is inactive. Rejecting the goal.

so fleet_manager sees "navigation to pickup failed", burns the task's single
requeue, and marks it FAILED. In the run that prompted this, all four tasks
omni_1 was eligible for died that way while the other three robots worked
normally.

This node runs once, after every stack should be up, and puts each robot's
nodes through the same configure-then-activate sequence nav2.launch.py uses -
only for the nodes that are actually stuck. It is a no-op when the lifecycle
managers did their job, and it exits either way.
"""

import os
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node

# Same membership and order as LIFECYCLE_NODES in nav2.launch.py: costmap
# sources and controllers before the nodes that consume them.
STACK = (
    "map_server",
    "amcl",
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "velocity_smoother",
    "collision_monitor",
)


class StackWatchdog(Node):
    def __init__(self):
        super().__init__("nav2_stack_watchdog")
        share = get_package_share_directory("hetero_fleet_description")
        with open(os.path.join(share, "config", "robot_profiles.yaml")) as f:
            profiles = yaml.safe_load(f)
        self.robots = [entry["name"] for entry in profiles["fleet"]]
        self.window = float(self.declare_parameter("window_sec", 120.0).value)
        # NOT self._clients: rclpy's Node keeps its own list under that name and
        # overwriting it makes create_client blow up with
        # "'dict' object has no attribute 'append'".
        self._stack_clients = {}

    # ----------------------------------------------------------- lifecycle
    def _client(self, ns, node, srv_type, verb):
        key = (ns, node, verb)
        if key not in self._stack_clients:
            self._stack_clients[key] = self.create_client(
                srv_type, f"{ns}/{node}/{verb}"
            )
        return self._stack_clients[key]

    def state(self, ns, node):
        client = self._client(ns, node, GetState, "get_state")
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        if not future.done() or future.result() is None:
            return None
        return future.result().current_state.id

    def change(self, ns, node, transition, label):
        client = self._client(ns, node, ChangeState, "change_state")
        if not client.wait_for_service(timeout_sec=5.0):
            return False
        request = ChangeState.Request()
        request.transition.id = transition
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=8.0)
        ok = future.done() and future.result() is not None and future.result().success
        if ok:
            self.get_logger().info(f"{ns}/{node}: {label} ok")
        return ok

    # --------------------------------------------------------------- repair
    def prepare_clients(self):
        """Create every client before asking any of them anything.

        A client created and immediately queried almost always times out: DDS
        still has to discover the service. With ~35 nodes in the fleet that is
        not a marginal effect - an earlier version of this script created
        clients lazily and reported every single service as missing.
        """
        for name in self.robots:
            ns = f"/{name}"
            for node in STACK:
                self._client(ns, node, GetState, "get_state")
                self._client(ns, node, ChangeState, "change_state")

    def repair(self, name):
        ns = f"/{name}"
        states = {node: self.state(ns, node) for node in STACK}
        if all(value == State.PRIMARY_STATE_ACTIVE for value in states.values()):
            return True
        stuck = ", ".join(
            f"{node}={'-' if value is None else value}" for node, value in states.items()
            if value != State.PRIMARY_STATE_ACTIVE
        )
        self.get_logger().warn(f"{name}: stack not fully active ({stuck})")
        # Mirror the lifecycle manager: configure everything first, then
        # activate, so no node is activated before its dependencies exist.
        for node, value in states.items():
            if value == State.PRIMARY_STATE_UNCONFIGURED:
                self.change(ns, node, Transition.TRANSITION_CONFIGURE, "configure")
        for node in STACK:
            if self.state(ns, node) == State.PRIMARY_STATE_INACTIVE:
                self.change(ns, node, Transition.TRANSITION_ACTIVATE, "activate")
        final = {node: self.state(ns, node) for node in STACK}
        return all(value == State.PRIMARY_STATE_ACTIVE for value in final.values())

    def run(self):
        # Give the lifecycle managers a moment to finish on their own before
        # stepping in - otherwise this races them for the same transitions.
        settle = float(self.declare_parameter("settle_sec", 10.0).value)
        time.sleep(min(settle, 30.0))
        self.prepare_clients()
        deadline = time.time() + self.window
        pending = list(self.robots)
        while rclpy.ok() and pending and time.time() < deadline:
            still = []
            for name in pending:
                if not self.repair(name):
                    still.append(name)
            pending = still
            if pending:
                self.get_logger().warn(
                    f"still not active: {', '.join(pending)} - retrying"
                )
                time.sleep(5.0)
        if pending:
            self.get_logger().error(
                f"gave up on {', '.join(pending)}; check the lifecycle_manager "
                f"output for those namespaces"
            )
        else:
            self.get_logger().info("all robot stacks are active")


def main():
    rclpy.init()
    node = StackWatchdog()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
