#!/usr/bin/python3
"""Make sure the visualisation /map_server actually ends up ACTIVE.

Why this exists: on this machine the lifecycle service response occasionally
gets lost in the DDS layer. The symptom is unmistakable in the map server's own
log - the map loads fine, then

    [map_server.rclcpp]: failed to send response to
    /map_server/change_state (timeout): client will not receive response

so lifecycle_manager_map never learns that configure succeeded and waits
forever, leaving the server `inactive`. A map server only publishes while
ACTIVE, so RViz's Map display shows an empty grid even though
maps/geomap.yaml is perfectly good - exactly the "map did not load" report.

This node runs once, shortly after the map server is supposed to be up, checks
its state and nudges it through configure -> activate if the manager got stuck.
It is a no-op when the lifecycle manager did its job, and it exits either way so
it never lingers in the graph.
"""

import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class MapServerWatchdog(Node):
    def __init__(self):
        super().__init__("map_server_watchdog")
        self.get_state = self.create_client(GetState, "/map_server/get_state")
        self.change = self.create_client(ChangeState, "/map_server/change_state")

    def state(self):
        if not self.get_state.wait_for_service(timeout_sec=5.0):
            return None
        future = self.get_state.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().current_state.id if future.done() and future.result() else None

    def transition(self, transition_id, label):
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.change.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        ok = future.done() and future.result() is not None and future.result().success
        self.get_logger().info(f"{label}: {'ok' if ok else 'no response'}")
        return ok

    def run(self):
        # Give the lifecycle manager time to do its own job first.
        deadline = time.time() + 45.0
        while rclpy.ok() and time.time() < deadline:
            current = self.state()
            if current is None:
                time.sleep(1.0)
                continue
            if current == State.PRIMARY_STATE_ACTIVE:
                return
            if current == State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().warn(
                    "map_server is still unconfigured (the lifecycle manager's "
                    "configure response was likely lost) - configuring it directly"
                )
                if not self.transition(Transition.TRANSITION_CONFIGURE, "configure"):
                    time.sleep(2.0)
                    continue
                continue
            if current == State.PRIMARY_STATE_INACTIVE:
                self.get_logger().warn(
                    "map_server is configured but never activated (the lifecycle "
                    "manager's activate response was likely lost) - activating it "
                    "directly so /map is published"
                )
                if self.transition(Transition.TRANSITION_ACTIVATE, "activate"):
                    return
                time.sleep(2.0)
                continue
            time.sleep(2.0)


def main():
    rclpy.init()
    node = MapServerWatchdog()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
