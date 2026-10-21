#!/usr/bin/python3
"""Forward authorised commands, stopping on an arbiter failure or stale input."""
import copy
import math
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ChannelGate(Node):
    def __init__(self, **kwargs):
        super().__init__('channel_gate', **kwargs)
        robot = self.declare_parameter('robot_name', '').value
        if not robot:
            raise ValueError('robot_name required')
        self.timeout = float(self.declare_parameter('command_timeout_sec', .35).value)
        if not math.isfinite(self.timeout) or self.timeout <= 0:
            raise ValueError('command_timeout_sec must be positive')
        self.last_command = Twist()
        self.received = -float('inf')
        self.pub = self.create_publisher(Twist, f'/{robot}/cmd_vel_reserved', 1)
        self.create_subscription(Twist, f'/{robot}/cmd_vel_channel', self.on_command, 1)
        # Wall-time watchdog also expires while the ROS/simulation clock pauses.
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self.publish, clock=self.wall_clock)

    def on_command(self, command):
        self.last_command = copy.deepcopy(command)
        self.received = time.monotonic()
        self.publish()

    def publish(self):
        age = time.monotonic() - self.received
        self.pub.publish(self.last_command if 0 <= age <= self.timeout else Twist())


def main():
    rclpy.init()
    node = ChannelGate()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
