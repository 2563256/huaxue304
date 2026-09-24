#!/usr/bin/python3
"""Publish map -> <robot>/base_footprint straight from the bridged Gazebo poses.

NOT started by bridge.launch.py, and it must stay that way: the fleet's TF is
map -> <robot>/odom (static, from the spawn pose) -> <robot>/base_footprint
(from omni_odom_relay.py, ground truth). Publishing map -> base_footprint here
as well would give every robot two competing parents on the same frame.

It is kept as a standalone debugging aid for the case where the odometry relay
is not running (it needs a /<robot>/gz_pose topic, which only exists when the
bridge is up). The robot list comes from robot_profiles.yaml, so it follows the
fleet when robots are added or removed.
"""

import os

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class GzPoseTfRelay(Node):
    def __init__(self):
        super().__init__('gz_pose_tf_relay')
        self.br = TransformBroadcaster(self)
        share = get_package_share_directory('hetero_fleet_description')
        with open(os.path.join(share, 'config', 'robot_profiles.yaml')) as f:
            profiles = yaml.safe_load(f)
        self.robots = tuple(entry['name'] for entry in profiles['fleet'])
        for name in self.robots:
            self.create_subscription(
                PoseStamped, f'/{name}/gz_pose',
                lambda msg, n=name: self.cb(msg, n), 10)

    def cb(self, msg, name):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = f'{name}/base_footprint'
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(GzPoseTfRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
