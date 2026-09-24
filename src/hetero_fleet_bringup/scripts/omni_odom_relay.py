#!/usr/bin/python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage


class OmniOdomRelay(Node):
    """Publish one robot's odometry from the Gazebo ground-truth model pose.

    Started once per robot by bridge.launch.py (the file keeps its original
    name because it began as the omni-only workaround). It publishes
    /<robot_name>/odom and the odom -> base_footprint transform, which is what
    keeps Nav2's laser projection consistent: the drive controllers run
    open_loop and their odometry drifts as soon as a robot is blocked, which
    paints phantom obstacles into the costmap until no path can be planned.
    """

    def __init__(self):
        super().__init__('omni_odom_relay')
        self.robot_name = self.declare_parameter('robot_name', 'omni_1').value
        self.spawn_x = self.declare_parameter('spawn_x', 13.0).value
        self.spawn_y = self.declare_parameter('spawn_y', 2.0).value
        self.spawn_yaw = self.declare_parameter('spawn_yaw', 0.0).value
        # Gazebo publishes the pose array far faster than Nav2 needs it. Doing
        # Python math and publishing odom + TF on every message costs ~25% of a
        # core per robot, which is enough to push this machine into thrashing
        # and make controller spawners time out - i.e. robots end up with no
        # active controller and stop taking tasks. 40 Hz is plenty for the
        # costmaps (0.5 s tolerance) and the 20 Hz controller.
        rate = float(self.declare_parameter('publish_rate_hz', 40.0).value)
        self.min_period = 0.0 if rate <= 0.0 else 1.0 / rate
        self.last_publish = None
        self.previous = None
        self.pose_index = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(
            Odometry, f'/{self.robot_name}/odom', 10)
        self.create_subscription(
            TFMessage, '/gz/dynamic_pose', self.pose_callback, 10)

    @staticmethod
    def yaw_from_quaternion(q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def quaternion_from_yaw(yaw):
        half = yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def pose_callback(self, msg):
        if not msg.transforms:
            return
        now = self.get_clock().now()
        if self.min_period > 0.0 and self.last_publish is not None:
            elapsed = (now - self.last_publish).nanoseconds * 1e-9
            if elapsed < self.min_period:
                return
        self.last_publish = now
        if self.pose_index is None:
            self.pose_index = min(
                range(len(msg.transforms)),
                key=lambda i: (
                    msg.transforms[i].transform.translation.x - self.spawn_x
                ) ** 2 + (
                    msg.transforms[i].transform.translation.y - self.spawn_y
                ) ** 2,
            )
        if self.pose_index >= len(msg.transforms):
            self.pose_index = None
            return

        pose = msg.transforms[self.pose_index].transform
        dx = pose.translation.x - self.spawn_x
        dy = pose.translation.y - self.spawn_y
        c0 = math.cos(self.spawn_yaw)
        s0 = math.sin(self.spawn_yaw)
        x = c0 * dx + s0 * dy
        y = -s0 * dx + c0 * dy
        yaw = self.yaw_from_quaternion(pose.rotation) - self.spawn_yaw
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        stamp = self.get_clock().now().to_msg()
        stamp_seconds = stamp.sec + stamp.nanosec * 1e-9
        vx = vy = wz = 0.0
        if self.previous is not None:
            previous_stamp, previous_x, previous_y, previous_yaw = self.previous
            dt = stamp_seconds - previous_stamp
            if dt > 0.0:
                world_vx = (x - previous_x) / dt
                world_vy = (y - previous_y) / dt
                cy = math.cos(yaw)
                sy = math.sin(yaw)
                vx = cy * world_vx + sy * world_vy
                vy = -sy * world_vx + cy * world_vy
                yaw_delta = math.atan2(
                    math.sin(yaw - previous_yaw), math.cos(yaw - previous_yaw))
                wz = yaw_delta / dt
        self.previous = (stamp_seconds, x, y, yaw)

        qx, qy, qz, qw = self.quaternion_from_yaw(yaw)
        odom_frame = f'{self.robot_name}/odom'
        base_frame = f'{self.robot_name}/base_footprint'

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = odom_frame
        odom.child_frame_id = base_frame
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = base_frame
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    rclpy.spin(OmniOdomRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
