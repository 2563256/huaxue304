#!/usr/bin/python3
"""Stamp the monitored Nav2 command without ever bypassing an emergency stop."""
import copy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist,TwistStamped


class CmdVelStamper(Node):
    def __init__(self):
        super().__init__('cmd_vel_stamper')
        robot=self.declare_parameter('robot_name','').value
        controller=self.declare_parameter('controller_name','').value
        if not robot or not controller:raise ValueError('robot_name and controller_name required')
        input_topic=self.declare_parameter('input_topic',f'/{robot}/cmd_vel').value
        output_topic=self.declare_parameter('output_topic',f'/{robot}/{controller}/cmd_vel').value
        self.mirror_y=bool(self.declare_parameter('mirror_y',False).value)
        self.angular_z_scale=float(self.declare_parameter('angular_z_scale',1.0).value)
        self.pub=self.create_publisher(TwistStamped,output_topic,10)
        self.create_subscription(Twist,input_topic,self.on_command,10)
        self.get_logger().info(f'stamping {input_topic} -> {output_topic}; stop commands remain authoritative')

    def on_command(self,command):
        out=TwistStamped();out.header.stamp=self.get_clock().now().to_msg()
        out.twist=copy.deepcopy(command)
        if self.mirror_y:out.twist.linear.y=-command.linear.y
        out.twist.angular.z=command.angular.z*self.angular_z_scale
        self.pub.publish(out)


def main():
    rclpy.init();node=CmdVelStamper()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
