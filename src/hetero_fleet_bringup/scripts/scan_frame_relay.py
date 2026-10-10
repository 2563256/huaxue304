#!/usr/bin/python3
"""Correct lidar frames and publish exact teammate outlines from ground truth.

Teammate scans have their own 10 Hz timer and use base_footprint, independent
of lidar height/offset and of whether Gazebo's physical scan has arrived.
Local costmaps see every body. A separate global-planning scan includes only
vehicles stationary for 10 seconds, avoiding route flips around passing traffic.
"""
import copy
import math
import os
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def rectangle_returns(angles, center_x, center_y, yaw, length, width,
                      range_min=0.01, range_max=12.0):
    """Intersect rays at (0,0) with a rotated rectangle, using slab intervals.

    Exact near-face distances retain bumper/corner coverage and work across
    the -pi/+pi seam. Inf means clear sky and is used to clear old positions.
    """
    angles=np.asarray(angles,dtype=float)
    c,s=math.cos(yaw),math.sin(yaw)
    origin=np.array([-c*center_x-s*center_y, s*center_x-c*center_y])
    dirs=np.stack([np.cos(angles-yaw),np.sin(angles-yaw)],axis=1)
    half=np.array([length/2.,width/2.])
    lo=np.full(len(angles),-np.inf);hi=np.full(len(angles),np.inf)
    for axis in range(2):
        d=dirs[:,axis];parallel=np.abs(d)<1e-10
        denom=np.where(parallel,1.,d)
        a=(-half[axis]-origin[axis])/denom
        b=(half[axis]-origin[axis])/denom
        near=np.where(parallel,-np.inf,np.minimum(a,b))
        far=np.where(parallel,np.inf,np.maximum(a,b))
        if abs(origin[axis])>half[axis]:
            far=np.where(parallel,-np.inf,far)
        lo=np.maximum(lo,near);hi=np.minimum(hi,far)
    hit=(hi>=np.maximum(lo,0.))&(lo<=range_max)
    return np.where(hit,np.clip(lo,range_min,range_max),np.inf)


def update_stationary(state, pose, now, linear_speed=0., angular_speed=0.):
    """Track net motion in ROS time; moving or turning vehicles leave at once."""
    anchor = state.get('stationary_anchor')
    since = state.get('stationary_since', now)
    moved = anchor is None or math.hypot(pose[0]-anchor[0], pose[1]-anchor[1]) > .04
    turned = anchor is not None and abs(math.atan2(
        math.sin(pose[2]-anchor[2]), math.cos(pose[2]-anchor[2]))) > .15
    if moved or turned or linear_speed > .03 or abs(angular_speed) > .05 or now < since:
        state['stationary_anchor'] = pose
        state['stationary_since'] = now
    elif 'stationary_since' not in state:
        state['stationary_since'] = now



class ScanFrameRelay(Node):
    def __init__(self):
        super().__init__('scan_frame_relay')
        share=get_package_share_directory('hetero_fleet_description')
        with open(os.path.join(share,'config','robot_profiles.yaml')) as f:
            profiles=yaml.safe_load(f)
        self.robots={}
        for entry in profiles['fleet']:
            typ=profiles['robot_types'][entry['type']]
            self.robots[entry['name']]={
                'spawn':tuple(map(float,entry['spawn'])),
                'footprint':tuple(map(float,typ['footprint'])),
                'pose':None,
            }
        self.max_range=float(self.declare_parameter('max_range',12.0).value)
        self.samples=int(self.declare_parameter('samples',1080).value)
        self.rate=float(self.declare_parameter('publish_rate_hz',10.0).value)
        self.padding=float(self.declare_parameter('teammate_padding',0.12).value)
        self.stationary_sec=float(self.declare_parameter('stationary_planning_sec',10.0).value)
        self.angles=np.arange(self.samples)*2*math.pi/self.samples-math.pi
        self.pubs={};self.obstacle_pubs={};self.contact_pubs={};self.planning_pubs={}
        for name in self.robots:
            self.pubs[name]=self.create_publisher(LaserScan,f'/{name}/scan_fixed',10)
            self.obstacle_pubs[name]=self.create_publisher(LaserScan,f'/{name}/robot_obstacles',10)
            self.contact_pubs[name]=self.create_publisher(LaserScan,f'/{name}/robot_contact',10)
            self.planning_pubs[name]=self.create_publisher(LaserScan,f'/{name}/robot_planning_obstacles',10)
            self.create_subscription(LaserScan,f'/{name}/scan',lambda m,n=name:self.on_scan(m,n),qos_profile_sensor_data)
            self.create_subscription(Odometry,f'/{name}/odom',lambda m,n=name:self.on_odom(n,m),qos_profile_sensor_data)
        self.create_timer(1./self.rate,self.publish_robot_obstacles)

    def on_odom(self,name,msg):
        sx,sy,syaw=self.robots[name]['spawn']
        dx,dy=msg.pose.pose.position.x,msg.pose.pose.position.y
        c,s=math.cos(syaw),math.sin(syaw);q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        pose=(sx+c*dx-s*dy,sy+s*dx+c*dy,yaw+syaw)
        state=self.robots[name];state['pose']=pose
        twist=msg.twist.twist
        update_stationary(state,pose,self.get_clock().now().nanoseconds/1e9,
                         math.hypot(twist.linear.x,twist.linear.y),twist.angular.z)

    def on_scan(self,msg,name):
        msg.header.frame_id=f'{name}/lidar_link'
        self.pubs[name].publish(msg)

    def publish_robot_obstacles(self):
        clock=self.get_clock().now();stamp=clock.to_msg();now=clock.nanoseconds/1e9
        for name,ego in self.robots.items():
            if ego['pose'] is None:
                continue
            ex,ey,eyaw=ego['pose'];c,s=math.cos(eyaw),math.sin(eyaw)
            ranges=np.full(self.samples,np.inf);contact_ranges=np.full(self.samples,np.inf)
            planning_ranges=np.full(self.samples,np.inf)
            for other,state in self.robots.items():
                if other==name or state['pose'] is None:continue
                ox,oy,oyaw=state['pose'];dx,dy=ox-ex,oy-ey
                bx,by=c*dx+s*dy,-s*dx+c*dy
                length,width=state['footprint']
                returns=rectangle_returns(self.angles,bx,by,oyaw-eyaw,
                    length+2*self.padding,width+2*self.padding,.01,self.max_range)
                ranges=np.minimum(ranges,returns)
                if now-state.get("stationary_since",now)>=self.stationary_sec:
                    planning_ranges=np.minimum(planning_ranges,returns)
                contact_ranges=np.minimum(contact_ranges,rectangle_returns(
                    self.angles,bx,by,oyaw-eyaw,length,width,.01,self.max_range))
            out=LaserScan();out.header.stamp=stamp
            out.header.frame_id=f'{name}/base_footprint'
            out.angle_min=-math.pi;out.angle_increment=2*math.pi/self.samples
            out.angle_max=float(self.angles[-1]);out.scan_time=1./self.rate
            out.range_min=.01;out.range_max=self.max_range
            out.ranges=ranges.tolist()
            self.obstacle_pubs[name].publish(out)
            contact=copy.deepcopy(out);contact.ranges=contact_ranges.tolist()
            self.contact_pubs[name].publish(contact)
            planning=copy.deepcopy(out);planning.ranges=planning_ranges.tolist()
            self.planning_pubs[name].publish(planning)


def main():
    rclpy.init();node=ScanFrameRelay()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
