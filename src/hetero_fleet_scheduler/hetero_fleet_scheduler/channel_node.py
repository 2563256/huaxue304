"""Central corridor arbiter and command filter before Nav2 collision monitors."""
import copy
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from ament_index_python.packages import get_package_share_directory
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from .reservations import ChannelReservations


def odom_pose(message, spawn):
    """Same map anchor as FleetManager and the ground-truth odometry relay."""
    p, q = message.pose.pose.position, message.pose.pose.orientation
    c, s = math.cos(spawn[2]), math.sin(spawn[2])
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return spawn[0] + c * p.x - s * p.y, spawn[1] + s * p.x + c * p.y, spawn[2] + yaw


class ChannelReservationNode(Node):
    def __init__(self, **kwargs):
        super().__init__('channel_reservations', **kwargs)
        enabled = self.declare_parameter('enabled', True).value
        profiles_path = self.declare_parameter('profiles_file', str(
            Path(get_package_share_directory('hetero_fleet_description')) / 'config/robot_profiles.yaml')).value
        channels_path = self.declare_parameter('channels_file', str(
            Path(get_package_share_directory('hetero_fleet_scheduler')) / 'config/channels.yaml')).value
        profiles = yaml.safe_load(Path(profiles_path).read_text())
        config = yaml.safe_load(Path(channels_path).read_text())
        self.spawns = {r['name']: tuple(r['spawn']) for r in profiles['fleet']}
        # Schedule using actual heterogeneous speed limits, not one fleet speed.
        config['robot_motion_limits'] = {
            r['name']: (profiles['robot_types'][r['type']]['max_speed'],
                        profiles['robot_types'][r['type']]['max_omega'])
            for r in profiles['fleet']}
        self.core = ChannelReservations(config, {
            r['name']: (*profiles['robot_types'][r['type']]['footprint'],
                        profiles['robot_types'][r['type']]['footprint_padding'])
            for r in profiles['fleet']}, enabled=enabled)
        self.create_subscription(String, '/fleet_schedule', self._on_schedule, 1)
        self.raw, self.intents, self.action_active, self.task_holds = {}, {}, {}, {}
        self.transport_intents = {}
        self.outputs, self.wait_outputs = {}, {}
        durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        odom_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for name in self.spawns:
            self.create_subscription(Bool, f'/{name}/reservation_transport_active',
                                     lambda msg, n=name: self.transport_intents.update(
                                         {n: (msg.data, time.monotonic())}), durable)
            self.outputs[name] = self.create_publisher(Twist, f'/{name}/cmd_vel_channel', 1)
            self.wait_outputs[name] = self.create_publisher(Bool, f'/{name}/channel_wait', 1)
            self.create_subscription(Odometry, f'/{name}/odom',
                                     lambda msg, n=name: self._on_odom(n, msg), odom_qos)
            self.create_subscription(NavPath, f'/{name}/plan',
                                     lambda msg, n=name: self._on_plan(n, msg), 1)
            self.create_subscription(Bool, f'/{name}/reservation_active',
                                     lambda msg, n=name: self._on_intent(n, msg), durable)
            self.create_subscription(Bool, f'/{name}/task_hold',
                                     lambda msg, n=name: self._on_task_hold(n, msg), durable)
            self.create_subscription(GoalStatusArray, f'/{name}/navigate_to_pose/_action/status',
                                     lambda msg, n=name: self._on_status(n, msg), durable)
            self.create_subscription(Twist, f'/{name}/cmd_vel_smoothed',
                                     lambda msg, n=name: self._on_command(n, msg), 1)
        self.status = self.create_publisher(String, '/channel_reservations', durable)
        self.markers = self.create_publisher(MarkerArray, '/channel_markers', durable)
        self.last_status = -math.inf
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05, self._update, clock=self.wall_clock)
        self.get_logger().info(f'Corridor reservations: {list(self.core.regions)}, enabled={enabled}')

    def _on_schedule(self, message):
        try:
            self.core.set_schedule_hint(json.loads(message.data), time.monotonic())
        except (ValueError, TypeError, KeyError) as exc:
            self.get_logger().warn(f'Ignoring invalid task schedule: {exc}')

    def _on_intent(self, name, message):
        self.intents[name] = message.data, time.monotonic()

    def _on_status(self, name, message):
        self.action_active[name] = any(s.status in (1, 2) for s in message.status_list)

    def _on_task_hold(self, name, message):
        self.task_holds[name] = message.data
        # Never replay a pre-service command when the hold ends.
        self.raw.pop(name, None)
        self.core.commands.pop(name, None)
        self._update()

    def _on_odom(self, name, message):
        try:
            self.core.observe(name, odom_pose(message, self.spawns[name]), time.monotonic())
        except ValueError:
            self.core.poses.pop(name, None)
            self.get_logger().warn(f'{name}: invalid odometry; holding channel admission')
            self._update()

    def _on_plan(self, name, message):
        if message.header.frame_id != 'map':
            self.core.set_path(name, [])
            self.get_logger().warn(f'{name}: reservation path must be in map frame')
            return
        poses = []
        for p in message.poses:
            q = p.pose.orientation
            poses.append((p.pose.position.x, p.pose.position.y,
                          math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))
        try:
            self.core.set_path(name, poses)
        except ValueError:
            self.core.set_path(name, [])
            self.get_logger().warn(f'{name}: invalid reservation path; holding commands')

    def _on_command(self, name, message):
        now = time.monotonic()
        try:
            self.core.set_command(name, (message.linear.x, message.linear.y, message.angular.z), now)
            self.raw[name] = copy.deepcopy(message), now
        except ValueError:
            self.raw.pop(name, None)
            self.core.commands.pop(name, None)
        # The 20 Hz wall timer checks the latest exact command before any
        # forwarding, including BackUp. Recomputing/publishing the entire fleet
        # on every input multiplied the output rate by the fleet size and
        # starved costmap/TF callbacks under Gazebo load.

    def _update(self):
        wall_now = time.monotonic()
        for name in self.spawns:
            transport, transport_stamp = self.transport_intents.get(name, (False, -math.inf))
            self.core.set_transport_hold(name, transport and 0 <= wall_now - transport_stamp <= 2.)
            intent, stamp = self.intents.get(name, (False, -math.inf))
            action_live = (self.action_active.get(name, False) and
                           self.count_publishers(f'/{name}/navigate_to_pose/_action/status') > 0)
            active = (intent and 0 <= wall_now - stamp <= 2.) or action_live
            if active != self.core.active[name]:
                self.core.set_active(name, active)
        decisions, snapshot = self.core.step(self.get_clock().now().nanoseconds / 1e9, wall_now)
        for name, decision in decisions.items():
            if self.task_holds.get(name, False):
                decision.update(allowed=False, waiting=False, reason='task_service')
            message, stamp = self.raw.get(name, (Twist(), -math.inf))
            safe = decision['allowed'] and 0 <= wall_now - stamp <= self.core.command_timeout
            self.outputs[name].publish(message if safe else Twist())
            self.wait_outputs[name].publish(Bool(data=decision['waiting']))
        if wall_now - self.last_status >= .5:
            self.status.publish(String(data=json.dumps(snapshot, separators=(',', ':'))))
            self.markers.publish(self._markers(snapshot))
            self.last_status = wall_now


    def _markers(self, snapshot):
        result = MarkerArray()
        clear = Marker(); clear.action = Marker.DELETEALL; result.markers.append(clear)
        for index, channel in enumerate(snapshot['channels']):
            group = snapshot['groups'][channel['group']]
            x0, y0, x1, y1 = channel['bounds']
            box = Marker(); box.header.frame_id = 'map'; box.header.stamp = self.get_clock().now().to_msg()
            box.ns = 'channel_reservations'; box.id = index * 2; box.type = Marker.CUBE
            box.pose.orientation.w = 1.; box.pose.position.x = (x0 + x1) / 2
            box.pose.position.y = (y0 + y1) / 2; box.pose.position.z = .025
            box.scale.x = float(x1 - x0); box.scale.y = float(y1 - y0); box.scale.z = .02
            color = (.2, .8, .3) if group['owner'] is None else (.2, .5, 1.)
            if group['queue']: color = (1., .6, .1)
            if not snapshot['pose_ready'] or not snapshot['enabled']: color = (.5, .5, .5)
            if len(group['occupants']) > 1: color = (1., .1, .1)
            box.color.r, box.color.g, box.color.b = color; box.color.a = .14
            result.markers.append(box)
            label = copy.deepcopy(box); label.id += 1; label.type = Marker.TEXT_VIEW_FACING
            label.pose.position.z = .25; label.scale.x = label.scale.y = 0.; label.scale.z = .22
            label.color.r = label.color.g = label.color.b = label.color.a = 1.
            label.text = (f"{channel['id']} ({snapshot['scheduler']['mode']})\nOwner: {group['owner'] or 'free'}"
                          f"\nQueue: {', '.join(group['queue']) or '-'}")
            result.markers.append(label)
        return result


def main():
    rclpy.init()
    node = ChannelReservationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
