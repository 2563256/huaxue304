"""Exercise real ROS topics, all fleet poses and watchdogs without Gazebo."""
import importlib.util
import math
from pathlib import Path
import time
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path as NavPath
from std_msgs.msg import Bool
from visualization_msgs.msg import MarkerArray, Marker
import yaml
from hetero_fleet_scheduler.channel_node import ChannelReservationNode

ROOT = Path(__file__).resolve().parents[3]
spec = importlib.util.spec_from_file_location('reservation_gate', ROOT / 'src/hetero_fleet_bringup/scripts/channel_gate.py')
gate_module = importlib.util.module_from_spec(spec); spec.loader.exec_module(gate_module)


@pytest.mark.parametrize('mode', ['fifo', 'sipp_pbs'])
def test_real_command_chain_holds_queue_and_fails_closed_on_arbiter_loss(tmp_path, mode):
    owned = not rclpy.ok()
    if owned: rclpy.init()
    config = tmp_path / 'channels.yaml'
    config.write_text(yaml.safe_dump({'scheduling': mode, 'reservation_exempt_robots': ['g1_1'], 'channels': [{'id': 'test_aisle', 'bounds': [0, -.6, 2, .6]}]}))
    arbiter = ChannelReservationNode(parameter_overrides=[Parameter('channels_file', value=str(config)),
                                                        Parameter('use_sim_time', value=True)])
    gates = [gate_module.ChannelGate(namespace=name, parameter_overrides=[
        Parameter('robot_name', value=name), Parameter('use_sim_time', value=True)]) for name in ['light_1', 'heavy_1', 'g1_1']]
    peer = Node('channel_test_peer')
    executor = SingleThreadedExecutor()
    for node in [arbiter, *gates, peer]: executor.add_node(node)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
    outputs = {}; odoms = {}; commands = {}; intentions = {}; plans = {}
    poses = {'light_1': (-2., 0., 0.), 'heavy_1': (4., 0., math.pi),
             'omni_1': (7., 4., 0.), 'g1_1': (1., 0., 0.)}
    for name in poses:
        odoms[name] = peer.create_publisher(Odometry, f'/{name}/odom', 1)
        commands[name] = peer.create_publisher(Twist, f'/{name}/cmd_vel_smoothed', 1)
        intentions[name] = peer.create_publisher(Bool, f'/{name}/reservation_active', durable)
        plans[name] = peer.create_publisher(NavPath, f'/{name}/plan', 1)
        if name in ['light_1', 'heavy_1', 'g1_1']:
            peer.create_subscription(Twist, f'/{name}/cmd_vel_reserved', lambda m, n=name: outputs.__setitem__(n, m), 1)
    rendered = []
    peer.create_subscription(MarkerArray, '/channel_markers', rendered.append, durable)
    destroyed = False
    active_names = {"light_1", "heavy_1", "g1_1"}
    def pump(duration, skip_pose=(), publish_commands=True):
        limit = time.monotonic() + duration
        while time.monotonic() < limit:
            for name, p in poses.items():
                if name not in skip_pose:
                    spawn = arbiter.spawns[name]
                    odom = Odometry()
                    dx, dy = p[0] - spawn[0], p[1] - spawn[1]
                    c, s = math.cos(spawn[2]), math.sin(spawn[2])
                    odom.pose.pose.position.x = c * dx + s * dy
                    odom.pose.pose.position.y = -s * dx + c * dy
                    odom.pose.pose.orientation.z = math.sin((p[2] - spawn[2]) / 2)
                    odom.pose.pose.orientation.w = math.cos((p[2] - spawn[2]) / 2)
                    odoms[name].publish(odom)
                intentions[name].publish(Bool(data=name in active_names))
                if publish_commands:
                    command = Twist(); command.linear.x = .20
                    commands[name].publish(command)
            for _ in range(12): executor.spin_once(timeout_sec=.002)
    try:
        pump(.6)
        assert rendered and len(rendered[-1].markers) == 3
        assert rendered[-1].markers[1].type == Marker.CUBE
        assert rendered[-1].markers[1].scale.x == 2.
        for name, goal in [('g1_1', (4., 0.)), ('light_1', (4., 0.)), ('heavy_1', (-2., 0.))]:
            path = NavPath(); path.header.frame_id = 'map'
            p = poses[name]
            for k in range(121):
                pose = PoseStamped(); pose.pose.position.x = p[0] + (goal[0] - p[0]) * k / 120
                pose.pose.position.y = 0.
                pose.pose.orientation.z = math.sin(p[2] / 2); pose.pose.orientation.w = math.cos(p[2] / 2)
                path.poses.append(pose)
            plans[name].publish(path)
            pump(.15)
        pump(.35)
        assert outputs['light_1'].linear.x == pytest.approx(.2)
        assert outputs['heavy_1'] == Twist()
        assert arbiter.core.tokens['test_aisle'].robot == 'light_1'
        assert outputs['g1_1'].linear.x == pytest.approx(.2)
        assert 'g1_1' not in arbiter.core.occupants['test_aisle']
        hold = peer.create_publisher(Bool, '/g1_1/task_hold', durable)
        hold.publish(Bool(data=True)); pump(.15)
        assert outputs['g1_1'] == Twist()
        hold.publish(Bool(data=False)); pump(.1, publish_commands=False)
        assert outputs['g1_1'] == Twist()  # no old command replay at hold release
        pump(.1)
        assert outputs['g1_1'].linear.x == pytest.approx(.2)
        poses['light_1'] = (1., 0., 0.); pump(.15)
        pump(.65, skip_pose={'light_1'})
        assert all(outputs[n] == Twist() for n in ('light_1', 'heavy_1'))
        assert outputs['g1_1'].linear.x == pytest.approx(.2)
        assert arbiter.core.tokens['test_aisle'].robot == 'light_1'
        poses['light_1'] = (5., 3., 0.); pump(.35)
        assert outputs['heavy_1'].linear.x == pytest.approx(.2)
        assert arbiter.core.tokens['test_aisle'].robot == 'heavy_1'
        # A directly submitted Nav2 goal also participates without a positive
        # fleet-manager intent. Losing its action server removes that intent.
        active_names.remove('light_1')
        status = peer.create_publisher(GoalStatusArray, '/light_1/navigate_to_pose/_action/status', durable)
        status.publish(GoalStatusArray(status_list=[GoalStatus(status=2)]))
        pump(.15)
        manual = NavPath(); manual.header.frame_id = 'map'
        for x in [5., 6.]:
            point = PoseStamped(); point.pose.position.x = x; point.pose.position.y = 3.
            point.pose.orientation.w = 1.; manual.poses.append(point)
        plans['light_1'].publish(manual); pump(.2)
        assert outputs['light_1'].linear.x == pytest.approx(.2)
        peer.destroy_publisher(status); pump(.35)
        assert not arbiter.core.active['light_1'] and outputs['light_1'] == Twist()
        executor.remove_node(arbiter); arbiter.destroy_node(); destroyed = True
        # No /clock exists: ROS time stays zero. The wall-time watchdog must
        # still stop a previously permitted command when the arbiter dies.
        pump(.5)
        assert outputs['heavy_1'] == Twist()
    finally:
        executor.shutdown()
        for node in [*gates, peer]: node.destroy_node()
        if not destroyed: arbiter.destroy_node()
        if owned: rclpy.shutdown()
