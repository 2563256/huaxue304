"""Real ROS schedule feedback loop, with no Gazebo/Nav2 action server."""
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from std_msgs.msg import String
from hetero_fleet_scheduler.fleet_manager import FleetManager
from hetero_fleet_scheduler.channel_node import ChannelReservationNode
from hetero_fleet_scheduler.task_model import Task,Pose2D,TaskState


def test_real_schedule_topic_reaches_arbiter_but_does_not_dispatch_without_nav(tmp_path):
    owned=not rclpy.ok()
    if owned:rclpy.init()
    manager=FleetManager()
    # A different ROS domain isolates topics, not filesystem reports.
    manager.metrics_dir=str(tmp_path)
    arbiter=ChannelReservationNode(parameter_overrides=[Parameter('use_sim_time',value=False)])
    peer=Node('joint_feedback_test')
    messages=[]
    peer.create_subscription(String,'/fleet_schedule',lambda m:messages.append(json.loads(m.data)),1)
    executor=MultiThreadedExecutor(num_threads=4)
    for n in [manager,arbiter,peer]:executor.add_node(n)
    task=Task('joint_ros_test',pickup=Pose2D(10.6,-.43),dropoff=Pose2D(0.,4.1),
              required_capability='light_payload',payload_kg=8.,created_at_sec=manager._now())
    try:
        # Spin first to establish publisher matching; no command/action server
        # exists, so even a feasible CP plan must remain uncommitted.
        until=time.monotonic()+1.
        while time.monotonic()<until: executor.spin_once(timeout_sec=.05)
        manager.submit_task(task)
        until=time.monotonic()+10.
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.05)
            if messages and arbiter.core.hint_revision>=0:break
        assert messages and messages[-1]['status'] in ('FEASIBLE','OPTIMAL')
        assert messages[-1]['rows'][0]['task_id']=='joint_ros_test'
        assert arbiter.core.hint_revision>=0 and arbiter.core.task_hints
        assert manager.tasks[task.id].state==TaskState.PENDING
        assert not arbiter.core.tokens
    finally:
        until=time.monotonic()+3.
        while manager._joint_worker_busy and time.monotonic()<until:
            executor.spin_once(timeout_sec=.05)
        executor.shutdown(timeout_sec=2.)
        for n in [manager,arbiter,peer]:n.destroy_node()
        if owned:rclpy.try_shutdown()
