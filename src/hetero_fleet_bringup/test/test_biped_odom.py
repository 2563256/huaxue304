"""A high humanoid lidar must follow real roll/pitch, unlike planar AMRs."""
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
spec = importlib.util.spec_from_file_location('odom_relay', SCRIPTS / 'omni_odom_relay.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


@pytest.mark.parametrize('full_orientation', [True, False])
def test_biped_roll_and_root_height_reach_sensor_tf(full_orientation):
    obj = object.__new__(module.OmniOdomRelay)
    obj.robot_name = 'g1_1'
    obj.spawn_x, obj.spawn_y, obj.spawn_yaw = 1.2, -.8, 0.0
    obj.full_orientation, obj.ground_z = full_orientation, .117
    obj.min_period, obj.last_publish, obj.previous, obj.pose_index = 0, None, None, 0
    now = SimpleNamespace(nanoseconds=1_000_000_000,
                          to_msg=lambda: TransformStamped().header.stamp)
    obj.get_clock = lambda: SimpleNamespace(now=lambda: now)
    published, transforms = [], []
    obj.odom_pub = SimpleNamespace(publish=published.append)
    obj.tf_broadcaster = SimpleNamespace(sendTransform=transforms.append)
    transform = TransformStamped()
    transform.child_frame_id = 'g1_1'
    transform.transform.translation.x = 1.2
    transform.transform.translation.y = -.8
    transform.transform.translation.z = .107
    transform.transform.rotation.x = math.sin(.1 / 2)
    transform.transform.rotation.w = math.cos(.1 / 2)
    msg = TFMessage(transforms=[transform])
    obj.pose_callback(msg)
    odom = published[0]
    assert transforms[0].transform.rotation.x == 0.0
    assert transforms[0].transform.translation.z == 0.0
    tf = transforms[-1].transform
    if full_orientation:
        assert tf.rotation.x == pytest.approx(math.sin(.1 / 2))
        assert tf.translation.z == pytest.approx(-.01)
        # A 1.274 m high sensor moves sideways when the trunk rolls.
        projected_y = -2 * tf.rotation.x * tf.rotation.w * 1.274
        assert projected_y == pytest.approx(-math.sin(.1) * 1.274)
    else:
        assert tf.rotation.x == 0.0
        assert tf.translation.z == 0.0
    assert odom.pose.pose.orientation.x == 0.0
    assert odom.pose.pose.position.z == 0.0


def test_pose_timestamp_and_velocity_use_acquisition_time():
    obj = object.__new__(module.OmniOdomRelay)
    obj.robot_name='omni_1';obj.spawn_x=obj.spawn_y=obj.spawn_yaw=0.
    obj.full_orientation=False;obj.min_period=0.;obj.last_publish=None
    obj.previous=(10.,0.,0.,0.)
    obj.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=20_000_000_000))
    published=[];obj.odom_pub=SimpleNamespace(publish=published.append)
    obj.tf_broadcaster=SimpleNamespace(sendTransform=lambda _:None)
    t=TransformStamped();t.child_frame_id='omni_1';t.header.stamp.sec=11
    t.transform.translation.x=.2;t.transform.rotation.w=1.
    obj.pose_callback(TFMessage(transforms=[t]))
    assert published[0].header.stamp.sec==11
    assert published[0].twist.twist.linear.x==pytest.approx(.2)
