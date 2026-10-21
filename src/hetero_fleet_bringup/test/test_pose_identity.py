import importlib.util
from pathlib import Path
from types import SimpleNamespace
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

spec=importlib.util.spec_from_file_location('identity_relay',Path(__file__).resolve().parents[1]/'scripts/omni_odom_relay.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def relay():
    obj=object.__new__(module.OmniOdomRelay);obj.robot_name='omni_1'
    obj.spawn_x,obj.spawn_y,obj.spawn_yaw=4.6,-.45,0.
    obj.full_orientation=False;obj.ground_z=.117;obj.min_period=0.;obj.last_publish=None;obj.previous=None
    now=SimpleNamespace(nanoseconds=1_000_000_000,to_msg=lambda:TransformStamped().header.stamp)
    obj.get_clock=lambda:SimpleNamespace(now=lambda:now)
    data=[];obj.odom_pub=SimpleNamespace(publish=data.append);obj.tf_broadcaster=SimpleNamespace(sendTransform=lambda m:None)
    return obj,data


def pose(name,x):
    t=TransformStamped();t.child_frame_id=name;t.transform.translation.x=x;t.transform.translation.y=-.45;t.transform.rotation.w=1.
    return t


def test_delayed_model_never_borrows_nearest_robots_pose():
    obj,data=relay();obj.pose_callback(TFMessage(transforms=[pose('heavy_1',2.7)]))
    assert not data


def test_name_wins_over_distance_from_spawn():
    obj,data=relay();obj.pose_callback(TFMessage(transforms=[pose('heavy_1',4.6),pose('omni_1',10.)]))
    assert len(data)==1 and abs(data[0].pose.pose.position.x-5.4)<1e-9


def test_reordered_dynamic_array_keeps_correct_robot():
    obj,data=relay();obj.pose_callback(TFMessage(transforms=[pose('heavy_1',2.7),pose('omni_1',4.6)]))
    obj.pose_callback(TFMessage(transforms=[pose('omni_1',5.6),pose('heavy_1',2.7)]))
    assert abs(data[-1].pose.pose.position.x-1.)<1e-9


def test_anonymous_and_nested_link_poses_do_not_fake_model_odom():
    obj,data=relay();obj.pose_callback(TFMessage(transforms=[pose('',4.6),pose('omni_1::base_link',4.6)]))
    assert not data
