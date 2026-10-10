"""Check corner coverage, rotation, seam handling, and missing lidar data."""
import importlib.util
import math
from pathlib import Path
import numpy as np
import pytest
import yaml
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

SCRIPTS=Path(__file__).resolve().parents[1]/'scripts'
spec=importlib.util.spec_from_file_location('scan_frame_relay',SCRIPTS/'scan_frame_relay.py')
relay=importlib.util.module_from_spec(spec);spec.loader.exec_module(relay)


def test_rotated_body_not_inscribed_circle():
    # Long side faces the observer: the actual near face must be marked,
    # even though it lies well outside the old width/2 circle.
    out=relay.rectangle_returns([0],2,0,0,.856,.645)
    assert out[0]==pytest.approx(2-.856/2)


def test_sideways_body_uses_width():
    out=relay.rectangle_returns([0],2,0,math.pi/2,.856,.645)
    assert out[0]==pytest.approx(2-.645/2)


def test_backward_seam_sees_both_sides():
    out=relay.rectangle_returns([-math.pi,math.pi-1e-4],-2,0,0,.672,.588)
    assert np.isfinite(out).all()
    assert out[0]==pytest.approx(2-.672/2)


def test_empty_and_receded_objects_clear_previous_ray():
    assert np.isfinite(relay.rectangle_returns([0],2,0,0,.672,.588)[0])
    assert np.isinf(relay.rectangle_returns([0],2,2,0,.672,.588)[0])
    assert np.isinf(relay.rectangle_returns([0],20,0,0,.672,.588,range_max=12)[0])


def test_observer_inside_body_reports_contact():
    assert np.all(relay.rectangle_returns([-2,0,1],0,0,.7,.672,.588)==.01)


@pytest.mark.parametrize('ego', ['light_1','heavy_1','omni_1'])
def test_all_robots_see_both_other_bodies_without_physical_lidar(ego):
    class Clock:
        nanoseconds=12_000_000_000
        def now(self):return self
        def to_msg(self):return LaserScan().header.stamp
    class Publisher:
        def publish(self,m):self.msg=m
    obj=object.__new__(relay.ScanFrameRelay)
    dims={'light_1':(.6888,.5412),'heavy_1':(.8556,.6448),'omni_1':(.672,.588)}
    others=[name for name in dims if name!=ego]
    poses={ego:(0.,0.,0.),others[0]:(2.,0.,0.),others[1]:(-2.,0.,0.)}
    obj.robots={name:{'pose':poses[name],'footprint':dims[name]} for name in dims}
    obj.robots[others[0]]['stationary_since']=0.0
    obj.robots[others[1]]['stationary_since']=12.0
    obj.samples=1080;obj.rate=10.;obj.max_range=12.;obj.padding=.02
    obj.angles=np.arange(1080)*2*math.pi/1080-math.pi
    obj.obstacle_pubs={name:Publisher() for name in dims}
    obj.contact_pubs={name:Publisher() for name in dims}
    obj.planning_pubs={name:Publisher() for name in dims}
    obj.stationary_sec=10.0
    obj.get_clock=lambda:Clock()
    obj.publish_robot_obstacles()
    scan=obj.obstacle_pubs[ego].msg
    assert scan.header.frame_id==ego+'/base_footprint'
    assert scan.ranges[540]==pytest.approx(2-dims[others[0]][0]/2-.02)
    assert scan.ranges[0]==pytest.approx(2-dims[others[1]][0]/2-.02)
    assert obj.contact_pubs[ego].msg.ranges[540]==pytest.approx(2-dims[others[0]][0]/2)

    assert np.isfinite(obj.planning_pubs[ego].msg.ranges[540])
    assert np.isinf(obj.planning_pubs[ego].msg.ranges[0])


def test_stop_is_never_overridden_or_mutates_source():
    spec=importlib.util.spec_from_file_location('stamper',SCRIPTS/'cmd_vel_stamper.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    class Clock:
        nanoseconds=0
        def now(self):return self
        def to_msg(self):return LaserScan().header.stamp
    class Publisher:
        def __init__(self):self.msg=[]
        def publish(self,m):self.msg.append(m)
    obj=object.__new__(mod.CmdVelStamper);obj.get_clock=lambda:Clock()
    obj.pub=Publisher();obj.mirror_y=True;obj.angular_z_scale=.8
    t=Twist();t.linear.y=.2;t.angular.z=.5;obj.on_command(t)
    assert t.linear.y==.2
    assert obj.pub.msg[-1].twist.linear.y==-.2
    assert obj.pub.msg[-1].twist.angular.z==pytest.approx(.4)
    assert t.angular.z==.5
    for _ in range(50):obj.on_command(Twist())
    assert all(m.twist.linear.x==m.twist.linear.y==m.twist.angular.z==0 for m in obj.pub.msg[1:])
    assert not hasattr(obj,'check_override')


def test_each_costmap_marks_and_clears_teammates_and_has_no_slowdown():
    config=SCRIPTS.parent/'config'
    for typ in ['light','heavy','omni']:
        data=yaml.safe_load((config/f'nav2_params_{typ}.yaml').read_text())
        for name in ['local_costmap','global_costmap']:
            p=data[name][name]['ros__parameters']
            assert 'robot_obstacle_layer' in p['plugins']
            obs=p['robot_obstacle_layer']['robot_obstacles']
            assert obs['marking'] and obs['clearing'] and obs['inf_is_valid']
        assert data['controller_server']['ros__parameters']['FollowPath']['ObstaclesCritic']['consider_footprint']
        monitor=data['collision_monitor']['ros__parameters']
        assert monitor['polygons']==['PolygonStop']
        assert 'slowdown' not in str(monitor)


def test_directional_stop_protects_motion_and_all_sides_when_turning():
    for typ in ['light','heavy','omni']:
        data=yaml.safe_load((SCRIPTS.parent/'config'/f'nav2_params_{typ}.yaml').read_text())
        stop=data['collision_monitor']['ros__parameters']['PolygonStop']
        assert stop['type']=='velocity_polygon' and stop['action_type']=='stop'
        front=np.array(yaml.safe_load(stop['forward']['points']))
        assert front[:,0].min()==0 and front[:,0].max()>0
        assert stop['forward']['theta_max']==.1
        body=np.array(yaml.safe_load(data['local_costmap']['local_costmap']['ros__parameters']['footprint']))
        turning=np.array(yaml.safe_load(stop['turning']['points']))
        assert np.all(turning.min(axis=0)<body.min(axis=0))
        assert np.all(turning.max(axis=0)>body.max(axis=0))
        assert stop['velocity_polygons'][-1]=='turning'


def test_omni_direction_ranges_use_radians_and_do_not_select_front_when_reversing():
    stop=yaml.safe_load((SCRIPTS.parent/'config/nav2_params_omni.yaml').read_text())['collision_monitor']['ros__parameters']['PolygonStop']
    f=stop['forward'];r=stop['reverse_left']
    assert f['direction_start_angle']==pytest.approx(-math.pi/4)
    assert f['direction_end_angle']==pytest.approx(math.pi/4)
    assert not (f['direction_start_angle']<=math.pi<=f['direction_end_angle'])
    assert r['direction_start_angle']<=math.pi<=r['direction_end_angle']


def test_stationary_detection_resets_when_motion_resumes():
    state={}
    relay.update_stationary(state,(0,0,0),0)
    relay.update_stationary(state,(.01,0,0),12)
    assert state['stationary_since']==0
    relay.update_stationary(state,(.01,0,0),13,linear_speed=.10)
    assert state['stationary_since']==13


def test_stationary_detection_uses_net_displacement_and_rotation():
    state={}
    relay.update_stationary(state,(0,0,0),0)
    relay.update_stationary(state,(.03,0,0),1)
    relay.update_stationary(state,(.06,0,0),2)
    assert state['stationary_since']==2
    relay.update_stationary(state,(.06,0,.2),3)
    assert state['stationary_since']==3


def test_stationary_detection_restarts_on_simulation_reset():
    state={}
    relay.update_stationary(state,(0,0,0),100)
    relay.update_stationary(state,(0,0,0),1)
    assert state['stationary_since']==1


def test_omni_only_parked_bodies_enter_global_planning():
    data=yaml.safe_load((SCRIPTS.parent/'config/nav2_params_omni.yaml').read_text())
    glob=data['global_costmap']['global_costmap']['ros__parameters']
    local=data['local_costmap']['local_costmap']['ros__parameters']
    assert 'obstacle_layer' not in glob['plugins']
    assert glob['robot_obstacle_layer']['robot_obstacles']['topic'].endswith('/robot_planning_obstacles')
    assert 'obstacle_layer' in local['plugins']
    assert local['robot_obstacle_layer']['robot_obstacles']['topic'].endswith('/robot_obstacles')
