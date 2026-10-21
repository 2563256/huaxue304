"""Transient physical blockage waits are bounded and never complete a task."""
import time
from types import SimpleNamespace
import pytest
from test_standoff import make_manager, place, FakeGoalHandle, resolved
from test_goal_geometry import scene
from hetero_fleet_scheduler.task_model import Pose2D


def scripted(errors, step=5.):
    m=make_manager();place(m,(0.,0.,0.));clock=[0.];handles=[];waits=[]
    m._now=lambda:clock[0];m.stall_max_recoveries=0
    m._has_nearby_route_peer=lambda *args:True
    def send(robot,target,tag):
        code=errors.pop(0);h=FakeGoalHandle('script',None)
        if code==0:m.live_pose['r1']=(target.x,target.y,.7)
        h.result=resolved(SimpleNamespace(status=6 if code else 4,result=SimpleNamespace(error_code=code)))
        handles.append(h);return h,None
    def dwell(seconds):waits.append(seconds);clock[0]+=step;return True
    m._send_goal=send;m._dwell=dwell
    m._back_off=lambda *args:pytest.fail('A transient route blockage must not trigger reverse motion')
    return m,handles,waits


@pytest.mark.parametrize('error',[205,206,207,208])
def test_fresh_peer_blockage_resumes_without_consuming_recovery(error):
    m,handles,waits=scripted([error,error,0])
    assert m._navigate(m.robots['r1'],Pose2D(2.,0.,0.),'task:pickup')[0]
    assert waits==[5.,5.] and len(handles)==3
    assert m.robots['r1'].pose.x==2. and m.robots['r1'].pose.yaw==.7
    assert handles[0].canceled and handles[1].canceled


def test_peer_wait_has_a_finite_ros_time_budget_and_does_not_fake_arrival():
    m,handles,waits=scripted([208,208,208],step=100.)
    assert not m._navigate(m.robots['r1'],Pose2D(2.,0.,0.),'task:pickup')[0]
    assert len(handles)==3 and waits==[5.,5.]
    assert m.robots['r1'].pose.x==0.


def test_no_peer_or_static_collision_is_not_treated_as_transient():
    m,handles,waits=scripted([205]);m._has_nearby_route_peer=lambda *args:False
    assert not m._navigate(m.robots['r1'],Pose2D(2.,0.,0.),'task:pickup')[0]
    assert len(handles)==1 and waits==[]


def test_peer_wait_remains_cancellable():
    m,handles,waits=scripted([208]);cancelled=[False]
    def dwell(seconds):cancelled[0]=True;return True
    m._dwell=dwell
    assert not m._navigate(m.robots['r1'],Pose2D(2.,0.,0.),'task:pickup',cancel_when=lambda:cancelled[0])[0]
    assert len(handles)==1 and m.robots['r1'].pose.x==0.


def test_peer_evidence_requires_fresh_odometry_and_static_free_current_body():
    m=make_manager();occupancy,profiles=scene();m._occupancy=occupancy;m.types=profiles['robot_types']
    place(m,(-.3,-.25,0.));m.live_pose['peer']=(2.,-.25,0.)
    m.live_pose_stamps={'r1':time.monotonic(),'peer':time.monotonic()}
    assert m._has_nearby_route_peer(m.robots['r1'],Pose2D(5.,-.25,0.))
    m.live_pose_stamps['peer']=time.monotonic()-1.
    assert not m._has_nearby_route_peer(m.robots['r1'],Pose2D(5.,-.25,0.))
    m.live_pose_stamps['peer']=time.monotonic();place(m,(1.2,.6,1.57))
    assert not m._has_nearby_route_peer(m.robots['r1'],Pose2D(5.,-.25,0.))
