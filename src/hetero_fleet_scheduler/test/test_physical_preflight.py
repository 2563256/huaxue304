"""Prevent CP's optimistic timing routes from authorizing blocked missions."""
import threading
from types import SimpleNamespace
import pytest
from geometry_msgs.msg import PoseStamped
from hetero_fleet_scheduler.fleet_manager import FleetManager
from hetero_fleet_scheduler.task_model import Task,Pose2D,RobotState,RobotStatus,TaskState


def fixture(monkeypatch):
    m=object.__new__(FleetManager);m._lock=threading.RLock()
    m._task_preflight_cache={};m._task_preflight_busy=set()
    m._rolling=SimpleNamespace(invalidate=lambda *args:None)
    m.charger_pose={'a':Pose2D()};m.charger_exit={'a':Pose2D(.2,0)}
    scene={'a':(0.,0.,0.),'b':(3.,0.,0.)}
    m._dispatch_scene=lambda:scene.copy();m._at_charger=lambda *args:True
    m.get_logger=lambda:SimpleNamespace(info=lambda *a:None,warn=lambda *a:None)
    class Immediate:
        def __init__(self,target,**kw):self.target=target
        def start(self):self.target()
    monkeypatch.setattr(threading,'Thread',Immediate)
    return m,RobotState('a'),Task('t',pickup=Pose2D(2.,1.),dropoff=Pose2D(4.,2.)),scene


def poses(x,y=0.):
    p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y
    p.pose.orientation.z=.7071067811865475;p.pose.orientation.w=.7071067811865476
    return [p]


def test_all_three_legs_are_planned_with_actual_arrival_heading(monkeypatch):
    m,r,t,_=fixture(monkeypatch);calls=[]
    def request(robot,target,planner,start=None):
        calls.append((target,planner,start));return poses(target.x,target.y)
    m._request_target_path=request
    assert m._task_route_ready(r,t) is None
    assert m._task_route_ready(r,t) is True
    assert [c[1] for c in calls]==['GridBased','TaskGridBased','TaskGridBased']
    assert calls[0][2] is None
    assert calls[1][2].x==.2 and calls[1][2].yaw==pytest.approx(1.57079632679)
    assert calls[2][2].x==2.


@pytest.mark.parametrize('failed_leg',[0,1,2])
def test_any_blocked_leg_prevents_dispatch_without_failure(monkeypatch,failed_leg):
    m,r,t,_=fixture(monkeypatch);count=[]
    def request(*args):
        index=len(count);count.append(1);return [] if index==failed_leg else poses(1.)
    m._request_target_path=request;m._task_route_ready(r,t)
    assert m._task_route_ready(r,t) is False
    assert t.state==TaskState.PENDING and t.requeue_count==0
    assert len(count)==failed_leg+1


def test_moving_peer_invalidates_preflight_evidence(monkeypatch):
    m,r,t,scene=fixture(monkeypatch)
    def request(*args):scene['b']=(4.,0.,0.);return poses(1.)
    m._request_target_path=request;m._task_route_ready(r,t)
    assert ('a','t') not in m._task_preflight_cache


def test_existing_probe_for_robot_is_not_duplicated(monkeypatch):
    m,r,t,_=fixture(monkeypatch);m._task_preflight_busy.add('a')
    m._request_target_path=lambda *a:(_ for _ in ()).throw(AssertionError('duplicate request'))
    assert m._task_route_ready(r,t) is None


def test_idle_clearance_independent_of_fixed_batch_order(monkeypatch):
    m,r,t,_=fixture(monkeypatch);m.scheduling_method='cp_sat_rhcr';m.wheel_dispatch_order=[]
    m.idle_clearance_pose={'a':Pose2D(-7.,-.25)};m.startup_clearance_robots=set()
    m._work_remains=lambda:True;m._work_for=lambda r:False
    assert m._wheel_clearance_required(r)
    m._work_for=lambda r:True;assert not m._wheel_clearance_required(r)


def test_initial_g1_can_clear_even_with_queued_work(monkeypatch):
    m,r,t,_=fixture(monkeypatch);r.type='g1_humanoid';m.scheduling_method='cp_sat_rhcr'
    m.idle_clearance_pose={'a':Pose2D(1.2,-1.)};m.startup_clearance_robots={'a'}
    m._work_remains=lambda:True;m._work_for=lambda r:True
    assert m._wheel_clearance_required(r)
    r.tasks_completed=1;assert not m._wheel_clearance_required(r)


def test_peer_wait_does_not_consume_failure_retries(monkeypatch):
    m,r,t,_=fixture(monkeypatch);r.current_task_id='t';t.state=TaskState.TO_PICKUP
    m.pending=[];m._task_preflight_cache[('a','t')]=(0,{},True)
    m._defer_task(r,t,'peer blocks exit')
    assert m.pending==['t'] and t.requeue_count==0 and t.state==TaskState.PENDING
    assert r.status==RobotStatus.IDLE and not r.current_task_id


def test_loaded_robot_cannot_silently_requeue_pickup(monkeypatch):
    m,r,t,_=fixture(monkeypatch);r.current_payload_kg=1.;t.state=TaskState.TO_DROPOFF
    with pytest.raises(ValueError):m._defer_task(r,t,'blocked')
