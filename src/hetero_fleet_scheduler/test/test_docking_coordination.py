"""Return traffic must not become phantom transport tasks or a blocked convoy."""
import threading
import time
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from hetero_fleet_scheduler.fleet_manager import FleetManager
from hetero_fleet_scheduler.metrics import MetricsRecorder
from hetero_fleet_scheduler.task_model import Pose2D, RobotState, RobotStatus, Task, TaskState
import rclpy


def manager():
    m=object.__new__(FleetManager)
    m._lock=threading.RLock();m._idle_since={};m.pending=[];m.tasks={}
    m.robots={n:RobotState(n,status=RobotStatus.IDLE) for n in ('light','omni','g1')}
    m._dispatch_scene=lambda:{'g1':(11.,4.,0.)}
    m.planner_clients={'g1':object()}
    m._now=lambda:0.
    return m


def path(target):
    p=PoseStamped();p.pose.position.x=target.x;p.pose.position.y=target.y
    p.pose.orientation.w=1.
    return [p]


def test_blocked_approach_does_not_hide_reachable_home():
    m=manager();home=Pose2D(1.2,-.8);via=Pose2D(3.8,-.2);calls=[]
    def request(robot,target,planner,start=None):
        calls.append((target,start));return [] if target==via else path(target)
    m._request_target_path=request
    assert m._plan_docking_route(m.robots['g1'],home,[via])==[home]
    assert [c[0] for c in calls]==[via,home]
    assert calls[-1][1] is None


def test_valid_approach_is_preserved_and_chains_actual_heading():
    m=manager();home=Pose2D(1.2,-.8);via=Pose2D(3.8,-.2);starts=[]
    def request(robot,target,planner,start=None):
        starts.append(start);return path(target)
    m._request_target_path=request
    assert m._plan_docking_route(m.robots['g1'],home,[via])==[via,home]
    assert starts[0] is None and starts[1].x==via.x


def test_no_route_or_moving_scene_does_not_authorize_docking():
    m=manager();home=Pose2D(1.2,-.8)
    m._request_target_path=lambda *a:[]
    assert m._plan_docking_route(m.robots['g1'],home,[]) is None
    scenes=iter([{'g1':(11.,4.,0.)},{'g1':(12.,4.,0.)}])
    m._dispatch_scene=lambda:next(scenes)
    m._request_target_path=lambda robot,target,*a:path(target)
    assert m._plan_docking_route(m.robots['g1'],home,[]) is None


def test_single_docking_owner_and_cooldown_let_other_candidates_run(monkeypatch):
    m=manager();workers=[]
    class HeldThread:
        def __init__(self,target,args,**kwargs):workers.append(args[0].id)
        def start(self):pass
    monkeypatch.setattr(threading,'Thread',HeldThread)
    m._start_docking(m.robots['light'])
    m._start_docking(m.robots['omni'])
    assert workers==['light']
    assert m.robots['omni'].status==RobotStatus.IDLE
    m._docking_owner=None;m.robots['light'].status=RobotStatus.IDLE
    m._docking_retry_at={'light':float('inf')}
    m._start_docking(m.robots['light']);m._start_docking(m.robots['g1'])
    assert workers==['light','g1']


def test_returning_robots_do_not_count_as_active_transport_tasks():
    m=manager();published=[]
    for r in m.robots.values():r.status=RobotStatus.CHARGING
    m.tasks={'done':Task('done',state=TaskState.COMPLETED)}
    m.metrics=MetricsRecorder(list(m.robots));m.metrics.m.tasks_completed=1
    m.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(to_msg=Time))
    m.status_pub=SimpleNamespace(publish=published.append)
    m._publish_status()
    assert published[-1].tasks_active==0 and published[-1].tasks_completed==1
    m.tasks['actual']=Task('actual',state=TaskState.TO_DROPOFF)
    m._publish_status()
    assert published[-1].tasks_active==1


def test_return_wait_yields_docking_turn_then_reacquires_after_evacuation():
    m=manager();r=m.robots['omni'];r.status=RobotStatus.CHARGING
    m._docking_owner=r.id;m._now=time.monotonic;m.channel_wait_timeout=2.
    m._channel_exit_owner=lambda *a:None
    m._locomotion_available=lambda *a:True
    m._channel_wait_states={r.id:(True,time.monotonic())}
    observed=[]
    def evacuate():
        until=time.monotonic()+1.
        while m._docking_owner is not None and time.monotonic()<until:time.sleep(.01)
        observed.append(m._docking_owner)
        with m._lock:m._docking_owner='heavy'
        m._channel_wait_states[r.id]=(False,time.monotonic())
        time.sleep(.1)
        assert m._docking_owner=='heavy'
        with m._lock:m._docking_owner=None
    owned=not rclpy.ok()
    if owned:rclpy.init()
    worker=threading.Thread(target=evacuate);worker.start()
    try:
        assert m._wait_channel_turn(r,None)[0]
        assert observed==[None] and m._docking_owner==r.id
    finally:
        worker.join(timeout=2.)
        if owned:rclpy.try_shutdown()


def test_refuge_does_not_block_future_unload_of_an_already_assigned_task():
    m=manager();r=m.robots['omni'];r.type='mecanum';light=m.robots['light'];light.type='diff'
    m.types={'mecanum':{'footprint':[1.285714,1.125],'footprint_padding':.02},
             'diff':{'footprint':[1.431818,1.125],'footprint_padding':.03}}
    light.current_task_id='committed'
    m.tasks={'committed':Task('committed',pickup=Pose2D(1.7,4.22),dropoff=Pose2D(8.1,-.43),state=TaskState.TO_PICKUP)}
    assert m._parking_blocks_committed_task(r,(7.5,-.25,0.))
    assert not m._parking_blocks_committed_task(r,(6.1,-.25,0.))
    light.current_task_id=''
    assert not m._parking_blocks_committed_task(r,(7.5,-.25,0.))


def test_new_commit_cancels_a_retreat_that_would_occupy_its_dropoff():
    m=manager();r=m.robots['omni'];r.type='mecanum';light=m.robots['light'];light.type='diff'
    m.types={'mecanum':{'footprint':[1.285714,1.125],'footprint_padding':.02},
             'diff':{'footprint':[1.431818,1.125],'footprint_padding':.03}}
    m._channel_snapshot=({'maneuvers':{'retreat_waypoints':[[7.5,-.25,0.]]}},time.monotonic())
    m._channel_maneuver=lambda *a:Pose2D(7.5,-.25)
    m.get_logger=lambda:SimpleNamespace(info=lambda *a:None)
    def navigate(robot,home,tag,**kwargs):
        assert not kwargs['cancel_when']()
        light.current_task_id='new'
        m.tasks['new']=Task('new',pickup=Pose2D(1.7,4.22),dropoff=Pose2D(8.1,-.43),state=TaskState.TO_PICKUP)
        assert kwargs['cancel_when']()
        return False,.1,.2
    m._navigate=navigate
    assert m._retreat_from_channel_queue(r,'heavy',None)==.1
    assert not m._channel_retreating
