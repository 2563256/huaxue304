"""Commit boundary checks, using the real manager dispatch method."""
import threading
import time
from types import SimpleNamespace
import pytest
from hetero_fleet_scheduler.fleet_manager import FleetManager
from hetero_fleet_scheduler.task_model import RobotState, Task, TaskState, RobotStatus, Pose2D
from hetero_fleet_scheduler.joint_schedule import Schedule
from hetero_fleet_scheduler.rolling_schedule import RollingSchedule


def manager(monkeypatch):
    m=object.__new__(FleetManager)
    m._lock=threading.Lock();m.use_sim_clock=False;m._now=lambda:50.
    m.tasks={n:Task(n,pickup=Pose2D(i*10.,0.),created_at_sec=1.) for i,n in enumerate(['t','u'])}
    m.pending=['t','u'];m.robots={n:RobotState(n) for n in ['a','b']}
    m._joint_worker_busy=True;m._joint_failure_at=-float('inf')
    m._task_preflight_cache={};m._task_preflight_busy=set()
    m._dispatch_scene=lambda:{'a':(0.,0.,0.),'b':(10.,0.,0.)}
    m._task_route_ready=lambda *a:True
    m._rolling=RollingSchedule(None)
    m._rolling.plan=Schedule(50.,'FEASIBLE',rows=[
        dict(task_id='t',robot_id='a',start=50.,end=60.,groups=['g']),
        dict(task_id='u',robot_id='b',start=100.,end=110.,groups=['g'])])
    m._rolling.key=(('t','u'),(('a','IDLE',''),('b','IDLE','')),())
    m._rolling.last=50.
    m._channel_snapshot=({'pose_ready':True},time.monotonic())
    m._can_serve=lambda *args:True
    m._locomotion_available=lambda *args:True
    m._navigation_available=lambda *args:True
    m._should_clear_channel=lambda *args:False
    m._pickup_ready=lambda *args:True
    m.get_parameter=lambda key:SimpleNamespace(value=2.)
    m.get_logger=lambda:SimpleNamespace(info=lambda msg:None)
    m.executed=[];m._execute_task=lambda r,t:m.executed.append((r.id,t.id))
    m.metrics=SimpleNamespace(m=SimpleNamespace(first_task_at_sec=1.))
    class Immediate:
        def __init__(self,target,args=(),**kwargs): self.target=target;self.args=args
        def start(self): self.target(*self.args)
    monkeypatch.setattr(threading,'Thread',Immediate)
    return m


def test_only_near_term_job_commits_and_old_plan_is_invalidated(monkeypatch):
    m=manager(monkeypatch);m._try_assign_joint()
    assert m.executed==[('a','t')]
    assert m.tasks['t'].state==TaskState.ASSIGNED and m.tasks['u'].state==TaskState.PENDING
    assert m.pending==['u'] and m._rolling.key is None


def test_stale_traffic_feedback_holds_dispatch(monkeypatch):
    m=manager(monkeypatch);m._channel_snapshot=({'pose_ready':True},time.monotonic()-10)
    m._try_assign_joint();assert not m.executed


def test_live_busy_robot_is_never_preempted_by_old_forecast(monkeypatch):
    m=manager(monkeypatch);m.robots['a'].status=RobotStatus.TO_PICKUP
    m._try_assign_joint();assert not m.executed


def test_navigation_and_locomotion_readiness_still_required(monkeypatch):
    m=manager(monkeypatch);m._locomotion_available=lambda *args:False
    m._try_assign_joint();assert not m.executed


def test_real_planner_veto_keeps_task_pending_without_failure(monkeypatch):
    m=manager(monkeypatch);m._task_route_ready=lambda *args:False
    m._try_assign_joint()
    assert not m.executed and m.tasks['t'].state==TaskState.PENDING
    assert m.tasks['t'].requeue_count==0


def test_unknown_real_route_is_not_treated_as_feasible(monkeypatch):
    m=manager(monkeypatch);m._task_route_ready=lambda *args:None
    m._try_assign_joint();assert not m.executed
