"""Reproduce the fresh-run G1 false-clearance and repeated CP head blockage."""
import threading
import time
from types import SimpleNamespace

from hetero_fleet_scheduler.fleet_manager import FleetManager
from hetero_fleet_scheduler.task_model import Pose2D, RobotState, RobotStatus


def manager():
    m=object.__new__(FleetManager);m._lock=threading.RLock()
    r=RobotState('g1',type='g1_humanoid')
    m.robots={'g1':r};m.live_pose={'g1':(1.22,-.84,0.)}
    m.charger_pose={'g1':Pose2D(1.2,-.8)}
    m.idle_clearance_pose={'g1':Pose2D(1.2,-1.08)}
    m.clearance_arrive_radius={'g1':.12};m.charger_arrive_radius_m=.35
    m.startup_clearance_robots={'g1'};m.scheduling_method='cp_sat_rhcr'
    m._work_remains=lambda:True;m._work_for=lambda r:True
    m._should_clear_idle_g1=lambda r:False;m._should_clear_channel=lambda n:False
    m._locomotion_available=lambda *a:True;m._now=lambda:10.
    m.return_to_charger=True;m._idle_since={'g1':0.}
    m.dock_when_idle_sec=3.;m.clear_task_point_sec=1.;m.clear_task_point_radius_m=1.2
    m._on_task_point=lambda *a:False
    m.get_logger=lambda:SimpleNamespace(info=lambda *a:None,warn=lambda *a:None,error=lambda *a:None)
    return m,r


def test_spawn_near_standby_is_not_actual_clearance():
    m,r=manager();target=m.idle_clearance_pose[r.id]
    assert m._at_charger(r,target)  # Old shared radius wrongly admitted this.
    assert not m._at_clearance(r,target)
    m.live_pose[r.id]=(1.2,-1.04,0.)
    assert m._at_clearance(r,target)
    m.live_pose[r.id]=(1.285,-1.10,0.)
    assert m._at_clearance(r,target)  # Ordinary biped stance oscillation.
    m.live_pose_stamps={r.id:time.monotonic()-2.}
    assert not m._at_clearance(r,target)


def test_idle_tick_really_starts_first_g1_clearance_with_queued_work():
    m,r=manager();started=[];m._start_docking=lambda robot:started.append(robot.id)
    m._update_idle_timers();assert started==[r.id]
    started.clear();m.live_pose[r.id]=(1.2,-1.04,0.)
    m._update_idle_timers();assert started==[]


def test_home_worker_cannot_skip_precise_clearance_as_already_home():
    m,r=manager();moves=[]
    m._apply_battery_drain=lambda *a:None
    m._plan_docking_route=lambda robot,home,via:[home]
    def navigate(robot,target,tag):
        moves.append(target);m.live_pose[r.id]=(target.x,target.y,0.)
        return True,.24,.48
    m._navigate=navigate
    m._drive_home(r)
    assert moves==[m.idle_clearance_pose[r.id]]
    assert r.status==RobotStatus.IDLE and r.odom_distance_m==.24


def test_negative_preflight_survives_solver_cycle_but_not_peer_clearance():
    m,r=manager();scene={'g1':(1.22,-.84,0.)}
    old=time.monotonic()-20.
    assert m._task_preflight_valid((old,scene,False),scene)
    assert not m._task_preflight_valid((old,scene,True),scene)
    assert not m._task_preflight_valid((old,scene,False),{'g1':(1.2,-1.04,0.)})
    assert not m._task_preflight_valid((time.monotonic()-61.,scene,False),scene)


def test_west_refuge_releases_after_its_charging_row_dependencies_finish():
    m,g1=manager();omni=RobotState('omni',type='omni_mecanum_amr')
    light=RobotState('light',type='light_diff_amr');heavy=RobotState('heavy',type='heavy_diff_amr')
    m.robots.update(omni=omni,light=light,heavy=heavy)
    m.idle_clearance_pose['omni']=Pose2D(-4.8,-.25)
    m.clearance_release_after_types={'omni':{'heavy_diff_amr','g1_humanoid'}}
    work={'light','g1'};m._work_for=lambda robot:robot.id in work
    assert m._wheel_clearance_required(omni)
    work.remove('g1');g1.current_task_id='last_delivery'
    assert m._wheel_clearance_required(omni)
    g1.current_task_id=''
    assert not m._wheel_clearance_required(omni)


def test_idle_tick_does_not_hold_released_refuge_forever():
    m,r=manager();r.type='omni_mecanum_amr';r.tasks_completed=3
    m.clearance_release_after_types={'g1':{'heavy_diff_amr','g1_humanoid'}}
    m._work_for=lambda robot:False;m.live_pose['g1']=(-4.8,-.25,0.)
    m.idle_clearance_pose['g1']=Pose2D(-4.8,-.25)
    m.charger_pose['g1']=Pose2D(4.6,-.45)
    started=[];m._start_docking=lambda robot:started.append(robot.id)
    m._update_idle_timers()
    assert started==[r.id]
