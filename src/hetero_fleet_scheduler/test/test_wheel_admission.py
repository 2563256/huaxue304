"""Forecourt batches release on actual parking; G1 never takes a wheel turn."""
from test_standoff import make_manager, make_task, place
from hetero_fleet_scheduler.task_model import Pose2D, RobotState, RobotStatus


def batch():
    m=make_manager(); m.wheel_dispatch_order=['outer','r1']
    m.wheel_batch_collect_sec=60.;m._wheel_batch_started=None;m._wheel_batch_robot=None
    m.idle_clearance_pose={'outer':Pose2D(-5.2,-.25,0.)}
    m.robots['outer']=RobotState(id='outer',type='omni_mecanum_amr',pose=Pose2D(4.6,-.45,0.),status=RobotStatus.IDLE,capabilities=['outer_only'],payload_capacity_kg=10.)
    m.robots['r1'].payload_capacity_kg=10.;m.robots['r1'].status=RobotStatus.IDLE;m.robots['r1'].capabilities=['inner_only']
    m.tasks={'a':make_task('a',1.,'outer_only'),'b':make_task('b',1.,'inner_only')};m.pending=['b','a']
    m.live_pose['outer']=(4.6,-.45,0.)
    return m


def test_collection_window_then_configured_order_overrides_task_arrival_order():
    m=batch()
    assert m._wheel_admission_owner(10.)==''
    assert m._wheel_admission_owner(69.)==''
    assert m._wheel_admission_owner(70.)=='outer'


def test_empty_queue_does_not_release_before_actual_parking():
    m=batch();m._wheel_admission_owner(0.);assert m._wheel_admission_owner(60.)=='outer'
    m.pending.remove('a')
    # A bookkeeping goal and even IDLE status do not establish physical arrival.
    m.robots['outer'].pose=Pose2D(-5.2,-.25,0.)
    assert m._wheel_admission_owner(70.)=='outer'
    m.live_pose['outer']=(-5.2,-.25,0.)
    assert m._wheel_admission_owner(71.)=='r1'


def test_busy_owner_retains_batch_even_at_parking_coordinate():
    m=batch();m._wheel_admission_owner(0.);m._wheel_admission_owner(60.)
    m.pending.remove('a');m.live_pose['outer']=(-5.2,-.25,0.)
    m.robots['outer'].status=RobotStatus.CHARGING
    assert m._wheel_admission_owner(70.)=='outer'


def test_no_configuration_keeps_existing_dispatch_behavior():
    assert make_manager()._wheel_admission_owner(0.) is None


def test_pending_wheel_job_does_not_hide_later_priority_g1_job(monkeypatch):
    from types import SimpleNamespace
    import hetero_fleet_scheduler.fleet_manager as module
    from hetero_fleet_scheduler.cost import CostWeights
    m=batch();m.wheel_dispatch_order=[];m.weights=CostWeights()
    m.load_time=m.unload_time=3.;m.use_sim_clock=False
    m.robots['g']=RobotState(id='g',type='g1_humanoid',pose=Pose2D(),
        status=RobotStatus.IDLE,capabilities=['g1_only'],payload_capacity_kg=5.)
    m.tasks['gjob']=make_task('gjob',1.,'g1_only');m.pending.append('gjob')
    m.metrics=SimpleNamespace(note_reschedule=lambda:None)
    m._g1_dispatch_priority=lambda:({('g','gjob')},True)
    m._locomotion_available=lambda *a:True;m._navigation_available=lambda *a:True
    m._should_clear_channel=lambda *a:False;m._pickup_ready=lambda *a:True
    workers=[]
    monkeypatch.setattr(module.threading,'Thread',lambda **kw:SimpleNamespace(start=lambda:workers.append(kw)))
    m._try_assign_all()
    assert m.robots['g'].current_task_id=='gjob'
    assert m.pending==['b','a'] and len(workers)==1
    assert not m.robots['r1'].current_task_id


def test_g1_transport_makes_new_wheel_jobs_yield_without_a_corridor_token():
    m=batch();m.wheel_yield_to_g1=True
    m.robots['g']=RobotState(id='g',type='g1_humanoid',current_task_id='gjob',status=RobotStatus.TO_PICKUP)
    m._dispatch_path_ready=lambda *a:(_ for _ in ()).throw(AssertionError('Active G1 is already admitted'))
    assert m._g1_dispatch_priority()==(set(),True)


def test_scene_cache_does_not_survive_a_peer_moving_or_missing_pose():
    from hetero_fleet_scheduler.fleet_manager import FleetManager
    a={'g':(0.,0.,0.),'wheel':(3.,0.,0.)}
    assert FleetManager._same_dispatch_scene(a,{'g':(.01,0.,0.),'wheel':(3.01,0.,0.)})
    assert not FleetManager._same_dispatch_scene(a,{'g':(0.,0.,0.),'wheel':(3.3,0.,0.)})
    assert not FleetManager._same_dispatch_scene(a,None)


def test_initial_idle_omni_does_not_leave_its_bay_for_end_of_batch_parking():
    m=batch();m.startup_clearance_robots=set();m._now=lambda:100.
    m._idle_since={'outer':0.,'r1':0.};m.charger_pose['outer']=Pose2D(4.6,-.45,0.)
    m.pending=['b'];m._locomotion_available=lambda *a:True;m._should_clear_channel=lambda *a:False
    m._should_clear_idle_g1=lambda *a:False;m._on_task_point=lambda *a:False
    parked=[];m._start_docking=lambda r:parked.append(r.id)
    m._update_idle_timers()
    assert 'outer' not in parked


def test_completed_wheel_keeps_its_clearance_parking_after_the_batch_finishes():
    m=batch();m.startup_clearance_robots=set();m.pending=[];m._now=lambda:100.
    m._idle_since={'outer':0.,'r1':0.};m.charger_pose['outer']=Pose2D(4.6,-.45,0.)
    m.robots['outer'].tasks_completed=3;m.live_pose['outer']=(-5.2,-.25,0.)
    m._locomotion_available=lambda *a:True;m._should_clear_channel=lambda *a:False
    m._should_clear_idle_g1=lambda *a:False;m._on_task_point=lambda *a:False
    parked=[];m._start_docking=lambda r:parked.append(r.id)
    m._update_idle_timers()
    assert 'outer' not in parked


def test_dispatch_path_rejects_an_occluded_peer_between_free_endpoints():
    import time
    from geometry_msgs.msg import PoseStamped
    from test_goal_geometry import scene
    m=make_manager();_,profiles=scene();m.types=profiles['robot_types']
    m.robots['r1'].type='g1_humanoid'
    m.robots['peer']=RobotState(id='peer',type='omni_mecanum_amr')
    m.live_pose={'r1':(-2.,4.22,0.),'peer':(7.,4.22,0.)}
    m.live_pose_stamps={n:time.monotonic() for n in m.robots}
    def point(x,y):
        p=PoseStamped();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;return p
    # Endpoint-only checks would miss the stationary vehicle at x=7.
    assert not m._path_clears_peers(m.robots['r1'],[point(-2.,4.22),point(11.,4.22)])
    assert m._path_clears_peers(m.robots['r1'],[point(-2.,4.22),point(0.,4.22)])
    m.live_pose_stamps['peer']-=1.
    assert not m._path_clears_peers(m.robots['r1'],[point(-2.,4.22),point(0.,4.22)])
