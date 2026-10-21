"""Reservation waiting preserves navigation intent without stall recovery."""
import threading
import time
from types import SimpleNamespace
import rclpy
import pytest
from test_standoff import make_manager, place, ScriptedNav
from hetero_fleet_scheduler.task_model import Pose2D


def intents(manager):
    published = []
    manager._channel_intent_publishers = {'r1': SimpleNamespace(publish=lambda m: published.append(m.data))}
    manager._channel_counts = {}; manager._channel_lock = threading.Lock()
    return published


def test_nested_recovery_does_not_cancel_outer_reservation_intent():
    manager = make_manager(); published = intents(manager)
    manager._set_channel_active('r1', True)
    manager._set_channel_active('r1', True)
    manager._set_channel_active('r1', False)
    manager._publish_channel_intents()
    assert published == [True, True, True, True]
    manager._set_channel_active('r1', False)
    assert published[-1] is False


def test_queued_goal_is_cancelled_then_resumed_without_backing_up():
    owned = not rclpy.ok()
    if owned: rclpy.init()
    manager = make_manager(); place(manager, (0., 0., 0.))
    manager.stall_timeout_sec = .10; manager.channel_wait_timeout = 2.
    manager._channel_wait_states = {'r1': (True, time.monotonic())}
    client = ScriptedNav(manager, 'r1', ['wedged', 'drive'], (0., 0., 0.))
    manager.nav_clients = {'r1': client}
    published = intents(manager)
    manager._back_off = lambda *args, **kwargs: (_ for _ in ()).throw(AssertionError('Queue waiting must not back off'))
    def release():
        time.sleep(.25)
        manager._channel_wait_states['r1'] = (False, time.monotonic())
    updater = threading.Thread(target=release); updater.start()
    try:
        result = manager._navigate(manager.robots['r1'], Pose2D(2., 0., 0.), 'reserved')
        assert result[0]
        assert len(client.goals) == 2 and client.goals[0].canceled
        assert not manager.active_goal
        assert published == [True, False]
    finally:
        updater.join(timeout=1)
        if owned: rclpy.shutdown()


def test_reservation_enforcement_is_active_when_stall_watchdog_disabled():
    manager = make_manager(); place(manager, (0., 0., 0.))
    manager.stall_enabled = False
    manager._channel_wait_states = {'r1': (True, time.monotonic())}
    client = ScriptedNav(manager, 'r1', ['wedged'], (0., 0., 0.))
    manager.nav_clients = {'r1': client}
    handle, _ = manager._send_goal(manager.robots['r1'], Pose2D(2., 0., 0.), 'reserved')
    status, stalled, _ = manager._wait_for_result(manager.robots['r1'], Pose2D(2., 0., 0.), handle.get_result_async())
    assert status == 'channel_wait' and not stalled


def test_cancel_while_queued_removes_navigation_and_reservation_intent():
    owned = not rclpy.ok()
    if owned: rclpy.init()
    manager = make_manager(); place(manager, (0., 0., 0.))
    manager.channel_wait_timeout = 2.
    manager._channel_wait_states = {'r1': (True, time.monotonic())}
    client = ScriptedNav(manager, 'r1', ['wedged'], (0., 0., 0.))
    manager.nav_clients = {'r1': client}; published = intents(manager)
    cancel = threading.Event()
    updater = threading.Timer(.25, cancel.set); updater.start()
    try:
        result = manager._navigate(manager.robots['r1'], Pose2D(2., 0., 0.), 'reserved', cancel_when=cancel.is_set)
        assert not result[0] and client.goals[0].canceled
        assert published[-1] is False and not manager.active_goal
    finally:
        updater.join(timeout=1)
        if owned: rclpy.shutdown()


def test_channel_owner_leaves_between_jobs_before_accepting_more_work():
    from hetero_fleet_scheduler.task_model import RobotStatus, Task, TaskState
    manager = make_manager(); robot = manager.robots['r1']; robot.status = RobotStatus.IDLE
    place(manager, (2., 3., .4))
    manager._channel_snapshot = ({'groups': {'ch': {'owner':'r1','entered':True,'queue':['peer']}}}, time.monotonic())
    task = Task('next'); task.state = TaskState.PENDING
    manager.tasks = {'next': task}; manager.pending = ['next']
    manager.load_time = manager.unload_time = 3.
    assert manager._work_for(robot)
    assert manager._should_clear_channel('r1')
    # Queued work must not prevent clearing an occupied shared bottleneck.
    manager._idle_since['r1'] = manager._now() - 1.
    requested = []
    manager._start_docking = lambda r: requested.append(r.id)
    manager._update_idle_timers()
    assert requested == ['r1']
    # Once the body exits, the exception no longer delays task assignment.
    manager._channel_snapshot[0]['groups']['ch']['entered'] = False
    assert not manager._should_clear_channel('r1')


def test_g1_and_stale_status_never_trigger_exclusive_channel_eviction():
    manager = make_manager(); manager._channel_snapshot = (
        {'groups': {'ch': {'owner':'r1','entered':True,'queue':['peer']}}}, time.monotonic() - 2.)
    assert not manager._should_clear_channel('r1')
    manager._channel_snapshot = (manager._channel_snapshot[0], time.monotonic())
    manager.robots['r1'].type = 'g1_humanoid'
    assert not manager._should_clear_channel('r1')


@pytest.mark.parametrize('entered',[True,False])
@pytest.mark.parametrize('owner_status',['CHARGING','TO_PICKUP','TO_DROPOFF'])
def test_queue_retreat_waits_for_physical_exit_before_resuming_transport(entered, owner_status):
    from copy import copy
    from hetero_fleet_scheduler.task_model import RobotStatus
    owned = not rclpy.ok()
    if owned: rclpy.init()
    manager = make_manager(); place(manager, (1., 0., 0.))
    owner = copy(manager.robots['r1']); owner.id = 'owner'; owner.status = getattr(RobotStatus, owner_status)
    manager.robots['owner'] = owner
    group = {'owner': 'owner', 'entered': entered, 'queue': ['r1']}
    manager._channel_snapshot = ({'groups': {'ch': group}}, time.monotonic())
    manager.channel_wait_timeout = 2.
    manager._channel_wait_states = {'r1': (True, time.monotonic())}
    client = ScriptedNav(manager, 'r1', ['wedged', 'drive', 'drive'], (1., 0., 0.))
    manager.nav_clients = {'r1': client}; published = intents(manager)
    checks = []
    def release():
        time.sleep(.3)
        # A retreat plan is no longer queued, but the occupied owner must
        # still be allowed to leave before the original route is retried.
        manager._channel_wait_states['r1'] = (False, time.monotonic())
        group['queue'] = []
        time.sleep(.2)
        checks.append(len(client.goals))
        group.update(owner=None, entered=False)
        manager._channel_wait_states['r1'] = (False, time.monotonic())
    updater = threading.Thread(target=release); updater.start()
    try:
        ok, distance, _ = manager._navigate(manager.robots['r1'], Pose2D(2., 0., 0.), 'reserved')
        assert ok and distance >= 2.
        assert checks == [2]  # Original goal, then retreat; no early restart.
        assert len(client.goals) == 3 and client.goals[0].canceled
        assert client.goals[1].goal.pose.pose.position.x == manager.charger_pose['r1'].x
        assert client.goals[2].goal.pose.pose.position.x == 2.
        assert not manager._channel_retreating and published[-1] is False
    finally:
        updater.join(timeout=1)
        if owned: rclpy.shutdown()


def observed_exit_deadlock():
    """Actual v5 poses: G1 blocks the charger and the long retreat route."""
    from pathlib import Path
    import yaml
    from PIL import Image
    from hetero_fleet_scheduler.task_model import RobotState, RobotStatus
    root = Path(__file__).resolve().parents[3]
    profiles = yaml.safe_load((root/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    config = yaml.safe_load((root/'src/hetero_fleet_scheduler/config/channels.yaml').read_text())
    manager = make_manager()
    manager.types = profiles['robot_types']
    manager.robots = {r['name']: RobotState(id=r['name'], type=r['type']) for r in profiles['fleet']}
    manager.robots['omni_1'].status = RobotStatus.CHARGING
    manager.live_pose = {'light_1':(-.46,-.25,0.),'heavy_1':(9.624,-.43,0.),
                         'omni_1':(6.90,4.225,3.138),'g1_1':(5.24,-.67,.2)}
    meta_path = root/'src/hetero_fleet_bringup/maps/geomap.yaml'
    meta = yaml.safe_load(meta_path.read_text())
    pixels = Image.open(meta_path.parent/meta['image']).convert('L')
    manager._occupancy = {'px':pixels.load(),'w':pixels.width,'h':pixels.height,
                          'res':meta['resolution'],'ox':meta['origin'][0],
                          'oy':meta['origin'][1],'free_thresh':meta['free_thresh']}
    manager._channel_snapshot = ({'channels':config['channels'], 'clearance_margin':.15,
        'maneuvers':{k:config[k] for k in ('exit_waypoints','retreat_waypoints')},
        'groups':{'upper_loop':{'owner':'omni_1','entered':True,'queue':['light_1','heavy_1']}}},time.monotonic())
    return manager


def test_observed_exit_selects_resource_exit_before_blocked_charger():
    manager = observed_exit_deadlock()
    target = manager._channel_maneuver(manager.robots['omni_1'], 'exit_waypoints')
    assert (target.x,target.y,target.yaw) == (10.8,-.25,0.)
    # G1's real body is never excluded from destination filtering.
    manager.live_pose['g1_1'] = (10.8,-.25,0.)
    target = manager._channel_maneuver(manager.robots['omni_1'], 'exit_waypoints')
    assert target is None or target.x != 10.8


def test_observed_queue_uses_short_retreat_instead_of_crossing_g1():
    manager = observed_exit_deadlock()
    target = manager._channel_maneuver(manager.robots['heavy_1'], 'retreat_waypoints')
    assert (target.x,target.y,target.yaw) == (7.5,-.25,0.)
    # A peer occupying the retreat destination must invalidate it.
    manager.live_pose['g1_1'] = (7.5,-.25,0.)
    assert manager._channel_maneuver(manager.robots['heavy_1'], 'retreat_waypoints') is None


def test_exit_navigation_visits_exit_before_charger():
    manager = observed_exit_deadlock(); robot = manager.robots['omni_1']
    manager.charger_pose = {'omni_1':Pose2D(4.6,-.45,0.)}
    manager.robots = {**manager.robots}
    legs = []
    def navigate(robot,target,tag):
        legs.append((target.x,target.y,tag))
        manager.live_pose[robot.id] = (target.x,target.y,target.yaw)
        return True,1.,0.
    manager._navigate = navigate
    manager._apply_battery_drain = lambda *args: None
    manager._drive_home(robot)
    assert [(x,y) for x,y,_ in legs] == [(10.8,-.25),(4.6,-.45)]


def test_g1_settles_between_general_legs_without_adding_task_point_rotation():
    manager = make_manager(); robot = manager.robots['r1']; robot.type = 'g1_humanoid'
    manager.live_pose['r1'] = (1.1,.1,1.45)
    published = intents(manager); pauses = []
    manager._navigate_impl = lambda *args: (True,1.,.1)
    def dwell(robot, seconds):
        assert published[-1] is True
        pauses.append(seconds)
        manager.live_pose['r1'] = (1.12,.12,1.46)
        return True
    manager._task_point_dwell = dwell
    assert manager._navigate(robot,Pose2D(1.2,.2,1.57),'r1:leave_charger')[0]
    assert pauses == [3.] and robot.pose.yaw == 1.46
    assert published[-1] is False
    assert manager._navigate(robot,Pose2D(5.8,-.2,0.),'job:pickup')[0]
    assert pauses == [3.]  # Task service already supplies its own hold.


def test_finished_g1_clears_dropoff_when_next_pickup_is_reserved():
    from hetero_fleet_scheduler.task_model import RobotStatus, Task, TaskState
    manager = observed_exit_deadlock(); g1 = manager.robots['g1_1']
    g1.status = RobotStatus.IDLE; g1.tasks_completed = 1
    g1.capabilities = ['humanoid_transport']; g1.payload_capacity_kg = 5.
    manager.live_pose['g1_1'] = (10.9,4.28,-.1)
    manager.clear_task_point_radius_m = 1.2
    manager.idle_clearance_pose = {'g1_1':Pose2D(-6.,-.25,3.14159265359)}
    done = Task('done',dropoff=Pose2D(11.,4.22,0.)); done.state = TaskState.COMPLETED
    next_task = Task('next',pickup=Pose2D(-2.,4.22,0.),required_capability='humanoid_transport',payload_kg=2.)
    held = Task('heavy_pickup',pickup=Pose2D(-2.,4.22,0.))
    manager.tasks = {t.id:t for t in [done,next_task,held]}; manager.pending = ['next']
    heavy = manager.robots['heavy_1']; heavy.status = RobotStatus.TO_PICKUP
    heavy.current_task_id = held.id
    assert manager._work_for(g1) and manager._should_clear_idle_g1(g1)
    manager._idle_since['g1_1'] = manager._now()-10.
    requested=[]; manager._start_docking=lambda robot: requested.append(robot.id)
    manager.charger_pose['g1_1'] = Pose2D(1.2,-.8,0.)
    manager.charger_route = {'g1_1':[Pose2D(3.8,.2,3.14)]}
    manager._update_idle_timers()
    assert 'g1_1' in requested
    targets=[]
    def navigate(robot,target,tag):
        targets.append((target.x,target.y,target.yaw))
        manager.live_pose[robot.id] = (target.x,target.y,target.yaw)
        return True,1.,0.
    manager._navigate=navigate; manager._apply_battery_drain=lambda *args:None
    g1.status=RobotStatus.CHARGING
    manager._drive_home(g1)
    assert targets == [(-6.,-.25,3.14159265359)] and g1.status == RobotStatus.IDLE
    assert not manager._should_clear_idle_g1(g1)
    # Once the pickup is released, immediately available work takes precedence.
    manager.live_pose['g1_1'] = (10.9,4.28,-.1)
    heavy.status=RobotStatus.IDLE; heavy.current_task_id=''
    assert not manager._should_clear_idle_g1(g1)


def test_nav2_abort_near_general_exit_does_not_reverse_away_from_it():
    manager=make_manager(); place(manager,(0.,0.,0.))
    client=ScriptedNav(manager,'r1',['wedged'],(0.,0.,0.));manager.nav_clients={'r1':client}
    manager._wait_for_result=lambda *args,**kwargs:(6,False,.19)
    manager._back_off=lambda *args: (_ for _ in ()).throw(AssertionError('Already within the general arrival radius'))
    result=manager._navigate(manager.robots['r1'],Pose2D(.19,0.,0.),'r1:charge:0')
    assert result[0] and len(client.goals)==1 and client.goals[0].canceled


def test_center_queue_retreats_left_without_crossing_the_owners_return():
    manager=observed_exit_deadlock()
    target=manager._channel_maneuver(manager.robots['light_1'],'retreat_waypoints')
    assert (target.x,target.y)==(-2.2,-.25)


def test_exit_choice_accounts_for_the_queued_robots_retreat_destination():
    manager=observed_exit_deadlock()
    manager.live_pose['omni_1']=(-2.,4.22,3.14)
    target=manager._channel_maneuver(manager.robots['omni_1'],'exit_waypoints')
    # The west exit is nearest, but the light robot's safe retreat blocks it.
    assert (target.x,target.y)==(0.,-.25)


def test_rotated_vehicle_retreats_without_a_new_parking_heading():
    import math
    manager=observed_exit_deadlock()
    manager.live_pose.update(heavy_1=(2.21,-.35,math.pi), light_1=(-2.2,-.25,0.),
                             omni_1=(10.93,-.19,0.), g1_1=(-6.,-.25,math.pi))
    target=manager._channel_maneuver(manager.robots['heavy_1'],'retreat_waypoints')
    assert (target.x,target.y)==(6.1,-.25) and target.yaw==math.pi


def test_queue_refuge_preserves_an_assigned_lights_future_dropoff():
    from hetero_fleet_scheduler.task_model import Task,TaskState,RobotStatus
    manager=observed_exit_deadlock()
    manager.live_pose.update(heavy_1=(10.905,4.159,0.),light_1=(-6.903,-.239,0.),
                            omni_1=(10.92,-.275,0.),g1_1=(1.26,-1.08,0.))
    task=Task('light08',pickup=Pose2D(1.7,4.22),dropoff=Pose2D(8.1,-.43),state=TaskState.TO_PICKUP)
    manager.tasks={task.id:task};manager.robots['light_1'].current_task_id=task.id
    manager.robots['light_1'].status=RobotStatus.TO_PICKUP
    target=manager._channel_maneuver(manager.robots['omni_1'],'retreat_waypoints')
    assert (target.x,target.y)==(6.1,-.25)


def test_already_staged_vehicle_holds_position_instead_of_changing_bays():
    import math
    manager=observed_exit_deadlock()
    manager.live_pose['heavy_1']=(7.6,-.27,math.pi)
    target=manager._channel_maneuver(manager.robots['heavy_1'],'retreat_waypoints')
    assert (target.x,target.y,target.yaw)==manager.live_pose['heavy_1']


def test_finished_g1_clears_for_other_robots_even_after_its_own_jobs_end():
    from hetero_fleet_scheduler.task_model import RobotStatus,Task,TaskState
    manager=observed_exit_deadlock();g1=manager.robots['g1_1']
    g1.status=RobotStatus.IDLE;g1.tasks_completed=2
    manager.idle_clearance_pose={'g1_1':Pose2D(-6.,-.25,3.14)}
    manager.clear_task_point_radius_m=1.2
    manager.live_pose['g1_1']=(11.9,4.22,0.)
    done=Task('done',dropoff=Pose2D(11.,4.22,0.));done.state=TaskState.COMPLETED
    manager.tasks={'done':done};manager.pending=[]
    manager.robots['heavy_1'].status=RobotStatus.TO_PICKUP
    assert manager._should_clear_idle_g1(g1)
    manager.live_pose['g1_1']=(-6.,-.25,3.14)
    manager.charger_pose['g1_1']=Pose2D(1.2,-.8,0.)
    manager._idle_since['g1_1']=manager._now()-100.
    requested=[];manager._start_docking=lambda r:requested.append(r.id)
    manager._update_idle_timers()
    assert 'g1_1' not in requested


def test_queue_retreat_uses_the_reverse_friendly_behavior_tree():
    manager=make_manager();client=ScriptedNav(manager,'r1',['wedged'],(0.,0.,0.))
    manager.nav_clients={'r1':client}
    _,err=manager._send_goal(manager.robots['r1'],Pose2D(1.,0.,0.),'r1:channel_queue_retreat')
    assert err is None
    assert client.goals[0].goal.behavior_tree.endswith('/retreat_navigation.xml')


def test_retreating_waiter_remains_visible_when_its_refuge_path_clears_the_queue():
    manager = make_manager()
    manager._channel_snapshot = ({'groups': {'ch': {
        'owner': 'r1', 'entered': True, 'queue': []}}}, time.monotonic())
    manager._channel_yield_waits = {'waiting_robot': 'r1'}
    assert manager._should_clear_channel('r1')
    manager._channel_yield_waits.clear()
    assert not manager._should_clear_channel('r1')


def test_transport_heartbeat_covers_task_legs_and_dwell_but_exempts_g1():
    from hetero_fleet_scheduler.task_model import RobotStatus
    manager=make_manager();intents(manager);messages=[]
    manager._channel_transport_publishers={'r1':SimpleNamespace(publish=lambda m:messages.append(m.data))}
    r=manager.robots['r1'];r.current_task_id='task'
    for status in (RobotStatus.TO_PICKUP,RobotStatus.LOADING,RobotStatus.TO_DROPOFF):
        r.status=status;manager._publish_channel_intents()
    r.type='g1_humanoid';manager._publish_channel_intents()
    r.type='light_diff_amr';r.status=RobotStatus.IDLE;r.current_task_id='';manager._publish_channel_intents()
    assert messages==[True,True,True,False,False]


def test_waiter_retries_if_the_refuge_was_initially_unavailable(monkeypatch):
    from copy import copy
    from hetero_fleet_scheduler.task_model import RobotStatus
    import hetero_fleet_scheduler.fleet_manager as module
    manager=make_manager();place(manager,(0.,0.,0.));clock=[0.];calls=[]
    owner=copy(manager.robots['r1']);owner.id='owner';owner.status=RobotStatus.TO_PICKUP
    manager.robots['owner']=owner;group={'owner':'owner','entered':True,'queue':['r1']}
    manager._channel_snapshot=({'groups':{'ch':group}},time.monotonic())
    manager._channel_wait_states={'r1':(True,time.monotonic())};manager.channel_wait_timeout=30.
    manager._now=lambda:clock[0]
    monkeypatch.setattr(module.rclpy,'ok',lambda:True)
    monkeypatch.setattr(module,'time',SimpleNamespace(monotonic=time.monotonic,sleep=lambda dt:clock.__setitem__(0,clock[0]+dt)))
    def retreat(robot,owner,cancel_when):
        calls.append(clock[0])
        if len(calls)==2:
            manager.live_pose['r1']=(5.,5.,0.);group['owner']=None
            manager._channel_wait_states['r1']=(False,time.monotonic())
        return 0.
    manager._retreat_from_channel_queue=retreat
    assert manager._wait_channel_turn(manager.robots['r1'],None)[0]
    assert len(calls)==2 and calls[1]-calls[0]>=10.
    assert manager._channel_yield_waits=={}


def test_exit_selection_skips_blocked_and_approximate_planner_results():
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    from test_standoff import resolved, FakeGoalHandle
    manager=make_manager();candidates=[Pose2D(0.,-.25,0.),Pose2D(-3.,-.25,0.),Pose2D(10.8,-.25,0.)]
    manager._channel_maneuver=lambda robot,kind,excluded=(): next((p for p in candidates if p not in excluded),None)
    manager._make_goal=lambda target:SimpleNamespace(pose=PoseStamped())
    calls=[]
    class Planner:
        def wait_for_server(self,**kwargs):return True
        def send_goal_async(self,goal):
            calls.append(goal);path=Path();pose=PoseStamped()
            # An empty failure and then a successful but approximate endpoint
            # must both be rejected before selecting the real third exit.
            if len(calls)==2:pose.pose.position.x=-2.;pose.pose.position.y=-.25;path.poses=[pose]
            if len(calls)==3:pose.pose.position.x=10.8;pose.pose.position.y=-.25;path.poses=[pose]
            h=FakeGoalHandle('plan',None)
            h.result=resolved(SimpleNamespace(status=6 if len(calls)==1 else 4,result=SimpleNamespace(path=path)))
            return resolved(h)
    manager.planner_clients={'r1':Planner()}
    manager._path_clears_peers=lambda *args:True
    assert manager._reachable_channel_exit(manager.robots['r1'])==candidates[2]
    assert len(calls)==3 and all(c.planner_id=='GridBased' and not c.use_start for c in calls)


def test_exit_selection_does_not_accept_an_unavailable_planner():
    manager=make_manager();manager._channel_maneuver=lambda *args,**kwargs:Pose2D(0.,-.25,0.)
    manager.planner_clients={'r1':SimpleNamespace(wait_for_server=lambda **kwargs:False)}
    assert manager._reachable_channel_exit(manager.robots['r1']) is None


def test_sideways_queue_retreat_uses_a_feasible_travel_heading():
    import math
    manager=observed_exit_deadlock()
    manager.live_pose.update(heavy_1=(-.98047,-.3506,1.38),light_1=(.13,4.24,-3.01),
                             omni_1=(7.58,-.35,0.),g1_1=(4.83,-.5,-1.27))
    target=manager._channel_maneuver(manager.robots['heavy_1'],'retreat_waypoints')
    assert (target.x,target.y)==(-2.2,-.25)
    assert abs(target.yaw)<.2


def test_dynamic_exit_occupancy_cancels_and_selects_another_route():
    manager=make_manager();clock=[0.];calls=[];candidates=[Pose2D(10.8,-.25,0.),Pose2D(-3.,-.25,0.)]
    manager._now=lambda:clock[0]
    manager._reachable_channel_exit=lambda robot,excluded=(): next((p for p in candidates if p not in excluded),None)
    manager._exit_target_occupied=lambda robot,target:target==candidates[0]
    def navigate(robot,target,tag,cancel_when=None):
        calls.append(target)
        if target==candidates[0]:
            assert not cancel_when();clock[0]=2.1;assert cancel_when()
            return False,.2,.1
        assert not cancel_when()
        return True,3.,.5
    manager._navigate=navigate
    ok,distance,energy=manager._drive_to_clearance_exit(manager.robots['r1'])
    assert ok and calls==candidates and distance==3.2 and energy==.6


def test_exit_occupancy_includes_g1_but_requires_fresh_actual_pose():
    manager=observed_exit_deadlock();robot=manager.robots['omni_1']
    manager.live_pose.update(g1_1=(11.06,-.16,-.98),heavy_1=(-2.2,-.25,0.),light_1=(-6.,-.25,0.))
    manager.live_pose_stamps={name:time.monotonic() for name in manager.live_pose}
    assert manager._exit_target_occupied(robot,Pose2D(10.8,-.25,0.))
    assert not manager._exit_target_occupied(robot,Pose2D(0.,-.25,0.))
    manager.live_pose_stamps['g1_1']=time.monotonic()-1.
    assert not manager._exit_target_occupied(robot,Pose2D(10.8,-.25,0.))
