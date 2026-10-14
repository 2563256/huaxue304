"""Exercise the real navigation/parking handshake for every robot pairing."""
from itertools import combinations
import math
import threading
import time
import pytest
import rclpy
from test_standoff import make_manager, ScriptedNav
from hetero_fleet_scheduler.task_model import RobotState, Pose2D, RobotStatus
from hetero_fleet_scheduler.traffic import TrafficCoordinator

TYPES = ['g1_humanoid', 'heavy_diff_amr', 'light_diff_amr', 'omni_mecanum_amr']

@pytest.mark.parametrize('winner_type,loser_type', list(combinations(TYPES, 2)))
def test_blocked_pair_parks_one_robot_and_resumes_both(winner_type, loser_type):
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    m = make_manager(return_to_charger=False)
    m.robots = {name: RobotState(id=name, type=typ, pose=Pose2D(*pose),
                                max_speed=.4, battery_capacity_wh=480.)
                for name, typ, pose in [('winner', winner_type, (0., 0., 0.)),
                                        ('loser', loser_type, (1.2, 0., math.pi))]}
    m.traffic = TrafficCoordinator(stationary_sec=.15)
    m._recovery_lock = threading.Lock()
    m.slot_clearance = {'winner': .5, 'loser': .5}
    m.charger_pose = {'loser': Pose2D(4., 2., math.pi)}
    m.stall_timeout_sec = 2.
    m.nav_clients = {name: ScriptedNav(m, name, ['wedged', 'drive', 'drive'],
                                      (r.pose.x, r.pose.y, r.pose.yaw))
                     for name, r in m.robots.items()}
    # Model a safe native BackUp; the test exercises the subsequent real
    # _yield_to_peer -> _navigate parking and resumed-goal handshake.
    def reverse(r):
        x, y, yaw = m.live_pose[r.id]
        m.live_pose[r.id] = (x-.55*math.cos(yaw), y-.55*math.sin(yaw), yaw)
        return .55
    m._back_off = reverse
    results = {}
    workers = [threading.Thread(target=lambda name=name: results.__setitem__(
        name, m._navigate(m.robots[name], Pose2D(5. if name=='winner' else -2., 0.,
                                               0. if name=='winner' else math.pi), 'pair_test')),
        daemon=True) for name in m.robots]
    try:
        for worker in workers: worker.start()
        for worker in workers: worker.join(timeout=6.)
        assert all(not worker.is_alive() for worker in workers)
        assert results['winner'][0] and results['loser'][0]
        assert m.nav_clients['winner'].goals[0].canceled
        assert m.nav_clients['loser'].goals[0].canceled
        assert len(m.nav_clients['winner'].goals) == 2
        assert len(m.nav_clients['loser'].goals) == 3  # pullout, then original goal
        assert m.traffic.encounter is None
        assert not m.active_goal
    finally:
        if m.traffic.encounter is not None:
            m.traffic.finish(m.traffic.encounter, time.monotonic())
        if owned_context:
            rclpy.shutdown()


def test_pullout_fits_actual_north_corridor_instead_of_crossing_wall():
    from pathlib import Path
    from PIL import Image
    import yaml
    from hetero_fleet_scheduler.traffic import Encounter
    m = make_manager(return_to_charger=False)
    root = Path(__file__).resolve().parents[3]
    map_file = root/'src/hetero_fleet_bringup/maps/geomap.yaml'
    meta = yaml.safe_load(map_file.read_text())
    image = Image.open(map_file.parent/meta['image']).convert('L')
    m._occupancy = {'px':image.load(), 'w':image.width, 'h':image.height,
                    'res':meta['resolution'], 'ox':meta['origin'][0],
                    'oy':meta['origin'][1], 'negate':meta.get('negate',0)}
    m.traffic = TrafficCoordinator()
    m._recovery_lock = threading.Lock()
    m.live_pose = {'r1': (7.25, 4.22, math.pi), 'g1': (5.5, 4.22, 0.)}
    m.slot_clearance = {'r1': math.hypot(.6888,.5412)/2}
    m._back_off = lambda r: 0.
    chosen = []
    m._navigate = lambda r, target, *args, **kwargs: (chosen.append(target) or False, 0., 0.)
    request = Encounter('g1','r1',m.live_pose['g1'],m._now()); request.held.set()
    m._yield_to_peer(m.robots['r1'], request)
    assert len(chosen) == 1
    assert chosen[0] != m.charger_pose['r1']
    assert m._slot_is_free(chosen[0].x, chosen[0].y, m.slot_clearance['r1'])
    assert abs(chosen[0].y-4.22) <= .51
    assert request.done.is_set()


def test_idle_g1_parks_forward_so_active_light_can_pass():
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    m = make_manager(return_to_charger=False)
    m._now = lambda: time.monotonic()*10.
    m.stall_timeout_sec = 20.
    m.traffic = TrafficCoordinator()
    m._recovery_lock = threading.Lock()
    m.robots = {'light':RobotState(id='light',type='light_diff_amr',pose=Pose2D(),
                                  status=RobotStatus.TO_PICKUP,
                                  max_speed=.4,battery_capacity_wh=480.),
                'g1':RobotState(id='g1',type='g1_humanoid',pose=Pose2D(1.2,0,0),
                               max_speed=.4,battery_capacity_wh=480.)}
    m.charger_pose = {'g1':Pose2D(-5,-5,0)}
    m.slot_clearance = {'light':.44,'g1':.4}
    m.nav_clients = {'light':ScriptedNav(m,'light',['wedged','drive'],(0,0,0)),
                     'g1':ScriptedNav(m,'g1',['drive'],(1.2,0,0))}
    def no_blind_reverse(*args):
        raise AssertionError('An idle blocker must search forward space too')
    m._back_off = no_blind_reverse
    stopped = threading.Event()
    def observe_idle():
        while not stopped.is_set():
            m.traffic.observe('g1',m.live_pose['g1'],m._now())
            time.sleep(.02)
    updater = threading.Thread(target=observe_idle,daemon=True);updater.start()
    try:
        result=m._navigate(m.robots['light'],Pose2D(4,0,0),'idle_pair')
        limit=time.monotonic()+3
        while m.traffic.encounter is not None and time.monotonic()<limit:time.sleep(.02)
        assert result[0]
        assert len(m.nav_clients['g1'].goals)==1
        refuge=m.nav_clients['g1'].goals[0].goal.pose.pose.position
        assert refuge.x > 1.2
        assert m.traffic.encounter is None
        assert m.robots['g1'].status.value=='IDLE'
        assert m.nav_clients['light'].goals[0].canceled
    finally:
        stopped.set();updater.join(timeout=1)
        if m.traffic.encounter is not None:m.traffic.finish(m.traffic.encounter,m._now())
        if owned_context:rclpy.shutdown()


def test_reversing_robot_pulls_forward_away_from_peer_behind_it():
    from hetero_fleet_scheduler.traffic import Encounter
    m=make_manager(return_to_charger=False)
    m.traffic=TrafficCoordinator();m._recovery_lock=threading.Lock()
    m.live_pose={'r1':(1.2,0,0),'g1':(0,0,0)}
    def unsafe_reverse(*args):raise AssertionError('BackUp would move toward the peer')
    m._back_off=unsafe_reverse
    chosen=[]
    m._navigate=lambda r,p,*args,**kwargs:(chosen.append(p) or False,0.,0.)
    request=Encounter('g1','r1',(0,0,0),m._now());request.held.set()
    m._yield_to_peer(m.robots['r1'],request)
    assert chosen[0].x > 1.2


def test_loser_stays_parked_until_peer_actually_passes_refuge():
    from hetero_fleet_scheduler.traffic import Encounter
    owned_context = not rclpy.ok()
    if owned_context:rclpy.init()
    m=make_manager(return_to_charger=False)
    m.traffic=TrafficCoordinator();m._recovery_lock=threading.Lock()
    m.live_pose={'r1':(8,0,math.pi),'winner':(2.1,0,0)}
    m.active_goal={'winner':object()}
    m._back_off=lambda *args:0.
    def park(r,p,*args,**kwargs):
        m.live_pose[r.id]=(p.x,p.y,p.yaw)
        return True,0.,0.
    m._navigate=park
    request=Encounter('winner','r1',(0,0,0),m._now(),loser_origin=(8,0,math.pi))
    request.held.set();request.resumed.set()
    worker=threading.Thread(target=m._yield_to_peer,args=(m.robots['r1'],request),daemon=True)
    try:
        worker.start();time.sleep(.2)
        assert request.parked.is_set()
        assert worker.is_alive() and not request.done.is_set()
        parked=m.live_pose['r1'];m.live_pose['winner']=(parked[0]+1.3,0,0)
        worker.join(timeout=1)
        assert not worker.is_alive() and request.done.is_set()
    finally:
        m.traffic.finish(request,m._now());worker.join(timeout=1)
        if owned_context:rclpy.shutdown()


def test_no_nearby_pullout_releases_hold_without_navigating_to_charger():
    from hetero_fleet_scheduler.traffic import Encounter
    m = make_manager(return_to_charger=False)
    m.traffic = TrafficCoordinator()
    m._recovery_lock = threading.Lock()
    m.live_pose = {'r1': (1.2, 0., math.pi), 'winner': (0., 0., 0.)}
    m._back_off = lambda *args: 0.
    m._slot_is_free = lambda *args: False
    m._navigate = lambda *args, **kwargs: pytest.fail('No safe pullout must not send a charger goal')
    request = Encounter('winner', 'r1', (0., 0., 0.), m._now())
    request.held.set()
    m.traffic.encounter = request
    assert m._yield_to_peer(m.robots['r1'], request) == 0.
    assert request.done.is_set() and m.traffic.encounter is None
