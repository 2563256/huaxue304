from itertools import combinations
from types import SimpleNamespace
import pytest
from hetero_fleet_scheduler.traffic import TrafficCoordinator

TYPES={'g1':'g1_humanoid','heavy':'heavy_diff_amr','light':'light_diff_amr','omni':'omni_mecanum_amr'}


@pytest.mark.parametrize('a,b', list(combinations(TYPES,2)))
@pytest.mark.parametrize('caller_is_first', [True,False])
def test_all_six_pairs_have_one_consistent_passing_order(a,b,caller_is_first):
    coordinator=TrafficCoordinator()
    robots={r:SimpleNamespace(type=TYPES[r]) for r in (a,b)}
    poses={a:(0.,0.,0.),b:(1.2,0.,3.14)}
    for r in poses:coordinator.observe(r,poses[r],10.)
    for r in poses:coordinator.observe(r,poses[r],18.1)
    first=a if caller_is_first else b
    request=coordinator.request(first,robots,poses,set(poses),18.1)
    if 'g1' in (a,b):
        assert request is None
        assert coordinator.request(b if first==a else a,robots,poses,set(poses),18.1) is None
        return
    assert request.winner==a and request.loser==b
    assert coordinator.request(b if first==a else a,robots,poses,set(poses),18.1) is request
    assert not request.parked.is_set() and not request.done.is_set()
    request.parked.set()
    assert not request.done.is_set()  # Parking must not release the loser.
    coordinator.finish(request,20.)
    assert request.done.is_set() and request.parked.is_set()
    assert coordinator.request(a,robots,poses,set(poses),20.1) is None


def test_a_moving_or_inactive_peer_is_not_evicted():
    c=TrafficCoordinator();robots={r:SimpleNamespace(type=TYPES[r]) for r in ('heavy','light')}
    poses={'heavy':(0,0,0),'light':(1.2,0,0)}
    for r in poses:c.observe(r,poses[r],0)
    c.observe('heavy',poses['heavy'],9);c.observe('light',(1.4,0,3.14),9)
    assert c.request('heavy',robots,poses,set(poses),9) is None
    c.observe('light',(1.4,0,3.14),18);c.observe('heavy',poses['heavy'],18)
    assert c.request('heavy',robots,poses,{'heavy'},18) is None


def test_sim_clock_reset_clears_motion_age():
    c=TrafficCoordinator();c.observe('g1',(0,0,0),100);c.observe('g1',(0,0,0),1)
    assert c.motion['g1'][1]==1

@pytest.mark.parametrize('a,b', list(combinations(TYPES,2)))
def test_head_on_pair_coordinates_before_either_robot_stops(a,b):
    c=TrafficCoordinator();robots={r:SimpleNamespace(type=TYPES[r]) for r in (a,b)}
    poses={a:(0.,0.,0.),b:(2.,0.,3.14)}
    for r in poses:c.observe(r,poses[r],10.)
    request=c.request(a,robots,poses,set(poses),10.)
    if 'g1' in (a,b):
        assert request is None
        return
    assert request is not None and request.winner==a
    assert not request.held.is_set()

@pytest.mark.parametrize('active,idle',[(a,b) for a in TYPES for b in TYPES if a!=b])
def test_idle_blocker_yields_even_when_it_has_higher_priority(active,idle):
    c=TrafficCoordinator();robots={r:SimpleNamespace(type=TYPES[r]) for r in (active,idle)}
    poses={active:(0.,0.,0.),idle:(1.2,0.,0.)}
    for r in poses:c.observe(r,poses[r],0.)
    for r in poses:c.observe(r,poses[r],2.1)
    request=c.request(active,robots,poses,{active},2.1,idle={idle})
    if 'g1' in (active,idle):
        assert request is None
        return
    assert request.winner==active and request.loser==idle and request.loser_idle


def test_reverse_motion_also_counts_as_approaching():
    c=TrafficCoordinator();robots={r:SimpleNamespace(type=TYPES[r]) for r in ('heavy','light')}
    initial={'heavy':(2.3,0,0),'light':(0,0,0)}
    for r,p in initial.items():c.observe(r,p,0.)
    poses={'heavy':(2.15,0,0),'light':(.15,0,0)}
    for r,p in poses.items():c.observe(r,p,.5)
    request=c.request('light',robots,poses,set(poses),.5)
    assert request is not None and request.winner=='heavy'


def test_goal_intent_coordinates_before_local_controller_turns_away():
    c=TrafficCoordinator();robots={r:SimpleNamespace(type=TYPES[r]) for r in ('heavy','light')}
    poses={'heavy':(3.5,0,0),'light':(0,0,0)}
    for r,p in poses.items():c.observe(r,p,0.)
    assert c.request('light',robots,poses,set(poses),0.) is None
    targets={'heavy':(-2,0),'light':(8,0)}
    request=c.request('light',robots,poses,set(poses),0.,targets=targets)
    assert request is not None and request.winner=='heavy'


def test_robots_in_different_corridors_are_not_coordinated_through_wall():
    c=TrafficCoordinator();robots={r:SimpleNamespace(type=TYPES[r]) for r in ('heavy','light')}
    poses={'heavy':(2,0,3.14),'light':(0,0,0)}
    for r,p in poses.items():c.observe(r,p,0.)
    assert c.request('light',robots,poses,set(poses),0.,corridor_clear=lambda a,b:False) is None


def test_distant_approaching_peer_does_not_cancel_navigation():
    c=TrafficCoordinator();robots={r:SimpleNamespace(type=TYPES[r]) for r in ('heavy','light')}
    poses={'heavy':(0,0,0),'light':(9,0,0)}
    for r,p in poses.items():c.observe(r,p,0.)
    request=c.request('heavy',robots,poses,set(poses),0.,targets={'heavy':(11,0),'light':(-2,0)},corridor_clear=lambda a,b:True)
    assert request is None
