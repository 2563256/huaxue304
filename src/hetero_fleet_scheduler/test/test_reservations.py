"""FIFO admission, physical release and fail-closed corridor protection."""
import math
from pathlib import Path
import pytest
import yaml
from hetero_fleet_scheduler.reservations import ChannelReservations, intersects


def route(start, end):
    n = max(1, math.ceil(math.dist(start[:2], end[:2]) / .05))
    turn = math.atan2(math.sin(end[2] - start[2]), math.cos(end[2] - start[2]))
    return [(start[0] + (end[0] - start[0]) * k / n,
             start[1] + (end[1] - start[1]) * k / n, start[2] + turn * k / n) for k in range(n + 1)]


def core(channels=None):
    return ChannelReservations({'channels': channels or [{'id': 'aisle', 'bounds': [0, -.6, 2, .6]}],
                                'preentry_lease_sec': 2.},
                               {'a': (.6, .4, .02), 'b': (.6, .4, .02)})


def ready(c, a=(-1., 0., 0.), b=(3., 0., math.pi), now=0.):
    c.observe('a', a, now); c.observe('b', b, now)
    c.set_active('a', True); c.set_active('b', True)
    c.set_path('a', route(a, (3., 0., 0.)))
    c.set_path('b', route(b, (-1., 0., math.pi)))


def test_opposing_robots_get_one_fifo_owner_before_entering():
    c = core(); ready(c)
    decisions, state = c.step(0., 0.)
    assert decisions['a']['allowed']
    assert decisions['b']['waiting'] and not decisions['b']['allowed']
    assert state['groups']['aisle']['owner'] == 'a'
    assert state['groups']['aisle']['queue'] == ['b']


def test_earlier_request_beats_robot_type_or_name_priority():
    c = core(); ready(c)
    c.set_active('a', False)
    c.step(0., 0.)
    c.set_active('a', True); c.set_path('a', route((-1., 0., 0.), (3., 0., 0.)))
    d, state = c.step(.1, .1)
    assert state['groups']['aisle']['owner'] == 'b'
    assert d['a']['waiting']


def test_body_and_release_margin_must_clear_even_after_goal_cancel():
    c = core(); ready(c); c.step(0, 0)
    c.observe('a', (1., 0., 0.), .1); c.observe('b', (3., 0., math.pi), .1)
    c.step(.1, .1)
    c.set_active('a', False)
    # Centre has exited, but the tail and release hysteresis are still inside.
    c.observe('a', (2.3, 0., 0.), .2); c.observe('b', (3., 0., math.pi), .2)
    d, s = c.step(100., .2)
    assert s['groups']['aisle']['owner'] == 'a'
    assert d['b']['waiting']
    c.observe('a', (2.7, 1., 0.), .3); c.observe('b', (3., 0., math.pi), .3)
    d, s = c.step(101., .3)
    assert s['groups']['aisle']['owner'] == 'b' and d['b']['allowed']


def test_preentry_timeout_moves_unentered_owner_behind_waiters():
    c = core(); ready(c); c.step(0, 0)
    c.observe('a', (-1., 0., 0.), .1); c.observe('b', (3., 0., math.pi), .1)
    d, s = c.step(2.1, .1)
    assert s['groups']['aisle']['owner'] == 'b'
    assert s['groups']['aisle']['queue'] == ['a'] and d['a']['waiting']


def test_cancelled_outside_request_is_removed():
    c = core(); ready(c); c.step(0, 0)
    c.set_active('b', False)
    _, s = c.step(.1, .1)
    assert s['groups']['aisle']['queue'] == []
    c.set_active('a', False)
    _, s = c.step(.2, .2)
    assert s['groups']['aisle']['owner'] is None


def test_replan_away_from_queued_channel_drops_future_request():
    c = core(); ready(c); c.step(0, 0)
    c.set_path('b', route((3., 0., math.pi), (5., 0., math.pi)))
    _, s = c.step(.1, .1)
    assert s['groups']['aisle']['queue'] == []


def test_stale_occupant_never_expires_into_another_permission():
    c = core(); ready(c); c.step(0, 0)
    c.observe('a', (1., 0., 0.), .1); c.observe('b', (3., 0., math.pi), .1)
    c.step(.1, .1)
    c.observe('b', (3., 0., math.pi), 10.)
    d, s = c.step(100., 10.)
    assert not any(r['allowed'] for r in d.values())
    assert s['groups']['aisle']['owner'] == 'a'
    assert s['groups']['aisle']['occupants'] == ['a']


def test_unknown_robot_pose_blocks_startup():
    c = core(); c.observe('a', (-1., 0., 0.), 0)
    c.set_active('a', True); c.set_path('a', route((-1., 0., 0.), (3., 0., 0.)))
    d, s = c.step(0, 0)
    assert not s['pose_ready'] and not d['a']['allowed']
    assert s['groups']['aisle']['owner'] is None


def test_incumbent_after_restart_owns_channel_without_a_live_action():
    c = core(); c.observe('a', (1., 0., 0.), 0); c.observe('b', (3., 0., math.pi), 0)
    c.set_active('b', True); c.set_path('b', route((3., 0., math.pi), (-1., 0., math.pi)))
    d, s = c.step(0, 0)
    assert s['groups']['aisle']['owner'] == 'a' and d['b']['waiting']


def test_multiple_incumbents_block_further_admission():
    c = core(); ready(c, a=(.2, 0, 0), b=(1.8, 0, math.pi))
    d, s = c.step(0, 0)
    assert s['groups']['aisle']['occupants'] == ['a', 'b']
    assert all(not v['allowed'] for v in d.values())
    assert all(v['reason'] == 'multiple_occupants' for v in d.values())


def test_adjacent_sections_merge_to_prevent_holding_one_and_waiting_for_next():
    c = core([{'id': 'left', 'bounds': [0, -.6, 2, .6]},
              {'id': 'right', 'bounds': [2.4, -.6, 4.4, .6]}])
    assert len(c.regions) == 1
    assert all(channel['group'] == 'left+right' for channel in c.channels)


def test_separated_channels_can_be_used_concurrently():
    c = core([{'id': 'left', 'bounds': [0, -.6, 2, .6]},
              {'id': 'right', 'bounds': [12, -.6, 14, .6]}])
    ready(c, a=(1., 0., 0.), b=(13., 0., math.pi))
    d, s = c.step(0, 0)
    assert len(c.regions) == 2 and all(v['allowed'] for v in d.values())
    assert s['groups']['left']['owner'] == 'a' and s['groups']['right']['owner'] == 'b'


def test_clock_reset_requires_fresh_poses_and_new_paths_but_keeps_occupancy():
    c = core(); ready(c, a=(1., 0., 0.)); c.step(100, 0)
    d, s = c.step(1, .1)
    assert not any(v['allowed'] for v in d.values())
    assert s['groups']['aisle']['owner'] == 'a'
    assert not c.paths
    c.observe('a', (5., 0., 0.), .2); c.observe('b', (3., 0., math.pi), .2)
    c.set_path('b', route((3., 0., math.pi), (-1., 0., math.pi)))
    d, s = c.step(2, .2)
    assert d['b']['allowed'] and s['groups']['aisle']['owner'] == 'b'


@pytest.mark.parametrize('command,pose', [((-.5, 0., 0.), (3., 0., 0.)),
                                         ((0., -.5, 0.), (1., 1.5, 0.))])
def test_reverse_and_strafe_recoveries_cannot_enter_unowned_channel(command, pose):
    c = core(); ready(c, a=(1., 0., 0.), b=pose); c.step(0, 0)
    c.set_path('b', route(pose, (pose[0] + 2, pose[1] + 2, pose[2])))
    c.set_command('b', command, .1)
    d, _ = c.step(.1, .1)
    assert d['b']['waiting'] and not d['b']['allowed']


def test_rotation_sweep_is_protected_when_center_stays_outside():
    c = ChannelReservations({'channels': [{'id': 'corner', 'bounds': [.9, .05, 1.0, .15]}]},
                            {'a': (1.6, 1.2, .01), 'b': (.4, .4, .01)})
    c.observe('a', (0., 0., 0.), 0); c.observe('b', (.95, .1, 0.), 0)
    c.set_active('a', True); c.set_path('a', [(0., 0., 0.)])
    c.set_command('a', (0., 0., .5), 0)
    d, _ = c.step(0, 0)
    assert d['a']['waiting'] and not d['a']['allowed']


def test_body_corners_overlap_a_region_even_if_center_does_not():
    assert intersects((0., 0., math.pi / 4), (1.6, 1.2), .02, [.9, .05, 1., .15])
    assert not intersects((0., 0., 0.), (1.6, 1.2), .02, [.9, .05, 1., .15])


def test_default_geomap_does_not_book_parked_chargers_forever():
    root = Path(__file__).resolve().parents[3]
    cfg = yaml.safe_load((root / 'src/hetero_fleet_scheduler/config/channels.yaml').read_text())
    profiles = yaml.safe_load((root / 'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    bodies = {r['name']: (*profiles['robot_types'][r['type']]['footprint'],
                          profiles['robot_types'][r['type']]['footprint_padding']) for r in profiles['fleet']}
    c = ChannelReservations(cfg, bodies)
    for r in profiles['fleet']:
        c.observe(r['name'], r['spawn'], 0.)
    _, s = c.step(0, 0)
    assert len(s['groups']) == 1 and 'upper_loop' in s['groups']
    assert all(not group['occupants'] and group['owner'] is None for group in s['groups'].values())


@pytest.mark.parametrize('config', [
    {'stop_distance_m': 4.}, {'preentry_lease_sec': 0},
    {'channels': [{'id': 'bad', 'bounds': [2, 0, 1, 1]}]},
    {'channels': [{'id': 'bad', 'bounds': [0, 0, 1, 1], 'capacity': 2}]},
])
def test_invalid_channel_configuration_is_rejected(config):
    base = {'channels': [{'id': 'aisle', 'bounds': [0, 0, 1, 1]}]}; base.update(config)
    with pytest.raises(ValueError):
        ChannelReservations(base, {'a': (.6, .4, .02)})


def test_preentry_owner_replanning_away_releases_without_waiting_for_lease():
    c = core(); ready(c); c.step(0, 0)
    c.set_path('a', route((-1., 0., 0.), (-4., 0., 0.)))
    d, s = c.step(.1, .1)
    assert s['groups']['aisle']['owner'] == 'b' and d['b']['allowed']


def test_exit_hysteresis_does_not_adopt_a_nearby_outside_robot():
    c = core(); ready(c, a=(1., 0., 0.), b=(-.43, 0., 0.))
    c.set_path('b', route((-.43, 0., 0.), (1., 0., 0.)))
    d, s = c.step(0, 0)
    assert s['groups']['aisle']['occupants'] == ['a']
    assert d['a']['allowed'] and d['b']['waiting']


def test_disabled_reservations_do_not_leave_phantom_tokens():
    c = core(); ready(c); c.step(0, 0)
    c.enabled = False
    d, s = c.step(1, 1)
    assert all(v['allowed'] and not v['waiting'] for v in d.values())
    assert not c.tokens and not c.pending


def test_one_transport_keeps_its_entered_token_across_a_forecourt_pickup():
    c=core();ready(c,b=(4.,0.,math.pi));c.set_transport_hold('a',True)
    c.step(0.,0.);c.observe('a',(1.,0.,0.),.1);c.observe('b',(4.,0.,math.pi),.1);c.step(.1,.1)
    # The body is outside for a loading dwell, but its next leg re-enters.
    c.observe('a',(3.,0.,0.),.2);c.observe('b',(4.,0.,math.pi),.2);c.set_active('a',False)
    _,state=c.step(10.,.2)
    assert state['groups']['aisle']['owner']=='a' and state['groups']['aisle']['transport_hold']
    c.set_active('a',True);c.set_path('a',route((3.,0.,0.),(1.,0.,0.)))
    c.observe('a',(1.,0.,0.),.3);c.observe('b',(4.,0.,math.pi),.3);c.step(11.,.3)
    # Completion cannot release a body still physically inside.
    c.set_transport_hold('a',False);c.set_active('a',False)
    _,state=c.step(12.,.3);assert state['groups']['aisle']['owner']=='a'
    c.observe('a',(3.,0.,0.),.4);c.observe('b',(4.,0.,math.pi),.4)
    _,state=c.step(13.,.4);assert state['groups']['aisle']['owner']=='b'


def test_transport_hold_does_not_keep_an_unentered_preentry_lease():
    c=core();ready(c);c.set_transport_hold('a',True);c.step(0.,0.)
    c.observe('a',(-1.,0.,0.),.1);c.observe('b',(3.,0.,math.pi),.1)
    _,state=c.step(3.,.1)
    assert state['groups']['aisle']['owner']=='b'


def test_g1_exemption_ignores_transport_holds():
    c=ChannelReservations({'reservation_exempt_robots':['g1'],
        'channels':[{'id':'aisle','bounds':[0.,-.6,2.,.6]}]},
        {'g1':(.6,.52,.15),'wheel':(.6,.4,.02)})
    c.set_transport_hold('g1',True)
    assert not c.transport_holds['g1']
