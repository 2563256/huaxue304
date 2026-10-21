"""Temporal interval search, priority branches, and physical execution guard."""
import math
import pytest
from hetero_fleet_scheduler.sipp_pbs import Edge, Route, conflict, pbs, sipp, topology
from hetero_fleet_scheduler.reservations import ChannelReservations, Token


def test_sipp_jumps_to_end_of_large_blocked_interval_without_time_steps():
    plan = sipp(Route((Edge(2., frozenset({'channel'})),)), {'channel': [(0., 1e6)]})
    assert plan.departures == (1e6,)
    assert plan.finish == 1e6 + 2.


def test_sipp_cannot_wait_through_unsafe_origin_interval():
    route = Route((Edge(2., frozenset({'edge'})),), (frozenset({'origin'}), frozenset()))
    assert sipp(route, {'origin': [(1., 10.)], 'edge': [(0., 5.)]}) is None


def test_sipp_searches_later_destination_safe_interval():
    route = Route((Edge(2.), Edge(1., frozenset({'edge'}))),
                  (frozenset(), frozenset({'junction'}), frozenset()))
    plan = sipp(route, {'junction': [(3., 10.)], 'edge': [(0., 8.)]}, headway=.1)
    assert plan.arrivals == (0., 10., 11.)
    assert plan.departures == (8., 10.)


def test_sipp_edge_headway_and_touching_intervals():
    route = Route((Edge(2., frozenset({'ch'})),))
    plan = sipp(route, {'ch': [(2.2, 4.), (4., 6.)]}, headway=.5)
    assert plan.departures == (6.,)
    assert plan.reservations == (('ch', 6., 8.5),)


def test_pbs_branches_away_from_long_job_first_and_reduces_sum_arrival():
    routes = {'long': Route((Edge(20., frozenset({'ch'})),)),
              'short': Route((Edge(2., frozenset({'ch'})),))}
    plans, edges, stats = pbs(routes, budget=1.)
    assert conflict(plans) is None
    assert ('short', 'long') in edges
    assert plans['short'].finish == 2.
    assert plans['long'].finish == 22.5
    # FIFO sum = 20 + 22.5; PBS sum = 2 + 22.5.
    assert sum(p.finish for p in plans.values()) == 24.5
    assert stats['expanded'] >= 3


def test_pbs_retains_concurrency_for_independent_resources():
    routes = {'a': Route((Edge(4., frozenset({'left'})),)),
              'b': Route((Edge(4., frozenset({'right'})),))}
    plans, edges, _ = pbs(routes, budget=1.)
    assert plans['a'].finish == plans['b'].finish == 4.
    assert edges == () and conflict(plans) is None


def test_pbs_respects_hard_aging_priority_and_transitive_ancestors():
    routes = {n: Route((Edge(t, frozenset({'ch'})),)) for n, t in [('a', 5.), ('b', 1.), ('c', 1.)]}
    plans, edges, _ = pbs(routes, [('a', 'b'), ('b', 'c')], budget=1.)
    assert conflict(plans) is None
    assert plans['a'].finish < plans['b'].finish < plans['c'].finish
    assert topology(routes, [('a', 'b'), ('b', 'c')])[1]['c'] == {'a', 'b'}
    assert pbs(routes, [('a', 'b'), ('b', 'a')])[0] is None


def test_bounded_pbs_returns_feasible_seed_or_no_advice():
    routes = {str(i): Route((Edge(i + 1., frozenset({'ch'})),)) for i in range(4)}
    plans, _, stats = pbs(routes, max_nodes=1, budget=1.)
    assert plans is not None and conflict(plans) is None and stats['budget_exhausted']
    plans, _, stats = pbs(routes, budget=1e-15)
    assert plans is None and stats['budget_exhausted']


def optimized():
    return ChannelReservations({'scheduling': 'sipp_pbs', 'pbs_budget_sec': .1,
                                'channels': [{'id': 'ch', 'bounds': [0., -.5, 5., .5]}]},
                               {'long': (.4, .4, .02), 'short': (.4, .4, .02)})


def prepare(c, now=0.):
    c.observe('long', (-1., 0., 0.), now)
    c.observe('short', (6., 0., math.pi), now)
    c.set_active('long', True); c.set_active('short', True)
    c.set_path('long', [(-1., 0., 0.), (6., 0., 0.)])
    c.set_path('short', [(6., 0., math.pi), (4., 0., math.pi), (6., 0., math.pi)])


def test_runtime_uses_short_first_advice_but_never_expires_an_incumbent():
    c = optimized(); prepare(c)
    decisions, state = c.step(0., 0.)
    assert state['groups']['ch']['owner'] == 'short'
    assert decisions['long']['waiting']
    assert state['scheduler']['windows']
    c.observe('short', (4., 0., math.pi), .1)
    c.observe('long', (-1., 0., 0.), .1)
    c.step(.1, .1)
    c.set_active('short', False)
    c.observe('short', (4., 0., math.pi), 100.)
    c.observe('long', (-1., 0., 0.), 100.)
    decisions, state = c.step(100., 100.)
    assert state['groups']['ch']['owner'] == 'short'
    assert not decisions['long']['allowed']


def test_runtime_cache_and_replan_cancel_invalidate_temporal_advice():
    c = optimized(); prepare(c); c.step(0., 0.)
    compiled = c.compiled['long']
    c.step(.1, .1)
    assert c.compiled['long'] is compiled
    c.set_path('short', [(6., 0., math.pi), (8., 0., math.pi)])
    d, state = c.step(.2, .2)
    assert state['groups']['ch']['owner'] == 'long' and d['long']['allowed']
    c.step(-1., .3)
    assert not c.compiled and not c.paths and not c.poses


def test_runtime_starvation_guard_gives_old_request_hard_priority():
    c = optimized(); prepare(c)
    c.tokens['ch'] = Token('short', 0.)
    c.step(0., 0.)
    # Short cancels then requeues behind the old, longer request.
    c.set_active('short', False); c.tokens.clear()
    c.set_active('short', True)
    c.set_path('short', [(6., 0., math.pi), (4., 0., math.pi), (6., 0., math.pi)])
    c.observe('long', (-1., 0., 0.), 61.); c.observe('short', (6., 0., math.pi), 61.)
    _, state = c.step(61., 61.)
    assert state['groups']['ch']['owner'] == 'long'


@pytest.mark.parametrize('config', [{'scheduling': 'bad'}, {'pbs_budget_sec': 0.},
                                    {'reservation_headway_sec': float('nan')}, {'pbs_max_nodes': 0},
                                    {'robot_motion_limits': {'long': [0., 1.]}}])
def test_invalid_temporal_configuration_is_rejected(config):
    config['channels'] = [{'id': 'ch', 'bounds': [0., 0., 1., 1.]}]
    with pytest.raises(ValueError):
        ChannelReservations(config, {'long': (.4, .4, .02)})


def test_pbs_accounts_for_physical_incumbents_as_fixed_calendar():
    routes = {'a': Route((Edge(2., frozenset({'ch'})),)),
              'b': Route((Edge(1., frozenset({'ch'})),))}
    plans, _, _ = pbs(routes, budget=1., fixed_calendar={'ch': [(0., 10.)]})
    assert min(p.departures[0] for p in plans.values()) == 10.
    assert conflict(plans) is None
    plans, _, _ = pbs(routes, budget=1., fixed_calendar={'ch': [(0., math.inf)]})
    assert plans is None
