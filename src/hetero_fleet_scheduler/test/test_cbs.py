import itertools
import pytest
from hetero_fleet_scheduler.cbs import Candidate, Plan, conflicts, low_level, solve


def route(duration,resource='aisle',begin=0,end=None):
    return Candidate(duration,((resource,begin,end or duration),))


@pytest.mark.parametrize('mode',['cbs','eecbs'])
def test_spatial_detour_selected_instead_of_only_waiting(mode):
    short=route(8);detour=route(10,'detour')
    plans,stats=solve({'a':(short,detour),'b':(short,)},mode,budget=2)
    assert plans and not conflicts(plans)
    assert plans['a'].candidate==detour
    assert stats['cost_ticks']==18


@pytest.mark.parametrize('mode',['cbs','eecbs'])
def test_head_on_same_resource_is_serialized(mode):
    plans,stats=solve({'a':(route(3),),'b':(route(3),)},mode,budget=2)
    assert plans and not conflicts(plans)
    assert sorted(p.departure for p in plans.values())==[0,3]


def test_wait_body_is_not_allowed_to_disappear():
    c=Candidate(2,(('aisle',0,2),),frozenset({'start'}))
    assert low_level((c,),fixed=(('aisle',0,3),('start',1,2))) is None


def test_constraints_affect_waiting_and_route_choice():
    a=route(4);b=route(5,'detour')
    assert low_level((a,b),constraints=(('aisle',0,4),)).candidate==b


def test_budget_exhaustion_cannot_return_conflicting_incumbent():
    plans,stats=solve({'a':(route(8),),'b':(route(8),)},max_nodes=1)
    assert plans is None and stats['reason']=='node_budget'


@pytest.mark.parametrize('seed',range(15))
def test_cbs_matches_exhaustive_finite_catalogue(seed):
    cats={n:(route(2+(seed+i)%3,'x',i%2),route(4+(seed+i)%2,'y',0))
          for i,n in enumerate(('a','b'))}
    optimum=min(sum(p.finish for p in ps) for a,b in itertools.product(*cats.values())
                for da,db in itertools.product(range(10),repeat=2)
                if not conflicts(dict(zip(cats,ps:=(Plan(a,da),Plan(b,db))))))
    exact,stats=solve(cats,'cbs',budget=3)
    assert exact and stats['cost_ticks']==optimum
    approximate,stats=solve(cats,'eecbs',weight=1.2,budget=3)
    assert approximate and not conflicts(approximate)
    assert stats['cost_ticks']<=1.2*optimum


def test_invalid_bound():
    with pytest.raises(ValueError):solve({},weight=.5)


@pytest.mark.parametrize('mode',['cbs','eecbs'])
def test_long_nonpassing_corridor_uses_admissible_pair_reasoning(mode):
    plans,stats=solve({'a':(route(128),),'b':(route(128),)},mode,max_nodes=256,budget=2)
    assert plans and not conflicts(plans)
    assert stats['cost_ticks']<=1.2*384
    if mode=='cbs':assert stats['cost_ticks']==384
