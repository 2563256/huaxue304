"""Real solver checks: assignment, sequencing, resources and rolling commits."""
import math
import pytest
from hetero_fleet_scheduler.joint_schedule import JointScheduler, Leg
from hetero_fleet_scheduler.rolling_schedule import RollingSchedule
from hetero_fleet_scheduler.task_model import RobotState, Pose2D, Task
from hetero_fleet_scheduler.reservations import ChannelReservations


def robot(name='a', capabilities=None):
    return RobotState(name, max_speed=1., payload_capacity_kg=10.,
                      capabilities=capabilities or [], battery_capacity_wh=1000.)


def task(name='t', pickup=1., dropoff=3., capability=''):
    return Task(name,pickup=Pose2D(pickup,0),dropoff=Pose2D(dropoff,0),
                required_capability=capability,payload_kg=1.)


def leg(r,a,b):
    d=abs(a.x-b.x)
    return Leg(max(.1,d),d)


def solver(estimate=leg,**kw):
    return JointScheduler(estimate,budget=2.,service=0.,**kw)


def test_parallel_assignment_beats_one_robot_and_honors_capability():
    plan=solver().solve([task('x',capability='x'),task('y',capability='y')],
                        [robot('a',['x']),robot('b',['y'])],100.)
    assert plan.status=='OPTIMAL'
    assert {r['task_id']:r['robot_id'] for r in plan.rows}=={'x':'a','y':'b'}
    assert max(r['end'] for r in plan.rows)==103.


def test_sequence_dependent_reposition_is_included():
    plan=solver().solve([task('near',1,2),task('far',10,11)],[robot()],0.)
    assert plan.status=='OPTIMAL'
    assert [r['task_id'] for r in plan.rows]==['near','far']
    assert plan.rows[-1]['end']==11.
    assert plan.rows[1]['start']>=plan.rows[0]['end']


def test_resource_conflict_serializes_different_robots():
    def shared(r,a,b):
        v=leg(r,a,b)
        return Leg(v.seconds,v.distance,frozenset(['bottleneck']))
    p=solver(shared).solve([task('x',capability='x'),task('y',capability='y')],
                          [robot('a',['x']),robot('b',['y'])],0.)
    assert p.status=='OPTIMAL'
    a,b=p.rows
    assert a['end']<=b['start']
    assert b['end']==6.


def test_busy_robot_release_and_traffic_delay_constrain_start():
    def shared(r,a,b): return Leg(1.,1.,frozenset(['g']))
    p=solver(shared).solve([task()],[robot()],10.,{'a':5.},{('a','g'):30.})
    assert p.rows[0]['start']>=30.


def test_unknown_route_cannot_be_replaced_by_euclidean_fallback():
    p=solver(lambda *args:None).solve([task()],[robot()],0.)
    assert p.status=='EMPTY' and p.unscheduled==['t'] and not p.rows


def test_no_capability_or_payload_never_dispatched():
    t=task(capability='heavy');t.payload_kg=100
    p=solver().solve([t],[robot()],0.)
    assert not p.rows and p.unscheduled==['t']


def test_cumulative_battery_constraint_is_hard():
    r=robot();r.battery_capacity_wh=1.
    p=solver().solve([task()],[r],0.)
    assert not p.rows and p.unscheduled==['t']


def test_too_short_horizon_holds_instead_of_violating_constraints():
    p=solver(horizon=1.).solve([task()],[robot()],0.)
    assert not p.rows and p.unscheduled==['t']


def test_task_cap_exposes_unplanned_tail():
    p=solver(max_tasks=1).solve([task('first'),task('second')],[robot()],0.)
    assert len(p.rows)==1 and p.unscheduled==['second']


def test_rolling_only_commits_near_term_head_and_discards_clock_future():
    rolling=RollingSchedule(solver(),period=5.,commit=1.)
    rolling.update(10.,'k',[task('near',1,2),task('far',10,11)],[robot()])
    assert [r['task_id'] for r in rolling.heads(10.)]==['near']
    assert rolling.heads(9.)==[] and rolling.heads(17.)==[]
    assert not rolling.due(11.,'k') and rolling.due(15.,'k')
    assert rolling.due(9.,'k') and rolling.due(11.,'changed')
    rolling.invalidate('traffic_delay'); assert rolling.due(11.,'k')


def core(channels=None):
    return ChannelReservations({'channels': channels or [{'id':'g','bounds':[0,0,2,2]}],
                                'scheduling':'sipp_pbs','schedule_hint_timeout_sec':1.},
                               {'a':(.6,.4,.02),'b':(.6,.4,.02)})


def hint(start=0.):
    return {'epoch':0.,'revision':1,'rows':[{'robot_id':'b','start':start,'end':start+10.}]}


def test_hint_is_advisory_and_never_grants_a_token():
    c=core();c.set_schedule_hint(hint(),0.)
    assert c.task_hints and not c.tokens
    # b's urgency must not remove physical occupancy of a.
    for n,p in [('a',(1.,1.,0.)),('b',(-2.,1.,0.))]: c.observe(n,p,0.)
    _,state=c.step(0.,0.)
    assert state['groups']['g']['owner']=='a'


def test_stale_hints_expire_even_if_simulation_pauses():
    c=core();c.set_schedule_hint(hint(),0.)
    c.step(0.,2.)
    assert not c.task_hints


@pytest.mark.parametrize('value',[None,[],{'epoch':float('nan'),'revision':1,'rows':[]},
    {'epoch':0,'revision':1,'rows':[{'robot_id':'unknown','start':0,'end':1}]}])
def test_invalid_hints_are_rejected(value):
    with pytest.raises(ValueError): core().set_schedule_hint(value,0.)


def test_spatially_close_sections_remain_atomic_but_far_sections_independent():
    c=core([{'id':'left','bounds':[0,0,2,2]},
            {'id':'close','bounds':[2.1,0,4,2]},
            {'id':'far','bounds':[30,0,32,2]}])
    assert len(c.regions)==2
    assert set(c.regions)=={'close+left','far'}


def test_short_horizon_still_admits_feasible_jobs_without_waiting_forever():
    p=solver(horizon=4.).solve([task('fits',1,2),task('too_far',100,110)],[robot()],0.)
    assert [r['task_id'] for r in p.rows]==['fits']
    assert p.unscheduled==['too_far']


def test_failed_robot_cannot_win_assignment():
    from hetero_fleet_scheduler.task_model import RobotStatus
    r=robot(); r.status=RobotStatus.FAILED
    assert not solver().solve([task()],[r],0.).rows


def test_schedule_message_keeps_solver_and_replan_reasons_separate():
    rolling=RollingSchedule(solver())
    rolling.update(10.,'k',[task()],[robot()])
    payload=rolling.hints()
    assert payload['replan_reason']=='startup' and 'reason' in payload
    assert payload['rows'] and payload['revision']==1


def test_simulation_reset_accepts_new_epoch_hints():
    c=core(); c.step(100.,0.)
    c.set_schedule_hint({'epoch':100.,'revision':5,'rows':[{'robot_id':'a','start':100.,'end':110.}]},0.)
    c.step(1.,1.)
    assert not c.task_hints and c.hint_revision==-1
    c.set_schedule_hint({'epoch':1.,'revision':6,'rows':[{'robot_id':'a','start':1.,'end':10.}]},1.)
    assert c.task_hints and c.hint_epoch==1.


def test_physical_planner_veto_excludes_a_robot_task_pair():
    p=solver().solve([task()],[robot('a'),robot('b')],0.,forbidden_pairs={('a','t')})
    assert len(p.rows)==1 and p.rows[0]['robot_id']=='b'


def test_all_real_routes_blocked_stay_unscheduled():
    p=solver().solve([task()],[robot('a')],0.,forbidden_pairs={('a','t')})
    assert not p.rows and p.unscheduled==['t']
