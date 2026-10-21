import math
from hetero_fleet_scheduler.mapf_router import compile_route
from hetero_fleet_scheduler.cbs import solve,conflicts


def test_rotation_reserves_swept_corner_and_cannot_exchange_bodies():
    a=compile_route([(0,0,0),(0,0,math.pi/2)],(2.,.6,.03),(1.,1.),{},grid=.2)
    assert 'cell:3:3' in {r for r,_,_ in a.occupancy}
    assert a.start_resources
    assert a.duration>=4


def test_repeated_corridor_visits_keep_section_intervals():
    path=[(-3,0,0),(0,0,0),(3,0,0),(3,3,0),(-3,3,0),(-3,0,0),(0,0,0)]
    candidate=compile_route(path,(.4,.4,.02),(1.,1.),{'aisle':[[-.5,-.5,.5,.5]]})
    visits=[(a,b) for r,a,b in candidate.occupancy if r=='section:aisle']
    assert len(visits)==2 and visits[0][1]<visits[1][0]


def test_swept_space_time_cells_detect_crossing_outside_corridors():
    a=compile_route([(-2,0,0),(2,0,0)],(.6,.6,.02),(1.,1.),{})
    b=compile_route([(0,-2,math.pi/2),(0,2,math.pi/2)],(.6,.6,.02),(1.,1.),{})
    plans,stats=solve({'a':(a,),'b':(b,)},budget=2)
    assert plans and not conflicts(plans)
    assert max(p.departure for p in plans.values())>0


def test_queued_robot_does_not_block_incumbent_exit_with_virtual_motion():
    from hetero_fleet_scheduler.mapf_router import committed_suffix
    path=[(0.,0.,0.),(1.,0.,0.),(2.,0.,0.)]
    assert committed_suffix(path,(0.,0.,0.),{'waiting':True}) is None
    assert committed_suffix(path,(1.,0.,0.),{'waiting':False,'allowed':True})==[(1.,0.,0.),(2.,0.,0.)]
    assert committed_suffix(path,(2.,0.,0.),{}) is None


def test_inactive_or_unplanned_robot_has_no_virtual_future_route():
    from hetero_fleet_scheduler.mapf_router import committed_suffix
    assert committed_suffix([(0,0,0),(2,0,0)],(0,0,0),{'allowed':False,'waiting':False}) is None
