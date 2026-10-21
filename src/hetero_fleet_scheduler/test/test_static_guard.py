import math
from pathlib import Path
from hetero_fleet_scheduler.static_guard import StaticBodyGuard
from hetero_fleet_scheduler.reservations import command_path
from test_goal_geometry import scene,ROOT


def guard():return StaticBodyGuard(ROOT/'src/hetero_fleet_bringup/maps/geomap.yaml')


def test_guard_rejects_observed_light_body_outside_static_free_space():
    _,profiles=scene();p=profiles['robot_types']['light_diff_amr'];g=guard()
    body=(*p['footprint'],p['footprint_padding'])
    assert not g.free((-6.6376,-.0887,1.1452),body)
    assert g.scale((-6.6376,-.0887,1.1452),(-.2,0.,.3),body)==0.


def test_guard_allows_valid_straight_initial_clearance_motion():
    _,profiles=scene();p=profiles['robot_types']['light_diff_amr'];g=guard()
    body=(*p['footprint'],p['footprint_padding'])
    assert g.scale((-.3,-.25,0.),(-.2,0.,0.),body)==1.


def test_guard_preserves_safe_rotation_out_of_observed_omni_skew():
    _,profiles=scene();p=profiles['robot_types']['omni_mecanum_amr'];g=guard()
    body=(*p['footprint'],p['footprint_padding'])
    assert g.scale((7.59,-.156,-.392),(0.,0.,.4),body)>0.


def test_corner_crossing_prediction_is_stopped_before_start_is_occupied():
    _,profiles=scene();p=profiles['robot_types']['light_diff_amr'];g=guard();body=(*p['footprint'],p['footprint_padding'])
    # A free body cannot rotate through the narrow southern lane at full rate.
    scale=g.scale((-6.6,-.08,0.),(0.,0.,1.5),body)
    assert 0.<=scale<1.


def test_filled_body_map_check_agrees_with_reference_static_checker():
    from hetero_fleet_scheduler.geometry import rectangle_is_free
    occ,profiles=scene();p=profiles['robot_types']['heavy_diff_amr'];g=guard();body=(*p['footprint'],p['footprint_padding'])
    for x,y in [(2.7,-.45),(-2.,4.22),(1.2,.6),(12.,1.8)]:
        for yaw in (0.,.4,math.pi/2):
            assert g.free((x,y,yaw),body)==rectangle_is_free(occ,(x,y,yaw),body[:2],body[2]+g.margin)


def test_convex_corner_preserves_safe_forward_component(tmp_path):
    from PIL import Image,ImageDraw
    import yaml
    image=Image.new('L',(120,120),255)
    ImageDraw.Draw(image).rectangle((40,0,119,79),fill=0) # world x>=2, y>=2
    image.save(tmp_path/'floor.png')
    path=tmp_path/'map.yaml'
    path.write_text(yaml.safe_dump(dict(image='floor.png',resolution=.05,origin=[0.,0.,0.],negate=0,free_thresh=.196)))
    g=StaticBodyGuard(path);body=(.4,.4,.02);pose=(1.7,1.7,0.);command=(.5,.5,0.)
    assert g.scale(pose,command,body)<1.
    actual=g.filter(pose,command,body)
    assert actual[0]>.25 and actual[1]==0.
    assert g.scale(pose,actual,body)==1.


def test_projection_never_creates_lateral_motion_for_differential_drive():
    _,profiles=scene();p=profiles['robot_types']['light_diff_amr'];g=guard();body=(*p['footprint'],p['footprint_padding'])
    command=(-.25,0.,1.5)
    actual=g.filter((-6.6,-.08,0.),command,body)
    assert actual[1]==0.
    assert all(a*b>=0 and abs(a)<=abs(b)+1e-9 for a,b in zip(actual,command))


def test_actual_angular_momentum_is_checked_after_request_changes():
    _,profiles=scene();p=profiles['robot_types']['light_diff_amr'];g=guard()
    body=(*p['footprint'],p['footprint_padding']);pose=(-6.6,-.08,0.)
    command=(-.15,0.,0.)
    assert g.filter(pose,command,body)==command
    assert g.filter(pose,command,body,measured=(0.,0.,1.5))==(0.,0.,0.)
