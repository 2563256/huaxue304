"""Regress enlarged bodies, invalid old exits, and approximate Nav2 goals."""
import math
from pathlib import Path

from PIL import Image
import pytest
import yaml

from hetero_fleet_scheduler.geometry import rectangle_is_free
from hetero_fleet_scheduler.task_model import Pose2D
from test_standoff import make_manager

ROOT = Path(__file__).resolve().parents[3]


def scene():
    map_file = ROOT / 'src/hetero_fleet_bringup/maps/geomap.yaml'
    meta = yaml.safe_load(map_file.read_text())
    image = Image.open(map_file.parent / meta['image']).convert('L')
    occupancy = {'px': image.load(), 'w': image.width, 'h': image.height,
                 'res': meta['resolution'], 'ox': meta['origin'][0], 'oy': meta['origin'][1],
                 'negate': meta['negate'], 'free_thresh': meta['free_thresh']}
    profiles = yaml.safe_load((ROOT / 'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    return occupancy, profiles


@pytest.mark.parametrize('name', ['light_1', 'heavy_1', 'omni_1', 'g1_1'])
def test_current_spawn_exit_and_charger_fit_the_complete_body(name):
    occupancy, profiles = scene()
    entry = next(robot for robot in profiles['fleet'] if robot['name'] == name)
    profile = profiles['robot_types'][entry['type']]
    for key in ['spawn', 'charger_exit', 'charger']:
        assert rectangle_is_free(occupancy, entry[key], profile['footprint'], profile['footprint_padding'])


@pytest.mark.parametrize('name', ['light_1', 'heavy_1', 'omni_1'])
def test_wall_intersecting_exit_is_rejected_before_action_or_simulated_success(name):
    occupancy, profiles = scene()
    entry = next(robot for robot in profiles['fleet'] if robot['name'] == name)
    manager = make_manager()
    manager._occupancy = occupancy
    manager.types = profiles['robot_types']
    robot = manager.robots['r1']
    robot.type = entry['type']
    # The reduced light body now fits the former y=.20 pose. Move it
    # toward the wall so this still exercises an actually invalid body pose.
    target = (Pose2D(1.20, .60, math.pi / 2) if name == 'light_1'
              else Pose2D(entry['spawn'][0], .20, math.pi / 2))

    class UnexpectedClient:
        def wait_for_server(self, **kwargs):
            pytest.fail('An invalid body pose must be rejected before contacting Nav2')

    manager.nav_clients['r1'] = UnexpectedClient()
    manager._simulate_leg = lambda *args: pytest.fail('Invalid geometry must never be simulated as success')
    assert manager._send_goal(robot, target, 'old_exit') == (None, 'goal_occupied')
    assert manager._navigate(robot, target, 'old_exit') == (False, 0.0, 0.0)


@pytest.mark.parametrize('blocked_value', [0, 205])
def test_cell_inside_body_is_checked_even_when_centre_and_corners_are_free(blocked_value):
    image = Image.new('L', (10, 10), 255)
    image.putpixel((4, 4), blocked_value)
    occupancy = {'px': image.load(), 'w': 10, 'h': 10, 'res': .1, 'ox': 0., 'oy': 0.}
    assert image.getpixel((5, 4)) == 255  # robot centre is a free cell
    assert not rectangle_is_free(occupancy, (.5, .5, 0.), (.6, .6))


@pytest.mark.parametrize('target',[(-2.,4.22,0.),(11.,4.22,0.),(5.4,4.22,0.)])
def test_empty_heavy_task_marker_uses_full_rectangle_instead_of_a_side_slot(target):
    from types import SimpleNamespace
    occupancy,profiles=scene();profile=profiles['robot_types']['heavy_diff_amr']
    manager=make_manager();manager._occupancy=occupancy;manager.types=profiles['robot_types']
    robot=manager.robots['r1'];robot.type='heavy_diff_amr'
    manager.slot_clearance['r1']=math.hypot(*profile['footprint'])/2
    manager.zone_claims={};manager.get_parameter=lambda name:SimpleNamespace(value=1.)
    point=Pose2D(*target)
    # This is the observed regression: the old circle rejected a valid marker.
    assert not manager._slot_is_free(point.x,point.y,manager.slot_clearance['r1'])
    assert rectangle_is_free(occupancy,target,profile['footprint'],profile['footprint_padding'])
    assert manager._claim_zone_slot(robot,point)==point
    assert manager.zone_claims[manager._zone_key(point)]=={0:'r1'}
