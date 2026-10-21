"""All fleet types must actually load the stable route tree and its plugin."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config'


def test_every_robot_loads_the_same_stable_strategy():
    for typ in ['light', 'heavy', 'omni', 'g1']:
        data = yaml.safe_load((CONFIG / f'nav2_params_{typ}.yaml').read_text())
        navigator = data['bt_navigator']['ros__parameters']
        tree = 'g1_navigation.xml' if typ == 'g1' else 'fleet_navigation.xml'
        assert navigator['default_nav_to_pose_bt_xml'] == f'<bringup_share>/config/{tree}'
        assert 'hetero_stable_route_bt' in navigator['plugin_lib_names']
    root = ET.parse(CONFIG / 'fleet_navigation.xml').getroot()
    assert not list(root.iter('PathExpiringTimer'))
    gate = next(root.iter('StableReplan'))
    assert [child.tag for child in gate] == ['IsPathValid', 'ComputePathToPose']
    assert not list(root.iter('Spin'))
    for name in ('fleet_navigation.xml', 'task_navigation.xml', 'g1_navigation.xml', 'g1_task_navigation.xml'):
        tree = ET.parse(CONFIG / name).getroot()
        assert not list(tree.iter('BackUp'))  # Unplanned reverse caused a real wall-envelope failure.
        assert next(tree.iter('StableReplan')).attrib['goal'] == '{goal}'
        occupied = tree.find('.//Fallback[@name="HandleTemporaryGoalOccupancy"]/Sequence')
        assert occupied.find('ScriptCondition').attrib['code'] == 'compute_path_error_code == 206'
        assert float(occupied.find('Wait').attrib['wait_duration']) == 5.


def test_task_trees_use_position_planner_checker_and_no_goal_angle_controller():
    for kind in ('light', 'heavy', 'omni', 'g1'):
        data = yaml.safe_load((CONFIG / f'nav2_params_{kind}.yaml').read_text())
        controller = data['controller_server']['ros__parameters']
        planner = data['planner_server']['ros__parameters']
        assert 'TaskFollowPath' in controller['controller_plugins']
        assert 'task_goal_checker' in controller['goal_checker_plugins']
        assert controller['task_goal_checker']['yaw_goal_tolerance'] > 3.141592653589793
        assert not controller['TaskFollowPath']['GoalAngleCritic']['enabled']
        assert controller['FollowPath']['GoalAngleCritic']['enabled']
        for name in ('FollowPath','TaskFollowPath'):
            control = controller[name]
            align = control['PathAlignCritic']
            horizon = abs(control['vx_min']) * control['model_dt'] * (control['time_steps'] - 1)
            assert align['offset_from_furthest'] * .05 < horizon
            assert align['threshold_to_consider'] == .15
        assert 'TaskGridBased' in planner['planner_plugins']
        assert not planner['TaskGridBased']['match_goal_heading']
        tree = ET.parse(CONFIG / ('g1_task_navigation.xml' if kind == 'g1' else 'task_navigation.xml')).getroot()
        assert all(n.attrib['planner_id'] == 'TaskGridBased' for n in tree.iter('ComputePathToPose'))
        follow = next(tree.iter('FollowPath'))
        assert follow.attrib['controller_id'] == 'TaskFollowPath'
        assert follow.attrib['goal_checker_id'] == 'task_goal_checker'
        assert not list(tree.iter('Spin'))
    for name in ('g1_navigation.xml', 'g1_task_navigation.xml'):
        gate = next(ET.parse(CONFIG / name).getroot().iter('StableReplan'))
        assert float(gate.attrib['blocked_duration']) == .5
        assert float(gate.attrib['min_interval']) == 1.
