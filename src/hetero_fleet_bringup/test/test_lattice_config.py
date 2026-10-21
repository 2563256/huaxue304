"""Reject incompatible control sets before launching a fleet stack."""
import importlib.util
import json
import math
from pathlib import Path
import pytest
import yaml

BRINGUP = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('fleet_nav2_launch', BRINGUP / 'launch/nav2.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


def params(kind):
    text = (BRINGUP / f'config/nav2_params_{kind}.yaml').read_text()
    return yaml.safe_load(text.replace('<bringup_share>', str(BRINGUP)))


@pytest.mark.parametrize('kind', ['light', 'heavy', 'omni', 'g1'])
def test_physical_lattice_config(kind):
    config = params(kind)
    launch.validate_lattice_params(config)
    planner = config['planner_server']['ros__parameters']['GridBased']
    data = json.loads(Path(planner['lattice_filepath']).read_text())
    metadata = data['lattice_metadata']
    assert metadata['num_of_headings'] == 64
    assert metadata['number_of_trajectories'] == len(data['primitives'])
    angles = metadata['heading_angles']
    groups = set()
    for primitive in data['primitives']:
        start, end = primitive['start_angle_index'], primitive['end_angle_index']
        groups.add(start)
        first, last = primitive['poses'][0], primitive['poses'][-1]
        assert first == [0., 0., angles[start]]
        assert abs(math.atan2(math.sin(last[2] - angles[end]), math.cos(last[2] - angles[end]))) < 1e-6
        assert all(abs(v / .05 - round(v / .05)) < 1e-6 for v in last[:2])
    assert groups == set(range(64))
    local = config['local_costmap']['local_costmap']['ros__parameters']
    global_map = config['global_costmap']['global_costmap']['ros__parameters']
    assert yaml.safe_load(local['footprint']) == yaml.safe_load(global_map['footprint'])
    assert local['footprint_padding'] == global_map['footprint_padding']


def test_mismatched_resolution_fails():
    config = params('heavy')
    config['global_costmap']['global_costmap']['ros__parameters']['resolution'] = .10
    with pytest.raises(ValueError, match='resolution'):
        launch.validate_lattice_params(config)


def test_mismatched_kinematics_fails():
    config = params('heavy')
    config['planner_server']['ros__parameters']['GridBased']['lattice_filepath'] = str(BRINGUP / 'config/lattice/omni_64.json')
    with pytest.raises(ValueError, match='motion model'):
        launch.validate_lattice_params(config)


@pytest.mark.parametrize('field, value', [('swept_footprint', '[[-1,-1],[1,-1],[1,1],[-1,1]]'),
                                          ('swept_footprint_padding', 0.0)])
def test_smaller_or_different_swept_body_fails(field, value):
    config = params('heavy')
    config['planner_server']['ros__parameters']['GridBased'][field] = value
    with pytest.raises(ValueError, match='physical footprint'):
        launch.validate_lattice_params(config)


def test_omni_uses_explicit_yaw_preserving_reverse():
    config = params('omni')
    planner = config['planner_server']['ros__parameters']['GridBased']
    control = json.loads(Path(planner['lattice_filepath']).read_text())
    starts_at_zero = [p for p in control['primitives'] if p['start_angle_index'] == p['end_angle_index'] == 0]
    endpoints = [p['poses'][-1] for p in starts_at_zero]
    for pose in [[.05, 0., 0.], [-.05, 0., 0.], [0., .05, 0.], [0., -.05, 0.]]:
        assert pose in endpoints
    planner['allow_reverse_expansion'] = True
    with pytest.raises(ValueError, match='reverse'):
        launch.validate_lattice_params(config)


def test_g1_lattice_preserves_reverse_headings_without_extra_task_rotation():
    config = params('g1')
    planner = config['planner_server']['ros__parameters']
    controller = config['controller_server']['ros__parameters']['TaskFollowPath']
    assert planner['GridBased']['plugin'] == 'hetero_fleet_navigation::SweptLattice'
    assert not planner['TaskGridBased']['match_goal_heading']
    assert controller['PathAlignCritic']['use_path_orientations']
    assert controller['motion_model'] == 'Omni'
    assert controller['vy_max'] == .12
    assert not controller['PathAngleCritic']['enabled']
    assert not controller['PreferForwardCritic']['enabled']
    assert not controller['GoalAngleCritic']['enabled']
    launch.validate_lattice_params(config)


@pytest.mark.parametrize('field, value', [('footprint_padding', .15),
                                         ('footprint', '[[-1,-1],[1,-1],[1,1],[-1,1]]')])
def test_global_path_validation_body_cannot_diverge(field, value):
    config = params('heavy')
    config['global_costmap']['global_costmap']['ros__parameters'][field] = value
    with pytest.raises(ValueError, match='global/local'):
        launch.validate_lattice_params(config)


def test_task_planner_body_is_validated_too():
    config = params('heavy')
    config['planner_server']['ros__parameters']['TaskGridBased']['swept_footprint_padding'] = 0.
    with pytest.raises(ValueError, match='physical footprint'):
        launch.validate_lattice_params(config)


@pytest.mark.parametrize('kind', ['light', 'heavy', 'omni', 'g1'])
def test_rendered_task_parameters_load_with_ros_c_parser(kind, tmp_path, monkeypatch):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    monkeypatch.setattr(launch, 'get_package_share_directory', lambda package: str(BRINGUP))
    name = f'{kind}_1'
    filename = launch.render_params(BRINGUP / f'config/nav2_params_{kind}.yaml', name, str(tmp_path), kind)
    context = Context(); nodes = []
    try:
        rclpy.init(args=['--ros-args', '--params-file', filename], context=context)
        controller = Node('controller_server', namespace=f'/{name}', context=context,
                          automatically_declare_parameters_from_overrides=True)
        nodes.append(controller)
        planner = Node('planner_server', namespace=f'/{name}', context=context,
                       automatically_declare_parameters_from_overrides=True)
        nodes.append(planner)
        assert 'TaskFollowPath' in controller.get_parameter('controller_plugins').value
        assert 'GoalAngleCritic' in controller.get_parameter('TaskFollowPath.critics').value
        assert not controller.get_parameter('TaskFollowPath.GoalAngleCritic.enabled').value
        assert controller.get_parameter('FollowPath.GoalAngleCritic.enabled').value
        assert controller.get_parameter('task_goal_checker.xy_goal_tolerance').value == (.07 if kind == 'g1' else .15)
        assert 'TaskGridBased' in planner.get_parameter('planner_plugins').value
        if kind == 'g1':
            assert not planner.get_parameter('TaskGridBased.match_goal_heading').value
            # Retain a cell-diagonal error budget even though the lattice
            # terminal sweep connects to the exact task coordinate.
            cell = params('g1')['global_costmap']['global_costmap']['ros__parameters']['resolution']
            assert math.sqrt(2) * cell + controller.get_parameter('task_goal_checker.xy_goal_tolerance').value < .15
        else:
            assert not planner.get_parameter('TaskGridBased.match_goal_heading').value
    finally:
        for node in nodes: node.destroy_node()
        context.try_shutdown()


@pytest.mark.parametrize('kind',['light','heavy','omni'])
def test_queue_retreat_planner_keeps_full_body_checks_without_reverse_penalty(kind):
    config=params(kind);launch.validate_lattice_params(config)
    planners=config['planner_server']['ros__parameters']
    assert 'RetreatGridBased' in planners['planner_plugins']
    retreat=planners['RetreatGridBased'];base=planners['GridBased']
    assert retreat['reverse_penalty']==1. and retreat['match_goal_heading']
    for key in ['swept_footprint','swept_footprint_padding','lattice_filepath','allow_reverse_expansion']:
        assert retreat[key]==base[key]
