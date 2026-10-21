"""Position-only arrival, actual headings, ROS-time dwell and G1 exemption."""
import math
import time
from types import SimpleNamespace
import pytest
from test_standoff import make_manager, place, ScriptedNav, FakeFuture
from hetero_fleet_scheduler.task_model import Pose2D
from hetero_fleet_scheduler.reservations import ChannelReservations


def test_task_arrival_ignores_heading_even_with_watchdog_disabled():
    m = make_manager(); place(m, (2., 0., 1.2)); m.stall_enabled = False
    status, stalled, _ = m._wait_for_result(m.robots['r1'], Pose2D(2., 0., -1.), FakeFuture(), position_only=True)
    assert status == 'task_arrived' and not stalled
    m.live_pose_stamps = {'r1': time.monotonic() - 1.}
    assert m._task_arrival_pose(m.robots['r1'], Pose2D(2., 0., -1.)) is None
    m.live_pose_stamps['r1'] = time.monotonic()
    assert m._task_arrival_pose(m.robots['r1'], Pose2D(2.2, 0., -1.)) is None


def test_task_action_is_cancelled_on_position_arrival_and_actual_yaw_is_retained(monkeypatch):
    m = make_manager(); place(m, (0., 0., .8))
    nav = ScriptedNav(m, 'r1', ['wedged'], (0., 0., .8)); m.nav_clients['r1'] = nav
    # Navigation completes by position, before a never-completing heading goal.
    def arrive(seconds):
        m.live_pose['r1'] = (1.98, .01, .8)
    import hetero_fleet_scheduler.fleet_manager as module
    # Replace only the scheduler's clock facade. Patching time.sleep itself
    # also changes unrelated scripted navigation threads from other tests.
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=time.monotonic, time=time.time, sleep=arrive))
    result = m._navigate(m.robots['r1'], Pose2D(2., 0., -2.), 'task:pickup')
    assert result[0] and nav.goals[0].canceled
    assert nav.goals[0].goal.behavior_tree.endswith('/task_navigation.xml')
    assert m.robots['r1'].pose.yaw == .8
    assert m.robots['r1'].pose.x == 1.98


def test_charger_goal_retains_heading_and_default_behavior_tree():
    m = make_manager(); nav = ScriptedNav(m, 'r1', ['wedged'], (0., 0., 0.)); m.nav_clients['r1'] = nav
    handle, _ = m._send_goal(m.robots['r1'], Pose2D(2., 0., 1.5), 'return:charger')
    assert handle.goal.behavior_tree == ''
    assert handle.goal.pose.pose.orientation.z == pytest.approx(math.sin(.75))


@pytest.mark.parametrize('ticks', [
    [0., 0., 0., .5, 1., 2.99, 3.],
    [0., 1., -1., -1., 0., 1.99, 2.],
])
def test_three_second_service_uses_ros_clock_and_handles_pause_and_reset(monkeypatch, ticks):
    import hetero_fleet_scheduler.fleet_manager as module
    m = make_manager(); current = [ticks[0]]; position = [0]; messages = []
    m._now = lambda: current[0]
    m._task_hold_publishers = {'r1': SimpleNamespace(publish=lambda msg: messages.append(msg.data))}
    def advance(_):
        assert messages[-1] is True
        position[0] += 1; current[0] = ticks[position[0]]
    monkeypatch.setattr(module, 'time', SimpleNamespace(monotonic=time.monotonic, time=time.time, sleep=advance))
    monkeypatch.setattr(module.rclpy, 'ok', lambda: True)
    assert m._task_point_dwell(m.robots['r1'], 3.)
    assert position[0] == len(ticks) - 1 and messages == [True, False]


def test_interrupted_service_never_reports_completion_and_clears_hold(monkeypatch):
    import hetero_fleet_scheduler.fleet_manager as module
    m = make_manager(); messages = []; m._now = lambda: 0.
    m._task_hold_publishers = {'r1': SimpleNamespace(publish=lambda msg: messages.append(msg.data))}
    monkeypatch.setattr(module.rclpy, 'ok', lambda: False)
    assert not m._task_point_dwell(m.robots['r1'], 3.)
    assert messages == [True, False]


@pytest.mark.parametrize('mode', ['fifo', 'sipp_pbs'])
def test_g1_does_not_take_a_token_or_wait_for_a_wheeled_occupant(mode):
    c = ChannelReservations({'scheduling': mode, 'reservation_exempt_robots': ['g1'],
                             'channels': [{'id': 'ch', 'bounds': [0., -.6, 2., .6]}]},
                            {'wheel': (.6, .4, .02), 'g1': (.6, .52, .04)})
    for name, pose in [('wheel', (1., 0., 0.)), ('g1', (1., .2, 0.))]:
        c.observe(name, pose, 0.); c.set_active(name, True); c.set_path(name, [pose, (3., pose[1], 0.)])
    d, s = c.step(0., 0.)
    assert d['wheel']['allowed'] and d['g1']['allowed'] and not d['g1']['waiting']
    assert s['groups']['ch']['owner'] == 'wheel'
    assert s['groups']['ch']['occupants'] == ['wheel']
    c.set_active('wheel', False); c.observe('wheel', (4., 0., 0.), .1); c.observe('g1', (1., .2, 0.), .1)
    _, s = c.step(.1, .1)
    assert s['groups']['ch']['owner'] is None and not s['groups']['ch']['occupants']
    # Missing wheel odometry stops wheel admissions, without queueing G1.
    c.observe('g1', (1., .2, 0.), 10.)
    d, s = c.step(10., 10.)
    assert not s['pose_ready'] and d['g1']['allowed'] and not d['g1']['waiting']


def test_unknown_exemption_is_rejected():
    with pytest.raises(ValueError, match='Unknown reservation-exempt'):
        ChannelReservations({'reservation_exempt_robots': ['missing'],
                             'channels': [{'id': 'ch', 'bounds': [0., 0., 2., 2.]}]}, {'wheel': (.6, .4, .02)})


def test_task_state_completes_only_after_both_three_second_services():
    from pathlib import Path
    import yaml
    from hetero_fleet_scheduler.task_model import Task, TaskState, RobotStatus
    from hetero_fleet_scheduler.metrics import MetricsRecorder
    root = Path(__file__).resolve().parents[3]
    profile = yaml.safe_load((root / 'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    m = make_manager(); m.charger_pose = {}; m.load_time = profile['scheduler']['load_time_sec']
    m.unload_time = profile['scheduler']['unload_time_sec']; m.zone_claims = {}
    m.metrics = MetricsRecorder(['r1']); m._claim_zone_slot = lambda robot, point: point
    m._release_zone = lambda *args: None; m._try_assign_all = lambda: None
    m._apply_battery_drain = lambda *args: None
    calls = []; task = Task('service', pickup=Pose2D(1., 0., 2.), dropoff=Pose2D(2., 0., -1.))
    def navigate(robot, point, tag):
        calls.append(tag); robot.pose = Pose2D(point.x, point.y, .7)
        return True, 1., 0.
    def dwell(robot, seconds):
        assert task.state != TaskState.COMPLETED
        assert robot.pose.yaw == .7 and seconds == 3.
        calls.append(seconds)
        return True
    m._navigate = navigate; m._task_point_dwell = dwell
    m._execute_task(m.robots['r1'], task)
    assert calls == ['service:pickup', 3., 'service:dropoff', 3.]
    assert task.state == TaskState.COMPLETED and m.robots['r1'].status == RobotStatus.IDLE
    assert m.robots['r1'].tasks_completed == 1
