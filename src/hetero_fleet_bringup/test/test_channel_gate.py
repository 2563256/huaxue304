"""Fail-closed command watchdog and final command-chain wiring."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import yaml
from geometry_msgs.msg import Twist

BRINGUP = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('channel_gate', BRINGUP / 'scripts/channel_gate.py')
gate = importlib.util.module_from_spec(spec); spec.loader.exec_module(gate)


def test_watchdog_stops_every_axis_and_does_not_replay_stale_command(monkeypatch):
    now = [10.]
    monkeypatch.setattr(gate.time, 'monotonic', lambda: now[0])
    output = []
    node = object.__new__(gate.ChannelGate)
    node.timeout = .35; node.received = -float('inf'); node.last_command = Twist()
    node.pub = SimpleNamespace(publish=output.append)
    node.publish()
    assert output[-1] == Twist()
    command = Twist(); command.linear.x = .3; command.linear.y = .2; command.angular.z = .5
    node.on_command(command)
    assert output[-1] == command
    now[0] += .36; node.publish()
    assert output[-1] == Twist()
    node.on_command(Twist())
    assert output[-1] == Twist()


def test_all_four_collision_monitors_consume_reserved_commands():
    for kind in ['light', 'heavy', 'omni', 'g1']:
        d = yaml.safe_load((BRINGUP / f'config/nav2_params_{kind}.yaml').read_text())
        monitor = d['collision_monitor']['ros__parameters']
        assert monitor['cmd_vel_in_topic'] == 'cmd_vel_reserved'
        assert monitor['cmd_vel_out_topic'] == 'cmd_vel'


def test_reservation_status_is_visible_in_rviz():
    config = yaml.safe_load((BRINGUP / 'rviz/fleet.rviz').read_text())
    displays = config['Visualization Manager']['Displays']
    display = next(d for d in displays if d['Name'] == 'Channel Reservations')
    assert display['Class'] == 'rviz_default_plugins/MarkerArray'
    assert display['Topic']['Value'] == '/channel_markers'
