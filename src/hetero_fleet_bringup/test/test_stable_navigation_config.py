"""All fleet types must actually load the stable route tree and its plugin."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml

CONFIG = Path(__file__).resolve().parents[1] / 'config'


def test_every_robot_loads_the_same_stable_strategy():
    for typ in ['light', 'heavy', 'omni', 'g1']:
        data = yaml.safe_load((CONFIG / f'nav2_params_{typ}.yaml').read_text())
        navigator = data['bt_navigator']['ros__parameters']
        assert navigator['default_nav_to_pose_bt_xml'] == '<bringup_share>/config/fleet_navigation.xml'
        assert 'hetero_stable_route_bt' in navigator['plugin_lib_names']
    root = ET.parse(CONFIG / 'fleet_navigation.xml').getroot()
    assert not list(root.iter('PathExpiringTimer'))
    gate = next(root.iter('StableReplan'))
    assert [child.tag for child in gate] == ['IsPathValid', 'ComputePathToPose']
    assert not list(root.iter('Spin'))
