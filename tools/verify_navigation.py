#!/usr/bin/env python3
"""Check real Nav2 plans and swept rectangular bodies against the static map.

Unlike the all-orientations circle check, this verifies heading-dependent exits,
transport legs and returns for each eligible wheeled robot. Other robots are not
inserted as obstacles: traffic scheduling and Gazebo tracking are separate checks.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from verify_scene import parse_zones

import numpy as np
from PIL import Image
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/hetero_fleet_scheduler'))
from hetero_fleet_scheduler.geometry import rectangle_is_free


class RectangleMap:
    """Rectangle/cell separating-axis test, including occupied cell interiors."""

    def __init__(self, path):
        meta = yaml.safe_load(Path(path).read_text())
        self.resolution = float(meta['resolution'])
        self.ox, self.oy = map(float, meta['origin'][:2])
        pixels = np.asarray(Image.open(Path(path).parent / meta['image']).convert('L'))
        occupancy = pixels / 255.0 if meta.get('negate', 0) else 1.0 - pixels / 255.0
        # Unknown cells also block this verification, matching allow_unknown: false.
        self.blocked = occupancy >= float(meta['free_thresh'])
        self.height, self.width = self.blocked.shape

    def free(self, pose, footprint, padding):
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        hx, hy = footprint[0] / 2 + padding, footprint[1] / 2 + padding
        ax, ay = abs(c) * hx + abs(s) * hy, abs(s) * hx + abs(c) * hy
        r = self.resolution
        i0, i1 = math.floor((x - ax - self.ox) / r), math.floor((x + ax - self.ox) / r)
        j0, j1 = math.floor((y - ay - self.oy) / r), math.floor((y + ay - self.oy) / r)
        if i0 < 0 or j0 < 0 or i1 >= self.width or j1 >= self.height:
            return False
        rows, cols = np.nonzero(self.blocked[self.height - 1 - j1:self.height - j0, i0:i1 + 1])
        dx = self.ox + (cols + i0 + 0.5) * r - x
        dy = self.oy + (j1 - rows + 0.5) * r - y
        half_cell_projection = r / 2 * (abs(c) + abs(s))
        overlap = ((np.abs(dx) <= ax + r / 2) & (np.abs(dy) <= ay + r / 2)
                   & (np.abs(c * dx + s * dy) <= hx + half_cell_projection)
                   & (np.abs(-s * dx + c * dy) <= hy + half_cell_projection))
        return not np.any(overlap)

    def swept_check(self, poses, footprint, padding):
        radius = math.hypot(footprint[0] / 2 + padding, footprint[1] / 2 + padding)
        checked = 0
        for a, b in zip(poses, poses[1:]):
            dyaw = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
            distance = math.hypot(b[0] - a[0], b[1] - a[1])
            # Include pure rotations. Limit translation + corner arc to 1/4 cell.
            steps = max(1, math.ceil((distance + radius * abs(dyaw)) / (self.resolution / 4)))
            for index in range(steps + 1):
                t = index / steps
                sample = (a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), a[2] + t * dyaw)
                checked += 1
                if not self.free(sample, footprint, padding):
                    return checked, list(sample)
        return checked, None


def cases_for(entry, robot_type, tasks):
    cases, seen = [], set()

    def add(name, start, goal):
        key = tuple(start), tuple(goal)
        if key not in seen:
            seen.add(key)
            cases.append({'name': name, 'start': list(start), 'goal': list(goal)})

    add('leave_charger', entry['spawn'], entry['charger_exit'])
    add('return_from_exit', entry['charger_exit'], entry['charger'])
    for task in tasks:
        required = task.get('required_capability', '')
        if task.get('payload_kg', 0) > robot_type['payload_kg']:
            continue
        if required and required not in robot_type['capabilities']:
            continue
        pickup, dropoff = task['pickup'], task['dropoff']
        add(task['id'] + ':to_pickup', entry['charger_exit'], pickup)
        add(task['id'] + ':transport', pickup, dropoff)
        add(task['id'] + ':return', dropoff, entry['charger'])
    return cases


def check(planner_binary, report_path=None):
    report_path = report_path or ROOT / "build/state_lattice_scene_report.json"
    map_path = ROOT / 'src/hetero_fleet_bringup/maps/geomap.yaml'
    profiles = yaml.safe_load((ROOT / 'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    scene = RectangleMap(map_path)
    meta = yaml.safe_load(map_path.read_text())
    pixels = Image.open(map_path.parent / meta['image']).convert('L')
    occupancy = {'px': pixels.load(), 'w': pixels.width, 'h': pixels.height,
                 'res': meta['resolution'], 'ox': meta['origin'][0],
                 'oy': meta['origin'][1], 'negate': meta.get('negate', 0),
                 'free_thresh': meta['free_thresh']}
    tasks = []
    for filename in ['demo_10.yaml', 'mixed_20.yaml']:
        tasks.extend(yaml.safe_load((ROOT / 'src/hetero_fleet_scheduler/config/tasks' / filename).read_text())['tasks'])
    results, failures = [], []
    zones = parse_zones(ROOT / 'src/hetero_fleet_gz/worlds/geomap_fleet.sdf')
    for index, entry in enumerate(profiles['fleet']):
        zone = {'light_1': 'charger_A', 'heavy_1': 'charger_C', 'omni_1': 'charger_D', 'g1_1': 'charger_B'}[entry['name']]
        assert tuple(entry['spawn']) == tuple(entry['charger']), f'{entry["name"]}: spawn/charger disagree'
        assert zones[zone] == tuple(entry['charger'][:2]), f'{entry["name"]}: world marker disagrees'
    for entry in profiles['fleet']:
        robot_type = profiles['robot_types'][entry['type']]
        for label in ['spawn', 'charger_exit', 'charger']:
            if not scene.free(entry[label], robot_type['footprint'], robot_type['footprint_padding']):
                failures.append(f'{entry["name"]}:{label} intersects the static map')

    (ROOT / 'build').mkdir(exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='navigation_check_', dir=ROOT / 'build') as scratch_name:
        scratch = Path(scratch_name)
        for entry in profiles['fleet']:
            robot_type = profiles['robot_types'][entry['type']]
            if robot_type['kinematics'] == 'legged':
                continue
            kind = {'diff': entry['name'].split('_')[0], 'mecanum': 'omni'}[robot_type['kinematics']]
            config = yaml.safe_load((ROOT / f'src/hetero_fleet_bringup/config/nav2_params_{kind}.yaml').read_text())
            planner = config['planner_server']['ros__parameters']['GridBased']
            if 'lattice_filepath' in planner:
                planner['lattice_filepath'] = planner['lattice_filepath'].replace('<bringup_share>', str(ROOT / 'src/hetero_fleet_bringup'))
            global_map = config['global_costmap']['global_costmap']['ros__parameters']
            local_map = config['local_costmap']['local_costmap']['ros__parameters']
            assert local_map['footprint_padding'] == robot_type['footprint_padding']
            assert global_map['footprint_padding'] == robot_type['planner_footprint_padding']
            polygon = np.asarray(yaml.safe_load(global_map['footprint']))
            planned_size = polygon.max(axis=0) - polygon.min(axis=0)
            assert np.allclose(planned_size, robot_type['planning_footprint'], atol=1e-6)
            assert np.all(planned_size + 2 * global_map['footprint_padding'] >=
                          np.asarray(robot_type['footprint']) + 2 * robot_type['footprint_padding'])
            assert global_map['inflation_layer']['inflation_radius'] == robot_type['inflation_radius']
            assert config['controller_server']['ros__parameters']['FollowPath']['PathAlignCritic']['use_path_orientations']
            assert yaml.safe_load(planner['swept_footprint']) == yaml.safe_load(local_map['footprint'])
            assert planner['swept_footprint_padding'] == local_map['footprint_padding']
            cases = cases_for(entry, robot_type, tasks)
            old_y = -.75 if entry['name'] == 'light_1' else -.85
            blocked_exit = ([1.2, .6, math.pi / 2] if entry['name'] == 'light_1'
                            else [entry['spawn'][0], .20, math.pi / 2])
            cases.append({'name': 'regression_old_exit', 'start': [entry['spawn'][0], old_y, 0.0],
                          'goal': blocked_exit})
            spec = {'map': str(map_path), 'planner': planner,
                    'footprint': global_map['footprint'], 'padding': global_map['footprint_padding'],
                    'scaling': global_map['inflation_layer']['cost_scaling_factor'],
                    'inflation_radius': global_map['inflation_layer']['inflation_radius'],
                    'cases': cases, 'output': str(scratch / 'paths.json')}
            spec_path = scratch / 'spec.yaml'
            spec_path.write_text(yaml.safe_dump(spec))
            log_path = scratch / 'planner.log'
            with log_path.open('w') as log:
                run = subprocess.run([str(planner_binary), str(spec_path)], stdout=subprocess.PIPE,
                                     stderr=log, text=True,
                                     env={**os.environ, 'ROS_DOMAIN_ID': '139'}, timeout=300)
            if run.returncode:
                raise RuntimeError(f'{entry["name"]}: native planner failed\n{log_path.read_text()[-2500:]}')
            paths = json.loads((scratch / 'paths.json').read_text())
            robot_failures = 0
            for request, path in zip(cases, paths):
                summary = {'robot': entry['name'], 'name': request['name'], 'planner': planner['plugin'],
                           'planning_seconds': round(path.get('planning_seconds', 0), 4)}
                if request['name'] == 'regression_old_exit':
                    summary['ok'] = not path['ok'] and not rectangle_is_free(
                        occupancy, request['goal'], robot_type['footprint'], robot_type['footprint_padding'])
                    summary['native_returned_approximate_path'] = path['ok']
                elif not path['ok']:
                    summary.update(ok=False, error=path.get('error', 'empty path'))
                else:
                    poses = [request['start'], *path['poses'], request['goal']]
                    count, collision = scene.swept_check(poses, robot_type['footprint'], robot_type['footprint_padding'])
                    summary.update(ok=collision is None and path['nav2_footprint_valid'],
                                   nav2_footprint_valid=path['nav2_footprint_valid'],
                                   samples=count, poses=len(path['poses']))
                    if collision:
                        summary['collision'] = collision
                    length = sum(math.dist(a[:2], b[:2]) for a, b in zip(poses, poses[1:]))
                    summary['length_m'] = round(length, 3)
                results.append(summary)
                if not summary['ok']:
                    robot_failures += 1
                    failures.append(f'{entry["name"]}:{request["name"]}: {summary}')

            print(f'{entry["name"]}: {len(cases)-robot_failures}/{len(cases)} checks passed', flush=True)
    report = {'scope': 'Static map, nominal padded rectangular body, native Nav2 planners; excludes fleet traffic and Gazebo tracking.',
              'checks': len(results), 'passed': sum(item['ok'] for item in results),
              'failures': failures, 'results': results}
    if report_path:
        Path(report_path).parent.mkdir(parents=True, exist_ok=True)
        Path(report_path).write_text(json.dumps(report, indent=2) + '\n')
    for failure in failures:
        print('FAIL:', failure)
    print(f'RESULT: {len(results)} checks, {len(failures)} failures')
    return 1 if failures else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planner', type=Path, default=ROOT / 'build/hetero_fleet_navigation/scene_planner_probe')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    if not os.environ.get('AMENT_PREFIX_PATH'):
        parser.error('Source /opt/ros/jazzy/setup.bash and install/setup.bash first.')
    if not args.planner.is_file():
        parser.error('Build hetero_fleet_navigation with BUILD_TESTING=ON first.')
    return check(args.planner.resolve(), args.report)


if __name__ == '__main__':
    raise SystemExit(main())
