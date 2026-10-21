#!/usr/bin/env python3
"""Native State Lattice regressions: kinematics, rotation sweep and map changes."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
from PIL import Image
import yaml
from verify_navigation import ROOT, RectangleMap


def check(binary, report_path=None):
    report_path = report_path or ROOT / "build/state_lattice_motion_report.json"
    results = []
    with tempfile.TemporaryDirectory(prefix='lattice_regression_', dir=ROOT / 'build') as dirname:
        scratch = Path(dirname)
        pixels = Image.new('L', (120, 120), 255)
        pixels.save(scratch / 'map.pgm')
        map_yaml = scratch / 'map.yaml'
        map_yaml.write_text(yaml.safe_dump({'image': 'map.pgm', 'resolution': .05,
                                           'origin': [0, 0, 0], 'negate': 0,
                                           'occupied_thresh': .65, 'free_thresh': .196}))
        for model, kind in [('diff', 'heavy'), ('omni', 'omni')]:
            config = yaml.safe_load((ROOT / f'src/hetero_fleet_bringup/config/nav2_params_{kind}.yaml').read_text())
            planner = config['planner_server']['ros__parameters']['GridBased'].copy()
            planner['lattice_filepath'] = planner['lattice_filepath'].replace('<bringup_share>', str(ROOT / 'src/hetero_fleet_bringup'))
            global_map = config['global_costmap']['global_costmap']['ros__parameters']
            local = config['local_costmap']['local_costmap']['ros__parameters']
            polygon = yaml.safe_load(local['footprint'])
            body = [2 * max(abs(p[i]) for p in polygon) for i in range(2)]
            origin = [3.025, 3.025, 0]
            cases = [
                {'name': 'forward', 'start': origin, 'goal': [3.525, 3.025, 0]},
                {'name': 'backwards', 'start': origin, 'goal': [2.525, 3.025, 0]},
                {'name': 'lateral', 'start': origin, 'goal': [3.025, 3.525, 0]},
                {'name': 'heading_seam', 'start': [*origin[:2], 2 * math.pi - .01], 'goal': [*origin[:2], .01]},
                # Both endpoint bodies fit. A direct rotation strikes this cell.
                {'name': 'rotation_sweep', 'start': origin, 'goal': [*origin[:2], math.pi / 2],
                 'obstacles': [[3.975, 3.125]]},
                {'name': 'occupied_goal_body', 'start': origin, 'goal': [3.525, 3.025, 0],
                 'obstacles': [[4.025, 3.225]], 'error': 'Goal body intersects'},
                {'name': 'cancel', 'start': origin, 'goal': origin, 'cancel': True,
                 'error': 'search cancelled'},
                {'name': 'dynamic_obstacle', 'start': [1.525, 2.025, 0], 'goal': [4.525, 2.025, 0],
                 'obstacles': [[3.025, 2.025]]},
                {'name': 'obstacle_removed', 'start': [1.525, 2.025, 0], 'goal': [4.525, 2.025, 0]},
            ]
            # For omni's shorter body, place the occupied-goal cell inside its
            # body and outside the start, while leaving the centre free.
            if model == 'omni':
                cases[5]['obstacles'] = [[4.025, 3.225]]
            spec = {'map': str(map_yaml), 'planner': planner, 'footprint': global_map['footprint'],
                    'padding': global_map['footprint_padding'], 'scaling': 1.5,
                    'inflation_radius': 1.2, 'cases': cases, 'output': str(scratch / 'paths.json')}
            spec_path = scratch / 'spec.yaml'; spec_path.write_text(yaml.safe_dump(spec))
            run = subprocess.run([str(binary), str(spec_path)], capture_output=True, text=True,
                                 env={**os.environ, 'ROS_DOMAIN_ID': '139'}, timeout=180)
            if run.returncode:
                raise RuntimeError(run.stderr[-3000:])
            paths = json.loads((scratch / 'paths.json').read_text())
            for request, path in zip(cases, paths):
                row = {'model': model, 'name': request['name'], 'ok': False,
                       'planning_seconds': round(path['planning_seconds'], 4)}
                if 'error' in request:
                    row['ok'] = not path['ok'] and request['error'] in path.get('error', '')
                elif path['ok']:
                    image = pixels.copy()
                    for x, y in request.get('obstacles', []):
                        image.putpixel((int(x / .05), 119 - int(y / .05)), 0)
                    image.save(scratch / 'map.pgm')
                    scene = RectangleMap(map_yaml)
                    poses = [request['start'], *path['poses'], request['goal']]
                    _, collision = scene.swept_check(poses, body, local['footprint_padding'])
                    row['ok'] = collision is None and path['nav2_footprint_valid']
                    if collision:
                        row['collision'] = collision
                    yaws = [abs(math.atan2(math.sin(p[2]), math.cos(p[2]))) for p in path['poses']]
                    if request['name'] in ['forward', 'backwards'] or (model == 'omni' and request['name'] == 'lateral'):
                        row['ok'] &= max(yaws) < 1e-6
                    if model == 'diff' and request['name'] == 'lateral':
                        row['ok'] &= max(yaws) > .1
                    if request['name'] == 'rotation_sweep':
                        # Rotation at a fixed centre is blocked; a translation
                        # must be inserted instead of accepting endpoint-only checks.
                        row['ok'] &= max(math.dist(p[:2], request['start'][:2]) for p in path['poses']) > .025
                    if request['name'] == 'obstacle_removed':
                        row['ok'] &= max(abs(p[1] - 2.025) for p in path['poses']) < 1e-6
                    pixels.save(scratch / 'map.pgm')
                else:
                    row['error'] = path.get('error')
                results.append(row)
                print(f'{model}:{request["name"]}: {"PASS" if row["ok"] else "FAIL"}', flush=True)
    report = {'checks': len(results), 'passed': sum(bool(r['ok']) for r in results), 'results': results}
    if report_path:
        Path(report_path).write_text(json.dumps(report, indent=2) + '\n')
    return int(report['checks'] != report['passed'])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planner', type=Path, default=ROOT / 'build/hetero_fleet_navigation/scene_planner_probe')
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    return check(args.planner, args.report)


if __name__ == '__main__':
    raise SystemExit(main())
