#!/usr/bin/env python3
"""Native position-only task arrivals and G1 detours around a full wheeled body.

Checks planner geometry, not Gazebo, locomotion tracking or moving-obstacle
prediction. General/docking plans retain their orientation constraints.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import yaml
from PIL import Image
from verify_navigation import ROOT, RectangleMap


def check(binary, report_path):
    results = []
    with tempfile.TemporaryDirectory(prefix='task_service_', dir=ROOT / 'build') as tmp:
        scratch = Path(tmp)
        base = Image.new('L', (120, 120), 255)
        base.save(scratch / 'map.pgm')
        map_path = scratch / 'map.yaml'
        map_path.write_text(yaml.safe_dump({'image': 'map.pgm', 'resolution': .05,
                                          'origin': [0., 0., 0.], 'negate': 0,
                                          'occupied_thresh': .65, 'free_thresh': .196}))
        for kind in ('light', 'heavy', 'omni', 'g1'):
            config = yaml.safe_load((ROOT / f'src/hetero_fleet_bringup/config/nav2_params_{kind}.yaml').read_text())
            planner = config['planner_server']['ros__parameters']['TaskGridBased'].copy()
            if 'lattice_filepath' in planner:
                planner['lattice_filepath'] = planner['lattice_filepath'].replace('<bringup_share>', str(ROOT / 'src/hetero_fleet_bringup'))
            costmap = config['global_costmap']['global_costmap']['ros__parameters']
            shape = yaml.safe_load(costmap['footprint'])
            body = [2 * max(abs(p[i]) for p in shape) for i in (0, 1)]
            origin = [3.025, 3.025, 0.]
            cases = [{'name': 'straight_ignores_requested_yaw', 'start': origin, 'goal': [3.525, 3.025, math.pi]},
                     {'name': 'already_at_point', 'start': origin, 'goal': [*origin[:2], math.pi]},
                     {'name': 'occupied_goal_centre', 'start': origin, 'goal': [4.525, 3.025, math.pi],
                      'obstacles': [[4.525, 3.025]], 'expect_failure': True}]
            if kind != 'g1':
                cases.append({'name': 'requested_yaw_collides_but_arrival_heading_fits', 'start': origin,
                              'goal': [3.525, 3.025, math.pi / 2], 'obstacles': [[3.525, 3.825]]})
            else:
                cases.append({'name': 'off_grid_task_arrival_budget', 'start': origin,
                              'goal': [3.58, 3.04, math.pi]})
                # Rasterize an actual heavy vehicle, not one point or its centre.
                obstacle = [[(ix + .5) * .05, (iy + .5) * .05]
                            for ix in range(43, 78) for iy in range(48, 74)]
                cases.append({'name': 'detour_around_wheeled_body', 'start': [1.225, 3.025, 0.],
                              'goal': [4.825, 3.025, math.pi], 'obstacles': obstacle})
            spec = {'reinflate_obstacles': kind == 'g1', 'map': str(map_path), 'planner': planner, 'footprint': costmap['footprint'],
                    'padding': costmap['footprint_padding'], 'scaling': costmap['inflation_layer']['cost_scaling_factor'],
                    'inflation_radius': costmap['inflation_layer']['inflation_radius'],
                    'cases': cases, 'output': str(scratch / 'paths.json')}
            spec_path = scratch / 'spec.yaml'; spec_path.write_text(yaml.safe_dump(spec))
            run = subprocess.run([str(binary), str(spec_path)], capture_output=True, text=True,
                                 env={**os.environ, 'ROS_DOMAIN_ID': '139'}, timeout=120)
            if run.returncode: raise RuntimeError(run.stderr[-3000:])
            paths = json.loads((scratch / 'paths.json').read_text())
            assert len(paths) == len(cases)
            for request, path in zip(cases, paths):
                row = {'kind': kind, 'name': request['name'], 'ok': False}
                if request.get('expect_failure'):
                    row['ok'] = not path['ok']
                elif path['ok']:
                    pixels = base.copy()
                    for x, y in request.get('obstacles', []):
                        pixels.putpixel((int(x / .05), 119 - int(y / .05)), 0)
                    pixels.save(scratch / 'map.pgm')
                    poses = [request['start'], *path['poses']]
                    _, collision = RectangleMap(map_path).swept_check(poses, body, costmap['footprint_padding'])
                    row['ok'] = collision is None and path['nav2_footprint_valid']
                    if collision: row['collision'] = collision
                    row['nav2_footprint_valid'] = path['nav2_footprint_valid']
                    assert math.dist(poses[-1][:2], request['goal'][:2]) < .13
                    if kind == 'g1':
                        endpoint_error = math.dist(poses[-1][:2], request['goal'][:2])
                        stopping = config['controller_server']['ros__parameters']['task_goal_checker']['xy_goal_tolerance']
                        row['arrival_error_budget_m'] = endpoint_error + stopping
                        row['ok'] &= row['arrival_error_budget_m'] <= .15
                    row['final_yaw'] = poses[-1][2]
                    if request['name'] not in ('detour_around_wheeled_body', 'off_grid_task_arrival_budget'):
                        row['ok'] &= max(abs(math.atan2(math.sin(p[2]), math.cos(p[2]))) for p in poses) < 1e-6
                    elif request['name'] == 'detour_around_wheeled_body':
                        row['max_lateral_detour_m'] = max(abs(p[1] - 3.025) for p in poses)
                        row['ok'] &= row['max_lateral_detour_m'] > .9
                    base.save(scratch / 'map.pgm')
                else:
                    row['error'] = path.get('error')
                results.append(row)
                print(f"{kind}:{request['name']}: {'PASS' if row['ok'] else 'FAIL'}", flush=True)
    report = {'scope': __doc__, 'checks': len(results), 'passed': sum(bool(r['ok']) for r in results), 'results': results}
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(json.dumps(report, indent=2) + '\n')
    return int(report['checks'] != report['passed'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planner', type=Path, default=ROOT / 'build/hetero_fleet_navigation/scene_planner_probe')
    parser.add_argument('--report', type=Path, default=ROOT / 'build/task_service_report.json')
    args = parser.parse_args()
    raise SystemExit(check(args.planner, args.report))
