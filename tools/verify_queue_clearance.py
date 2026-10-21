#!/usr/bin/env python3
"""Replay the observed exit deadlock with real Nav2 and complete peer bodies."""
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


def check(binary, report):
    bringup = ROOT / 'src/hetero_fleet_bringup'
    profiles = yaml.safe_load((ROOT / 'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    fleet = {r['name']: r for r in profiles['fleet']}
    map_path = bringup / 'maps/geomap.yaml'
    meta = yaml.safe_load(map_path.read_text()); ox, oy = meta['origin'][:2]; res = meta['resolution']

    def peer(name, x, y, yaw=0.):
        length, width = profiles['robot_types'][fleet[name]['type']]['footprint']
        # Same peer margin as scan_frame_relay, rasterized conservatively.
        length += .24; width += .24
        c, sn = math.cos(yaw), math.sin(yaw)
        ax, ay = abs(c)*length/2 + abs(sn)*width/2, abs(sn)*length/2 + abs(c)*width/2
        out = []
        for i in range(math.floor((x-ax-ox)/res), math.ceil((x+ax-ox)/res)):
            for j in range(math.floor((y-ay-oy)/res), math.ceil((y+ay-oy)/res)):
                dx, dy = ox+(i+.5)*res-x, oy+(j+.5)*res-y
                projection = res/2*(abs(c)+abs(sn))
                if abs(c*dx+sn*dy) <= length/2+projection and abs(-sn*dx+c*dy) <= width/2+projection:
                    out.append([x+dx, y+dy])
        return out

    light = peer('light_1', -.76, -.45)
    cases = [
        {'name': 'both_exits_blocked', 'start': [6.9, 4.2, -2.65], 'goal': [4.6, -.45, 0.],
         'obstacles': light + peer('heavy_1', 9.62, -.43), 'expect_failure': True},
        {'name': 'east_queue_retreats_to_bay', 'start': [6.9, 4.2, -2.65], 'goal': [4.6, -.45, 0.],
         'obstacles': light + peer('heavy_1', *fleet['heavy_1']['charger'][:2])},
        {'name':'v5_charger_blocked_by_g1', 'start':[6.90,4.225,3.138],
         'goal':[4.6,-.45,0.], 'expect_failure':True,
         'obstacles':peer('light_1',-.46,-.25)+peer('heavy_1',7.5,-.25)+peer('g1_1',5.24,-.67,.2)},
        {'name':'v5_exit_before_docking', 'start':[6.90,4.225,3.138], 'goal':[10.8,-.25,0.],
         'obstacles':peer('light_1',-.46,-.25)+peer('heavy_1',7.5,-.25)+peer('g1_1',5.24,-.67,.2)}]
    for x, expected in [(2.7, True), (-2.2, False)]:
        cases.append({'name':f'owner_return_after_light_retreat_{x}',
                      'start':[10.93,-.18,-.02],'goal':[4.6,-.45,0.],
                      'expect_failure':expected,
                      'obstacles':peer('light_1',x,-.25)+peer('heavy_1',7.6,-.27,-.12)+
                                  peer('g1_1',*fleet['g1_1']['idle_clearance'])})
    cases.append({'name':'choose_exit_clear_of_the_queue_retreat',
                  'start':[-2.,4.22,3.14],'goal':[0.,-.25,0.],
                  'obstacles':peer('light_1',-2.2,-.25)+peer('heavy_1',7.5,-.25)+
                              peer('g1_1',*fleet['g1_1']['idle_clearance'])})
    cases.append({'name':'unsafe_reverse_start_rejected','robot':'heavy',
                  'planner':'RetreatGridBased','start':[2.21,-.35,math.pi],
                  'goal':[7.5,-.25,math.pi], 'expect_failure':True,
                  'obstacles':peer('light_1',-2.2,-.25)+peer('omni_1',10.93,-.19)+
                              peer('g1_1',*fleet['g1_1']['idle_clearance'])})
    cases.append({'name':'preserve_heading_during_reverse_retreat','robot':'heavy',
                  'planner':'RetreatGridBased','start':[2.60,-.35,math.pi],
                  'goal':[7.5,-.25,math.pi], 'preserve_heading':True,
                  'obstacles':peer('light_1',-2.2,-.25)+peer('omni_1',10.93,-.19)+
                              peer('g1_1',*fleet['g1_1']['idle_clearance'])})
    cases.append({'name':'v16_retreat_follows_displacement_heading','robot':'heavy',
                  'planner':'RetreatGridBased','start':[-.98047,-.3506,1.38],
                  'goal':[-2.2,-.25,math.atan2(.1006,-1.21953)-math.pi], 'forecourt_only':True,
                  'obstacles':peer('light_1',.13,4.24,-3.01)+peer('omni_1',7.58,-.35)+
                              peer('g1_1',4.83,-.5,-1.27)})
    heavy_obstacles = peer('light_1',-.46,-.25)+peer('omni_1',6.90,4.225,3.138)+peer('g1_1',5.24,-.67,.2)
    cases.extend([
        {'name':'v5_long_queue_retreat_blocked', 'robot':'heavy', 'start':[9.624,-.43,0.],
         'goal':[2.7,-.45,0.], 'obstacles':heavy_obstacles, 'expect_failure':True},
        {'name':'v5_short_queue_retreat', 'robot':'heavy', 'start':[9.624,-.43,0.],
         'goal':[7.5,-.25,0.], 'obstacles':heavy_obstacles}])
    results = []
    # The observed light-robot failure began with blind reverse along y=-.45.
    # Verify the new bay's straight reverse segment using the full padded body.
    light_profile = profiles['robot_types'][fleet['light_1']['type']]
    scene = RectangleMap(map_path)
    # Retain the original observed body for the historical failure replay.
    # The newly reduced body may now clear that old segment as well.
    _, old_collision = scene.swept_check([[-.3,-.45,0.],[-3.,-.45,0.]],
                                         [1.590909, 1.25], light_profile['footprint_padding'])
    start = fleet['light_1']['spawn']
    _, new_collision = scene.swept_check([start,[-3.,start[1],0.]],
                                         light_profile['footprint'], light_profile['footprint_padding'])
    results.append({'case':'light_westbound_reverse_clearance',
                    'ok':old_collision is not None and new_collision is None,
                    'old_collision':old_collision, 'new_collision':new_collision})
    for kind,planner_id in [('omni','GridBased'),('heavy','GridBased'),('heavy','RetreatGridBased')]:
        requests = [case for case in cases if case.get('robot','omni')==kind and case.get('planner','GridBased')==planner_id]
        config = yaml.safe_load((bringup / f'config/nav2_params_{kind}.yaml').read_text())
        planner = config['planner_server']['ros__parameters'][planner_id].copy()
        planner['lattice_filepath'] = planner['lattice_filepath'].replace('<bringup_share>', str(bringup))
        costmap = config['global_costmap']['global_costmap']['ros__parameters']
        with tempfile.TemporaryDirectory(prefix='queue_exit_', dir=ROOT / 'build') as tmp:
            scratch = Path(tmp)
            spec = {'map': str(map_path), 'planner': planner, 'footprint': costmap['footprint'],
                    'padding': costmap['footprint_padding'], 'scaling': costmap['inflation_layer']['cost_scaling_factor'],
                    'inflation_radius': costmap['inflation_layer']['inflation_radius'], 'reinflate_obstacles': True,
                    'cases': requests, 'output': str(scratch / 'paths.json')}
            (scratch / 'spec.yaml').write_text(yaml.safe_dump(spec))
            run = subprocess.run([str(binary), str(scratch / 'spec.yaml')], capture_output=True, text=True,
                                 env={**os.environ, 'ROS_DOMAIN_ID': '145'}, timeout=60)
            if run.returncode: raise RuntimeError(run.stderr[-3000:])
            paths = json.loads((scratch / 'paths.json').read_text())
            for request, path in zip(requests, paths):
                ok = not path['ok'] if request.get('expect_failure') else path['ok'] and path['nav2_footprint_valid']
                if path['ok']:
                    pixels = Image.open(map_path.parent / meta['image']).copy()
                    for x, y in request['obstacles']:
                        pixels.putpixel((round((x - ox) / res - .5), pixels.height - 1 - round((y - oy) / res - .5)), 0)
                    pixels.save(scratch / 'combined.pgm')
                    (scratch / 'combined.yaml').write_text(yaml.safe_dump({**meta, 'image': 'combined.pgm'}))
                    body = profiles['robot_types'][fleet[kind+'_1']['type']]['footprint']
                    _, collision = RectangleMap(scratch / 'combined.yaml').swept_check(
                        [request['start'], *path['poses']], body, costmap['footprint_padding'])
                    ok &= collision is None
                    if request.get('forecourt_only'):
                        ok &= all(p[1]+abs(math.sin(p[2]))*(body[0]/2+costmap['footprint_padding'])+abs(math.cos(p[2]))*(body[1]/2+costmap['footprint_padding']) < .85 for p in path['poses'])
                    if request.get('preserve_heading'):
                        drift=max(abs(math.atan2(math.sin(p[2]-request['start'][2]),math.cos(p[2]-request['start'][2])))
                                  for p in path['poses'])
                        # A short S-curve accommodates the lateral bay offset;
                        # reject a turn-around while allowing that valid adjustment.
                        terminal_error = abs(math.atan2(
                            math.sin(path['poses'][-1][2] - request['goal'][2]),
                            math.cos(path['poses'][-1][2] - request['goal'][2])))
                        ok &= drift < .5 and terminal_error < 1e-4
                results.append({'case': request['name'], 'ok': bool(ok), 'route_found': path['ok'],
                                'planning_seconds': path['planning_seconds'], 'error': path.get('error')})
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps({'scope': __doc__, 'results': results}, indent=2) + '\n')
    for row in results: print(f"{row['case']}: {'PASS' if row['ok'] else 'FAIL'}", flush=True)
    return int(len(results) != 13 or not all(r['ok'] for r in results))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planner', type=Path, default=ROOT / 'build/hetero_fleet_navigation/scene_planner_probe')
    parser.add_argument('--report', type=Path, default=ROOT / 'build/channel_exit_report.json')
    args = parser.parse_args()
    raise SystemExit(check(args.planner, args.report))
