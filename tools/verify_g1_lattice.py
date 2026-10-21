#!/usr/bin/env python3
"""Replay the observed G1 heading reversal, departure and return with full-body sweeps."""
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
    bringup = ROOT/'src/hetero_fleet_bringup'
    config = yaml.safe_load((bringup/'config/nav2_params_g1.yaml').read_text())
    profiles = yaml.safe_load((ROOT/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    entry = next(r for r in profiles['fleet'] if r['name']=='g1_1')
    body = profiles['robot_types'][entry['type']]
    costmap = config['global_costmap']['global_costmap']['ros__parameters']
    map_path = bringup/'maps/geomap.yaml'
    scene = RectangleMap(map_path)
    meta = yaml.safe_load(map_path.read_text()); res = meta['resolution']; ox, oy = meta['origin'][:2]
    fleet = {r['name']:r for r in profiles['fleet']}
    def peer(name, x, y):
        length, width = profiles['robot_types'][fleet[name]['type']]['footprint']
        length += .24; width += .24
        return [[ox+(i+.5)*res,oy+(j+.5)*res]
                for i in range(math.floor((x-length/2-ox)/res),math.ceil((x+length/2-ox)/res))
                for j in range(math.floor((y-width/2-oy)/res),math.ceil((y+width/2-oy)/res))]
    traffic = peer('light_1',-2.2,-.25)+peer('heavy_1',7.5,-.25)+peer('omni_1',10.93,-.18)
    # Last known free pose immediately before the observed orientation jump.
    start = [.31028922867266795,1.106479896581199,1.5807832514312994]
    legacy = {'plugin':'nav2_smac_planner::SmacPlanner2D', 'tolerance':0.,
              'downsample_costmap':False, 'allow_unknown':True,
              'max_iterations':1000000, 'max_on_approach_iterations':1000,
              'max_planning_time':2., 'cost_travel_multiplier':2.,
              'use_final_approach_orientation':True}
    cases = [
        ('legacy', legacy, [{'name':'reproduce_unsafe_heading_jump','start':start,'goal':[5.8,-.2,0.]}]),
        ('task', config['planner_server']['ros__parameters']['TaskGridBased'], [
            {'name':'swept_heading_reversal','start':start,'goal':[5.8,-.2,0.]},
            {'name':'reverse_from_connector','start':[.2076,2.2673,1.2783],'goal':[5.8,-.2,0.]},
            {'name':'detour_instead_of_tight_south_bypass','start':[5.7,-.2,-.4],
             'goal':[11.,4.22,0.],
             'obstacles':peer('light_1',-2.2,-.25)+peer('heavy_1',7.5,-.25)+peer('omni_1',12.05,2.4)},
            {'name':'resume_from_idle_clearance','start':entry['idle_clearance'],'goal':[-2.,4.22,0.],'obstacles':traffic}]),
        ('general', config['planner_server']['ros__parameters']['GridBased'], [
            {'name':'leave_charger','start':entry['spawn'],'goal':entry['charger_exit']},
            {'name':'clear_finished_dropoff','start':[10.9,4.28,-.1],'goal':entry['idle_clearance'],'obstacles':traffic},
            *[{'name':f'return_leg_{i}','start':a,'goal':b}
              for i,(a,b) in enumerate(zip([[11.,4.22,0.],*entry['charger_route']],
                                          [*entry['charger_route'],entry['charger']]))]])]
    results = []
    with tempfile.TemporaryDirectory(prefix='g1_turn_', dir=ROOT/'build') as tmp:
        scratch = Path(tmp)
        for group, planner, requests in cases:
            planner = dict(planner)
            if 'lattice_filepath' in planner:
                planner['lattice_filepath'] = planner['lattice_filepath'].replace('<bringup_share>',str(bringup))
            spec = {'reinflate_obstacles':True,'map':str(map_path),'planner':planner,'footprint':costmap['footprint'],
                    'padding':costmap['footprint_padding'],
                    'inflation_radius':costmap['inflation_layer']['inflation_radius'],
                    'scaling':costmap['inflation_layer']['cost_scaling_factor'],
                    'cases':requests,'output':str(scratch/'paths.json')}
            (scratch/'spec.yaml').write_text(yaml.safe_dump(spec))
            run = subprocess.run([str(binary),str(scratch/'spec.yaml')],capture_output=True,text=True,
                                 env={**os.environ,'ROS_DOMAIN_ID':'145'},timeout=120)
            if run.returncode:raise RuntimeError(run.stderr[-3000:])
            paths = json.loads((scratch/'paths.json').read_text())
            assert len(paths)==len(requests)
            for request,path in zip(requests,paths):
                poses = [request['start'],*path.get('poses',[])]
                current_scene = scene
                if request.get('obstacles'):
                    pixels = Image.open(map_path.parent/meta['image']).convert('L')
                    for x,y in request['obstacles']:
                        pixels.putpixel((round((x-ox)/res-.5),pixels.height-1-round((y-oy)/res-.5)),0)
                    pixels.save(scratch/'combined.pgm')
                    (scratch/'combined.yaml').write_text(yaml.safe_dump({**meta,'image':'combined.pgm'}))
                    current_scene = RectangleMap(scratch/'combined.yaml')
                samples,collision = current_scene.swept_check(poses,body['footprint'],body['footprint_padding'])
                ok = path['ok'] and (collision is not None if group=='legacy' else
                                     collision is None and path['nav2_footprint_valid'])
                if group!='legacy' and path['ok']:
                    ok &= all(abs(poses[-1][i]-request['goal'][i])<1e-6 for i in (0,1))
                    if request['name']=='detour_instead_of_tight_south_bypass':
                        ok &= min(p[0] for p in poses) < 1.5
                results.append({'case':request['name'],'ok':bool(ok),'collision':collision,
                                'samples':samples,'planning_seconds':path['planning_seconds'],
                                'error':path.get('error')})
    report.parent.mkdir(parents=True,exist_ok=True)
    report.write_text(json.dumps({'scope':__doc__,'results':results},indent=2)+'\n')
    for r in results:print(f"{r['case']}: {'PASS' if r['ok'] else 'FAIL'}",flush=True)
    return int(len(results)!=10 or not all(r['ok'] for r in results))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planner',type=Path,default=ROOT/'build/hetero_fleet_navigation/scene_planner_probe')
    parser.add_argument('--report',type=Path,default=ROOT/'build/g1_lattice_report.json')
    args=parser.parse_args()
    raise SystemExit(check(args.planner,args.report))
