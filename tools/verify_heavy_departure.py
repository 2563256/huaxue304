#!/usr/bin/python3
"""Native full-body replay of the observed heavy/charger-row deadlock."""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import tempfile
import yaml
from verify_navigation import RectangleMap

ROOT=Path(__file__).resolve().parents[1]


def check(binary,report):
    bringup=ROOT/'src/hetero_fleet_bringup'
    profiles=yaml.safe_load((ROOT/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    fleet={r['name']:r for r in profiles['fleet']}
    scene=json.loads((ROOT/'src/hetero_fleet_navigation/test/fixtures/heavy_exit_blocked.json').read_text())
    meta=yaml.safe_load((bringup/'maps/geomap.yaml').read_text());res=meta['resolution'];ox,oy=meta['origin'][:2]
    def peer(name,pose):
        l,w=profiles['robot_types'][fleet[name]['type']]['footprint'];l+=.24;w+=.24
        x,y,yaw=pose;c,s=math.cos(yaw),math.sin(yaw);ax,ay=abs(c)*l/2+abs(s)*w/2,abs(s)*l/2+abs(c)*w/2
        out=[]
        for i in range(math.floor((x-ax-ox)/res),math.ceil((x+ax-ox)/res)):
            for j in range(math.floor((y-ay-oy)/res),math.ceil((y+ay-oy)/res)):
                dx,dy=ox+(i+.5)*res-x,oy+(j+.5)*res-y;extra=res/2*(abs(c)+abs(s))
                if abs(c*dx+s*dy)<=l/2+extra and abs(-s*dx+c*dy)<=w/2+extra:out.append([x+dx,y+dy])
        return out
    cases=[]
    for name,moves,success in [('blocked',{},False),
            ('light_only',{'light_1':fleet['light_1']['idle_clearance']},False),
            ('g1_only',{'g1_1':fleet['g1_1']['idle_clearance']},False),
            ('both_clear',{'light_1':fleet['light_1']['idle_clearance'],'g1_1':fleet['g1_1']['idle_clearance']},True),
            # This simplified native scene still admits a heavy route. Do not
            # equate a missed clearance maneuver with universal infeasibility.
            ('g1_near_standby',{'light_1':fleet['light_1']['idle_clearance'],
                'omni_1':fleet['omni_1']['idle_clearance'],'g1_1':[1.22,-.84,0.]},True),
            ('g1_precise_clearance',{'light_1':fleet['light_1']['idle_clearance'],
                'omni_1':fleet['omni_1']['idle_clearance'],'g1_1':fleet['g1_1']['idle_clearance']},True)]:
        poses={**scene,**moves}
        cases.append(dict(name=name,start=scene['heavy_1'],goal=[-2.,4.22,0.],expect_success=success,
                          obstacles=sum([peer(n,p) for n,p in poses.items() if n!='heavy_1'],[])))
    config=yaml.safe_load((bringup/'config/nav2_params_heavy.yaml').read_text())
    planner=config['planner_server']['ros__parameters']['TaskGridBased'].copy()
    planner['lattice_filepath']=planner['lattice_filepath'].replace('<bringup_share>',str(bringup))
    cost=config['global_costmap']['global_costmap']['ros__parameters']
    with tempfile.TemporaryDirectory(prefix='heavy_exit_',dir=ROOT/'build') as tmp:
        tmp=Path(tmp)
        spec=dict(map=str(bringup/'maps/geomap.yaml'),planner=planner,footprint=cost['footprint'],
            padding=cost['footprint_padding'],scaling=cost['inflation_layer']['cost_scaling_factor'],
            inflation_radius=cost['inflation_layer']['inflation_radius'],reinflate_obstacles=True,
            cases=cases,output=str(tmp/'paths.json'))
        (tmp/'spec.yaml').write_text(yaml.safe_dump(spec))
        result=subprocess.run([str(binary),str(tmp/'spec.yaml')],env={**os.environ,'ROS_DOMAIN_ID':'145'},
                              capture_output=True,text=True,timeout=90)
        if result.returncode:raise RuntimeError(result.stderr[-3000:])
        paths=json.loads((tmp/'paths.json').read_text())
        summary=[]
        for case,path in zip(cases,paths):
            ok=bool(path['ok'])==case['expect_success']
            if path['ok']:
                ok &= bool(path['nav2_footprint_valid'])
                # Native probe already checks the combined peer+static map.
                body=profiles['robot_types'][fleet['heavy_1']['type']]['footprint']
                _,collision=RectangleMap(bringup/'maps/geomap.yaml').swept_check(path['poses'],body,cost['footprint_padding'])
                ok &= collision is None
            summary.append(dict(case=case['name'],ok=bool(ok),route_found=path['ok'],error=path.get('error')))
        report.parent.mkdir(parents=True,exist_ok=True)
        report.write_text(json.dumps(dict(scope=__doc__,results=summary),indent=2))
        for row in summary:print(row)
        return int(not all(r['ok'] for r in summary))


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--planner',type=Path,default=ROOT/'build/hetero_fleet_navigation/scene_planner_probe')
    p.add_argument('--report',type=Path,default=ROOT/'build/heavy_departure_report.json');args=p.parse_args()
    raise SystemExit(check(args.planner,args.report))
