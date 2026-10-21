#!/usr/bin/python3
"""Native full-body plans and velocity guard must use the same body envelope."""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import yaml
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/hetero_fleet_scheduler'))
from hetero_fleet_scheduler.static_guard import StaticBodyGuard


def main():
    p=argparse.ArgumentParser();p.add_argument('--planner',type=Path,required=True);args=p.parse_args()
    bringup=ROOT/'src/hetero_fleet_bringup';profiles=yaml.safe_load((ROOT/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    guard=StaticBodyGuard(bringup/'maps/geomap.yaml')
    report=[]
    for robot,kind,goal in [('light_1','light',[-7.,-.25,0.]),('heavy_1','heavy',[-2.,4.22,0.]),
                            ('omni_1','omni',[1.7,4.22,0.]),('g1_1','g1',[1.2,-1.08,0.])]:
        entry=next(r for r in profiles['fleet'] if r['name']==robot);profile=profiles['robot_types'][entry['type']]
        config=yaml.safe_load((bringup/f'config/nav2_params_{kind}.yaml').read_text())
        planner=config['planner_server']['ros__parameters']['GridBased'].copy()
        planner['lattice_filepath']=planner['lattice_filepath'].replace('<bringup_share>',str(bringup))
        cost=config['global_costmap']['global_costmap']['ros__parameters']
        with tempfile.TemporaryDirectory(prefix='guard_native_',dir=ROOT/'build') as tmp:
            tmp=Path(tmp)
            cases=[dict(name=robot,start=entry['spawn'],goal=goal,expect_success=True)]
            if kind=='omni':
                cases.append(dict(name='observed_east_turn',start=[11.484,.249,-.003],goal=goal,expect_success=True))
            spec=dict(map=str(bringup/'maps/geomap.yaml'),planner=planner,footprint=cost['footprint'],padding=cost['footprint_padding'],
                scaling=cost['inflation_layer']['cost_scaling_factor'],inflation_radius=cost['inflation_layer']['inflation_radius'],
                cases=cases,output=str(tmp/'paths.json'))
            (tmp/'spec.yaml').write_text(yaml.safe_dump(spec))
            run=subprocess.run([str(args.planner),str(tmp/'spec.yaml')],env={**os.environ,'ROS_DOMAIN_ID':'146'},capture_output=True,text=True,timeout=90)
            if run.returncode:raise RuntimeError(run.stderr[-2000:])
            paths=json.loads((tmp/'paths.json').read_text())
            for case,row in zip(cases,paths):
                assert row['ok'] and row['nav2_footprint_valid'],row
                body=(*profile['footprint'],profile['footprint_padding']+guard.res/16)
                blocked=[(i,pose) for i,pose in enumerate(row['poses']) if not guard.free(pose,body)]
                assert not blocked,(robot,blocked[:3])
                report.append(dict(robot=robot,case=case['name'],poses=len(row['poses']),guard_matches_native=True))
    output=ROOT/'build/static_guard_native.json';output.write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
