#!/usr/bin/python3
"""Offline copy-A scheduling check. NOT a Gazebo makespan measurement."""
from pathlib import Path
import json
import time
import yaml
from hetero_fleet_scheduler.route_estimate import RouteEstimator
from hetero_fleet_scheduler.joint_schedule import JointScheduler
from hetero_fleet_scheduler.task_model import Task,RobotState,Pose2D


def main():
    root=Path(__file__).resolve().parents[1]
    profiles=yaml.safe_load((root/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    channels=yaml.safe_load((root/'src/hetero_fleet_scheduler/config/channels.yaml').read_text())
    scenario=yaml.safe_load((root/'src/hetero_fleet_scheduler/config/tasks/demo_10.yaml').read_text())
    robots=[]
    for f in profiles['fleet']:
        p=profiles['robot_types'][f['type']]
        robots.append(RobotState(f['name'],type=f['type'],pose=Pose2D(*f['spawn']),
            max_speed=p['max_speed'],payload_capacity_kg=p['payload_kg'],
            capabilities=p['capabilities'],battery_capacity_wh=p['battery_capacity_wh'],
            wh_per_meter_empty=p['wh_per_meter_empty'],wh_per_meter_kg=p['wh_per_meter_kg']))
    tasks=[Task(t['id'],pickup=Pose2D(*t['pickup']),dropoff=Pose2D(*t['dropoff']),
        payload_kg=t['payload_kg'],required_capability=t['required_capability'],
        priority=t['priority']) for t in scenario['tasks']]
    estimator=RouteEstimator(root/'src/hetero_fleet_bringup/maps/geomap.yaml',profiles,channels)
    started=time.monotonic()
    solver=JointScheduler(estimator,budget=2.,service=10.,horizon=1800.)
    plan=solver.solve(tasks,robots,100.)
    report=dict(scope='Offline static-map estimates plus CP-SAT; no Gazebo/MPPI tracking; no physical performance claim.',
                total_wall_sec=time.monotonic()-started, effective_groups=estimator.regions,
                plan=plan.as_dict())
    (root/'work/joint_schedule_check.json').write_text(json.dumps(report,indent=2))
    print(plan.status,'planned',len(plan.rows),'unscheduled',plan.unscheduled,
          'wall_seconds',round(report['total_wall_sec'],3))
    if plan.status not in ('OPTIMAL','FEASIBLE') or len(plan.rows)!=len(tasks):
        raise SystemExit(1)
    for rid in [r.id for r in robots]:
        rows=[r for r in plan.rows if r['robot_id']==rid]
        assert all(a['end']<=b['start'] for a,b in zip(rows,rows[1:]))
    for group in estimator.regions:
        rows=sorted([r for r in plan.rows if group in r['groups']],key=lambda r:r['start'])
        assert all(a['end']<=b['start'] for a,b in zip(rows,rows[1:]))
    print('Robot sequencing and shared-resource exclusions verified.')


if __name__=='__main__': main()
