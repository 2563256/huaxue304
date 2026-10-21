#!/usr/bin/python3
"""Submit one scenario to a fresh physical simulation and audit its completion.

Uses ordinary task requests only; never moves robots or edits task results.
Requires the same launch process to remain alive throughout the experiment.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import rclpy
from hetero_fleet_msgs.msg import FleetStatus
from lifecycle_msgs.srv import GetState
from controller_manager_msgs.srv import ListControllers
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Bool, String
from hetero_fleet_scheduler.channel_node import odom_pose
from hetero_fleet_scheduler.static_guard import StaticBodyGuard
import yaml


def owner_running(owner,expected):
    try:
        fields=owner.read_text().split()
        return fields[2] not in ('Z','X') and fields[21]==expected
    except (FileNotFoundError,IndexError):
        return False


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario',type=Path,required=True)
    p.add_argument('--metrics-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--launch-pid',type=int,required=True)
    p.add_argument('--timeout',type=float,default=10800.)
    p.add_argument('--interval',type=float,default=8.)
    p.add_argument('--expected-mapf-mode',choices=['cbs','eecbs'])
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[1]
    expected={t['id'] for t in yaml.safe_load(args.scenario.read_text())['tasks']}
    profiles=yaml.safe_load((root/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    fleet={r['name']:r for r in profiles['fleet']}
    body_map=StaticBodyGuard(root/'src/hetero_fleet_bringup/maps/geomap.yaml',margin=0.)
    body_samples={name:0 for name in fleet}
    owner=Path(f'/proc/{args.launch_pid}/stat');owner_start=owner.read_text().split()[21]
    rclpy.init();node=rclpy.create_node('full_batch_audit')
    state={};poses={};ready={};channel={};mapf={};subs=[]
    subs.append(node.create_subscription(String,'/mapf/status',lambda m:mapf.update(json.loads(m.data)),10))
    subs.append(node.create_subscription(FleetStatus,'/fleet_status',lambda m:state.update(message_to_ordereddict(m)),10))
    subs.append(node.create_subscription(String,'/channel_reservations',lambda m:channel.update(json.loads(m.data)),10))
    for name,entry in fleet.items():
        ox,oy=entry['spawn'][:2]
        def odom(m,name=name,entry=entry):
            pose=odom_pose(m,entry['spawn'])
            poses[name]=list(pose)
            shape=profiles['robot_types'][entry['type']]['footprint']
            body_samples[name]+=1
            if not body_map.free(pose,(*shape,0.)):
                raise RuntimeError(f'Actual full physical body intersects static map: {name} {pose}')
        subs.append(node.create_subscription(Odometry,'/'+name+'/odom',odom,qos_profile_sensor_data))
        if entry['type']=='g1_humanoid':
            subs.append(node.create_subscription(Bool,'/'+name+'/locomotion_ready',lambda m,n=name:ready.update({n:m.data}),10))
    clients={f'{name}/{part}':node.create_client(GetState,f'/{name}/{part}/get_state')
             for name in fleet for part in ('planner_server','controller_server','bt_navigator')}
    drive_clients={name:node.create_client(ListControllers,f'/{name}/controller_manager/list_controllers')
        for name,r in fleet.items() if profiles['robot_types'][r['type']]['kinematics']!='legged'}
    drive_active=set();drive_pending={}
    active=set();pending={};publisher=None;started=None;last_print=None;last_save=0.;report=None
    deadline=time.monotonic()+args.timeout;boot=time.monotonic()
    trace=(args.output/'trace.jsonl').open('w')
    try:
        while time.monotonic()<deadline:
            if not owner_running(owner,owner_start):
                raise RuntimeError('Launch process changed; this is not a continuous validation run')
            rclpy.spin_once(node,timeout_sec=.05);now=time.monotonic()
            if publisher is None:
                for key,client in clients.items():
                    if key in active:continue
                    request=pending.get(key)
                    future=request[0] if request is not None else None
                    if future is not None and future.done():
                        if future.result().current_state.id==3:active.add(key)
                        pending.pop(key)
                    elif future is not None and now-request[1]>3.:
                        future.cancel();pending.pop(key)
                    if key not in active and key not in pending and client.service_is_ready():
                        pending[key]=(client.call_async(GetState.Request()),now)
                for name,client in drive_clients.items():
                    if name in drive_active:continue
                    request=drive_pending.get(name)
                    future=request[0] if request is not None else None
                    if future is not None and future.done():
                        response=future.result()
                        states={c.name:c.state for c in response.controller} if response else {}
                        kin=profiles['robot_types'][fleet[name]['type']]['kinematics']
                        drive='diff_drive_controller' if kin=='diff' else 'mecanum_drive_controller'
                        if states.get(drive)=='active' and states.get('joint_state_broadcaster')=='active':drive_active.add(name)
                        drive_pending.pop(name)
                    elif future is not None and now-request[1]>3.:
                        future.cancel();drive_pending.pop(name)
                    if name not in drive_active and name not in drive_pending and client.service_is_ready():
                        drive_pending[name]=(client.call_async(ListControllers.Request()),now)
                if len(active)==len(clients) and len(drive_active)==len(drive_clients) and state and len(poses)==len(fleet) and ready and all(ready.values()):
                    try:existing=json.loads((args.metrics_dir/'metrics.json').read_text())
                    except (FileNotFoundError,json.JSONDecodeError):continue
                    if existing['summary']['tasks_submitted'] or state['tasks_completed'] or state['queue']:
                        raise RuntimeError('Fresh run required: scheduler already contains tasks')
                    started=now
                    cmd=[sys.executable,'-m','hetero_fleet_scheduler.task_publisher','--scenario',str(args.scenario),'--interval',str(args.interval)]
                    output=(args.output/'task-publisher.log').open('w')
                    publisher=subprocess.Popen(cmd,stdout=output,stderr=subprocess.STDOUT);output.close()
                    print('All twelve Nav2 lifecycle nodes and three wheel drives active; submitting fresh batch',flush=True)
                elif now-boot>300.:raise RuntimeError('Navigation/biped readiness timeout')
            if now-last_save>=2.:
                last_save=now
                snapshot=dict(wall_since_submit_sec=round(now-started,3) if started else None,
                    active_lifecycle_nodes=sorted(active),active_drive_robots=sorted(drive_active),locomotion_ready=ready,
                    fleet=state,poses=poses,channel=channel,mapf=mapf)
                (args.output/'progress.json').write_text(json.dumps(snapshot,indent=2))
                trace.write(json.dumps(snapshot)+'\n');trace.flush()
                key=(state.get('tasks_completed'),state.get('tasks_active'),state.get('tasks_pending'))
                if key!=last_print:
                    print('completed/active/pending:',key,flush=True);last_print=key
            if publisher is None:continue
            if publisher.poll() not in (None,0):raise RuntimeError('Task publisher failed')
            try:data=json.loads((args.metrics_dir/'metrics.json').read_text())
            except (FileNotFoundError,json.JSONDecodeError):continue
            if data['summary']['tasks_failed']:raise RuntimeError('Task failure: preserve run for diagnosis')
            completed={r['task_id'] for r in data['tasks'] if r.get('completed_at_sec') not in ('',None)}
            if completed==expected and data['summary']['tasks_completed']==len(expected) and not state.get('tasks_active') and not state.get('tasks_pending'):
                assert publisher.poll()==0
                if args.expected_mapf_mode and (mapf.get('mode') != args.expected_mapf_mode or mapf.get('requests',0)<10):
                    raise RuntimeError('Missing expected CBS/EECBS runtime evidence')
                report=dict(passed=True,scope='Single fresh Gazebo batch; no scheduler restart, task re-ID or result merging',
                    launch_pid=args.launch_pid,launch_start_ticks=owner_start,
                    scenario=str(args.scenario.resolve()),scenario_sha256=hashlib.sha256(args.scenario.read_bytes()).hexdigest(),
                    active_drive_robots=sorted(drive_active),interval_wall_sec=args.interval,wall_to_report_sec=round(now-started,3),
                    ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),gz_partition=os.environ.get('GZ_PARTITION'),metrics=data,
                    completed_task_ids=sorted(completed),live_status=state,live_poses=poses,mapf=mapf,
                    physical_body_audit=dict(collision_samples=0,pose_samples=body_samples,scope="Actual planar physical footprints without optional planner padding"))
                (args.output/'result.json').write_text(json.dumps(report,indent=2))
                print('FULL_BATCH_PASS',json.dumps(data['summary']),flush=True)
                break
        else:raise RuntimeError('Full batch validation timeout')
    except Exception as exc:
        (args.output/'failure.json').write_text(json.dumps(dict(passed=False,error=str(exc),live_status=state,live_poses=poses),indent=2))
        raise
    finally:
        trace.close();node.destroy_node();rclpy.try_shutdown()
        if publisher is not None and publisher.poll() is None:publisher.terminate()


if __name__=='__main__':main()
