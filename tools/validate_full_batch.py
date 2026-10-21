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
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from std_msgs.msg import Bool, String
import yaml


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--scenario',type=Path,required=True)
    p.add_argument('--metrics-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--launch-pid',type=int,required=True)
    p.add_argument('--timeout',type=float,default=10800.)
    p.add_argument('--interval',type=float,default=8.)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    root=Path(__file__).resolve().parents[1]
    expected={t['id'] for t in yaml.safe_load(args.scenario.read_text())['tasks']}
    profiles=yaml.safe_load((root/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    fleet={r['name']:r for r in profiles['fleet']}
    owner=Path(f'/proc/{args.launch_pid}/stat');owner_start=owner.read_text().split()[21]
    rclpy.init();node=rclpy.create_node('full_batch_audit')
    state={};poses={};ready={};channel={};subs=[]
    subs.append(node.create_subscription(FleetStatus,'/fleet_status',lambda m:state.update(message_to_ordereddict(m)),10))
    subs.append(node.create_subscription(String,'/channel_reservations',lambda m:channel.update(json.loads(m.data)),10))
    for name,entry in fleet.items():
        ox,oy=entry['spawn'][:2]
        def odom(m,name=name,ox=ox,oy=oy):
            poses[name]=[m.pose.pose.position.x+ox,m.pose.pose.position.y+oy]
        subs.append(node.create_subscription(Odometry,'/'+name+'/odom',odom,qos_profile_sensor_data))
        if entry['type']=='g1_humanoid':
            subs.append(node.create_subscription(Bool,'/'+name+'/locomotion_ready',lambda m,n=name:ready.update({n:m.data}),10))
    clients={f'{name}/{part}':node.create_client(GetState,f'/{name}/{part}/get_state')
             for name in fleet for part in ('planner_server','controller_server','bt_navigator')}
    active=set();pending={};publisher=None;started=None;last_print=None;last_save=0.;report=None
    deadline=time.monotonic()+args.timeout;boot=time.monotonic()
    trace=(args.output/'trace.jsonl').open('w')
    try:
        while time.monotonic()<deadline:
            if not owner.exists() or owner.read_text().split()[21]!=owner_start:
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
                if len(active)==len(clients) and state and len(poses)==len(fleet) and ready and all(ready.values()):
                    try:existing=json.loads((args.metrics_dir/'metrics.json').read_text())
                    except (FileNotFoundError,json.JSONDecodeError):continue
                    if existing['summary']['tasks_submitted'] or state['tasks_completed'] or state['queue']:
                        raise RuntimeError('Fresh run required: scheduler already contains tasks')
                    started=now
                    cmd=[sys.executable,'-m','hetero_fleet_scheduler.task_publisher','--scenario',str(args.scenario),'--interval',str(args.interval)]
                    output=(args.output/'task-publisher.log').open('w')
                    publisher=subprocess.Popen(cmd,stdout=output,stderr=subprocess.STDOUT);output.close()
                    print('All twelve Nav2 lifecycle nodes active; submitting fresh batch',flush=True)
                elif now-boot>300.:raise RuntimeError('Navigation/biped readiness timeout')
            if now-last_save>=2.:
                last_save=now
                snapshot=dict(wall_since_submit_sec=round(now-started,3) if started else None,
                    active_lifecycle_nodes=sorted(active),locomotion_ready=ready,
                    fleet=state,poses=poses,channel=channel)
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
                report=dict(passed=True,scope='Single fresh Gazebo batch; no scheduler restart, task re-ID or result merging',
                    launch_pid=args.launch_pid,launch_start_ticks=owner_start,
                    scenario=str(args.scenario.resolve()),scenario_sha256=hashlib.sha256(args.scenario.read_bytes()).hexdigest(),
                    interval_wall_sec=args.interval,wall_to_report_sec=round(now-started,3),
                    ros_domain_id=os.environ.get('ROS_DOMAIN_ID'),metrics=data,
                    completed_task_ids=sorted(completed),live_status=state,live_poses=poses)
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
