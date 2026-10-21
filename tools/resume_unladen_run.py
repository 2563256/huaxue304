#!/usr/bin/python3
"""Start a separate recovery batch from a saved, unladen fleet snapshot.

Original results are retained. In-flight/failed jobs get new IDs and creation
times; this batch is not a continuous-run benchmark. Run only after cancelling
old navigation goals and stopping the old scheduler, while retaining Gazebo.
"""
import argparse
import json
import os
from pathlib import Path
import time
import rclpy
from rclpy.clock import Clock,ClockType
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from hetero_fleet_scheduler.fleet_manager import FleetManager
from hetero_fleet_scheduler.task_model import RobotStatus


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint',type=Path,required=True)
    parser.add_argument('--original-metrics',type=Path,required=True)
    parser.add_argument('--scenario',type=Path,required=True)
    parser.add_argument('--metrics-dir',type=Path,required=True)
    parser.add_argument('--retry',action='append',default=[])
    parser.add_argument('--owner-pid',type=int)
    args=parser.parse_args()
    snapshot=json.loads(args.checkpoint.read_text())
    if any(r['current_payload_kg']>0 for r in snapshot['robots']):
        raise SystemExit('Loaded tasks require a load-preserving handoff; refusing reset')
    old=json.loads(args.original_metrics.read_text())
    completed={t['task_id'] for t in old['tasks'] if t.get('completed_at_sec') not in ('',None)}
    queue={t['id'] for t in snapshot['queue']}
    interrupted={r['current_task_id'] for r in snapshot['robots'] if r['current_task_id']}
    required=(queue|interrupted|set(args.retry))-completed
    if not required:raise SystemExit('No unfinished jobs to recover')
    args.metrics_dir.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','metrics_dir:='+str(args.metrics_dir.resolve())])
    node=FleetManager();executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node)
    try:
        until=time.monotonic()+10.
        while time.monotonic()<until and (node._now()<=0 or node._dispatch_scene() is None):
            executor.spin_once(timeout_sec=.05)
        if node._now()<=0 or node._dispatch_scene() is None:raise RuntimeError('No fresh live fleet poses')
        for row in snapshot['robots']:
            node.robots[row['id']].battery_pct=row['battery_pct']
            node.robots[row['id']].status=RobotStatus.IDLE
        provenance=[]
        for task in node._load_tasks(str(args.scenario)):
            if task.id not in required:continue
            original=task.id
            task.id=('retry_' if original in args.retry else 'resume_')+original
            task.created_at_sec=node._now()
            provenance.append(dict(original_id=original,recovery_id=task.id,created_at_sec=task.created_at_sec))
            node.submit_task(task)
        if len(provenance)!=len(required):raise RuntimeError('Checkpoint references unknown scenario tasks')
        (args.metrics_dir/'recovery_provenance.json').write_text(json.dumps(dict(
            original_metrics=str(args.original_metrics.resolve()),
            note='New recovery batch with fresh task IDs/timestamps; original failures are preserved.',tasks=provenance),indent=2))
        if args.owner_pid:
            stat=Path(f'/proc/{args.owner_pid}/stat')
            owner_start=stat.read_text().split()[21]
            clock=Clock(clock_type=ClockType.STEADY_TIME)
            def watch_owner():
                if not stat.exists() or stat.read_text().split()[21]!=owner_start:
                    node.get_logger().info('Original launch stopped; stopping recovery scheduler')
                    rclpy.try_shutdown()
            node.create_timer(1.,watch_owner,clock=clock)
        print('Recovery scheduler PID',os.getpid(),'tasks',provenance,flush=True)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():node._report()
        executor.shutdown(timeout_sec=2.)
        node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
