#!/usr/bin/python3
"""Resume docking after a completed batch without resetting its task results.

Cancel old Nav2 goals and stop the previous scheduler before using this tool.
It accepts only an empty, unladen queue with no assigned transport tasks.
"""
import argparse
import json
from pathlib import Path
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.clock import Clock, ClockType
from hetero_fleet_scheduler.fleet_manager import FleetManager
from hetero_fleet_scheduler.task_model import Pose2D, RobotStatus, TaskState


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--metrics',type=Path,required=True)
    p.add_argument('--scenario',type=Path,required=True)
    p.add_argument('--provenance',type=Path,required=True)
    p.add_argument('--owner-pid',type=int,required=True)
    args=p.parse_args()
    snapshot=json.loads(args.checkpoint.read_text());results=json.loads(args.metrics.read_text())
    if snapshot['queue'] or any(r['current_task_id'] or r['current_payload_kg']>0 for r in snapshot['robots']):
        raise SystemExit('Only completed, unladen batches may resume docking')
    summary=results['summary'];rows=results['tasks']
    if (summary['tasks_submitted']!=summary['tasks_completed'] or summary['tasks_failed']
            or len(rows)!=summary['tasks_completed'] or any(t['completed_at_sec'] in ('',None) for t in rows)):
        raise SystemExit('Task results are not a fully completed batch')
    mapping={t['recovery_id']:t['original_id'] for t in json.loads(args.provenance.read_text())['tasks']}
    if set(mapping)!={t['task_id'] for t in rows}:
        raise SystemExit('Provenance does not match completed results')
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true','-p','metrics_dir:='+str(args.metrics.parent.resolve())])
    node=FleetManager();executor=MultiThreadedExecutor(num_threads=6);executor.add_node(node)
    try:
        # Restore results before spinning timers that publish/write metrics.
        tasks={t.id:t for t in node._load_tasks(str(args.scenario))}
        for row in rows:
            task=tasks[mapping[row['task_id']]];task.id=row['task_id']
            task.state=TaskState.COMPLETED;task.assigned_robot=row['robot_id']
            task.created_at_sec=row['created_at_sec'];task.assigned_at_sec=row['assigned_at_sec']
            task.completed_at_sec=row['completed_at_sec'];task.requeue_count=row['requeue_count']
            node.tasks[task.id]=task
        node.metrics.rows=rows;node.metrics.submitted=summary['tasks_submitted']
        m=node.metrics.m;m.tasks_completed=summary['tasks_completed'];m.tasks_failed=summary['tasks_failed']
        m.first_task_at_sec=min(t['created_at_sec'] for t in rows)
        m.last_completion_at_sec=max(t['completed_at_sec'] for t in rows)
        m.completion_times=[t['completion_time_sec'] for t in rows]
        m.reschedule_count=summary['reschedule_count'];m.tardy_tasks=summary['tardy_tasks']
        for row in snapshot['robots']:
            robot=node.robots[row['id']];saved=summary['per_robot'][robot.id]
            robot.status=RobotStatus.IDLE;robot.pose=Pose2D(row['pose']['x'],row['pose']['y'],row['pose']['theta'])
            robot.battery_pct=row['battery_pct'];robot.tasks_completed=row['tasks_completed']
            robot.odom_distance_m=row['odom_distance_m'];robot.energy_consumed_wh=row['energy_consumed_wh']
            m.robot_busy_sec[robot.id]=saved['busy_sec']
        stat=Path(f'/proc/{args.owner_pid}/stat');owner_start=stat.read_text().split()[21]
        def owner_stopped():
            if not stat.exists() or stat.read_text().split()[21]!=owner_start:rclpy.try_shutdown()
        node.create_timer(1.,owner_stopped,clock=Clock(clock_type=ClockType.STEADY_TIME))
        print('Restored completed batch:',summary['tasks_completed'],'tasks; docking only',flush=True)
        executor.spin()
    except (KeyboardInterrupt,ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():node._report()
        executor.shutdown(timeout_sec=2.);node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
