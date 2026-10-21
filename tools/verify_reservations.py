#!/usr/bin/env python3
"""Replay three real State Lattice missions through the reservation core.

Uses native Nav2 plans, profile-sized bodies and per-robot speed limits.
Each replay uses identical routes and a 0.1 s clock; tracking is idealized.
Verifies admission before entry, exclusive occupancy, eventual completion and physical release. This is
an offline reservation replay; it does not simulate MPPI, robot-to-robot
collisions outside the configured channels, or Gazebo tracking.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import copy
import yaml
from verify_navigation import ROOT, RectangleMap
sys.path.insert(0, str(ROOT / 'src/hetero_fleet_scheduler'))
from hetero_fleet_scheduler.reservations import ChannelReservations, intersects


def samples(poses, radius):
    result = [poses[0]]
    for a, b in zip(poses, poses[1:]):
        turn = math.atan2(math.sin(b[2] - a[2]), math.cos(b[2] - a[2]))
        steps = max(1, math.ceil((math.dist(a[:2], b[:2]) + radius * abs(turn)) / .025))
        for k in range(1, steps + 1):
            t = k / steps
            result.append((a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + turn * t))
    return result


def replay(config, bodies, spawn_poses, missions):
    poses = dict(spawn_poses)
    reservations = ChannelReservations(config, bodies)
    progress = {name: [0, 0] for name in missions}
    for name, legs in missions.items():
        reservations.set_active(name, True); reservations.set_path(name, legs[0])
    completed, events, waits = set(), [], {name: 0 for name in missions}
    completion_times, step_times, cpu_times = {}, [], []
    credit = {name: 0. for name in missions}
    owners = {group: None for group in reservations.regions}
    for frame in range(12000):
        now = frame / 10
        for name, pose in poses.items(): reservations.observe(name, pose, now)
        tick = time.perf_counter(); cpu_start = time.process_time()
        decisions, state = reservations.step(now, now)
        step_times.append(time.perf_counter() - tick)
        cpu_times.append(time.process_time() - cpu_start)
        for group, data in state['groups'].items():
            if data['owner'] != owners[group]:
                events.append({'time': now, 'group': group, 'from': owners[group], 'to': data['owner']})
                owners[group] = data['owner']
        for name, legs in missions.items():
            if name in completed: continue
            if not decisions[name]['allowed']:
                waits[name] += 1; credit[name] = 0.; continue
            leg, point = progress[name]
            credit[name] += .1
            if point + 1 < len(legs[leg]):
                linear, angular = config['robot_motion_limits'][name]
                while point + 1 < len(legs[leg]):
                    next_pose = legs[leg][point + 1]
                    current = poses[name]
                    turn = math.atan2(math.sin(next_pose[2] - current[2]), math.cos(next_pose[2] - current[2]))
                    duration = max(math.dist(current[:2], next_pose[:2]) / linear, abs(turn) / angular, 1e-6)
                    if credit[name] + 1e-9 < duration: break
                    l, w, padding = bodies[name]
                    for group, bounds in reservations.regions.items():
                        if any(intersects(next_pose, (l, w), padding, b) for b in bounds):
                            assert state['groups'][group]['owner'] == name, f'{name}: entry without booking'
                    poses[name] = next_pose; point += 1; progress[name][1] = point
                    credit[name] -= duration
            else:
                reservations.set_active(name, False)
                credit[name] = 0.
                if leg + 1 < len(legs):
                    progress[name] = [leg + 1, 0]
                    reservations.set_active(name, True); reservations.set_path(name, legs[leg + 1])
                else:
                    completed.add(name)
                    completion_times[name] = now
        for group, bounds in reservations.regions.items():
            occupants = []
            for name, pose in poses.items():
                l, w, padding = bodies[name]
                if any(intersects(pose, (l, w), padding, b) for b in bounds): occupants.append(name)
            assert len(occupants) <= 1, f'{group}: simultaneous body occupancy {occupants}'
        if len(completed) == len(missions): break
    assert len(completed) == len(missions), f'Incomplete missions: {set(missions) - completed}'
    for name, pose in poses.items(): reservations.observe(name, pose, now + .1)
    _, state = reservations.step(now + .1, now + .1)
    assert all(group['owner'] is None and not group['queue'] and not group['occupants'] for group in state['groups'].values())
    return {'completed_missions': sorted(completed), 'frames': frame + 1,
            'waiting_frames': waits, 'ownership_events': events,
            'completion_seconds': completion_times, 'sum_completion_seconds': sum(completion_times.values()),
            'step_cpu_seconds': sum(cpu_times), 'step_compute_wall_seconds': sum(step_times), 'step_p95_ms': sorted(step_times)[int(.95 * len(step_times))] * 1000.,
            'final_state': state, 'ok': True}

def check(binary, report_path):
    profiles = yaml.safe_load((ROOT / 'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    config = yaml.safe_load((ROOT / 'src/hetero_fleet_scheduler/config/channels.yaml').read_text())
    tasks = yaml.safe_load((ROOT / 'src/hetero_fleet_scheduler/config/tasks/demo_10.yaml').read_text())['tasks']
    task_index = {'light_1': 2, 'heavy_1': 0, 'omni_1': 1}
    bodies = {r['name']: (*profiles['robot_types'][r['type']]['footprint'],
                          profiles['robot_types'][r['type']]['footprint_padding']) for r in profiles['fleet']}
    poses = {r['name']: tuple(r['spawn']) for r in profiles['fleet']}
    missions, planner_results = {}, []
    scene = RectangleMap(ROOT / 'src/hetero_fleet_bringup/maps/geomap.yaml')
    with tempfile.TemporaryDirectory(prefix='channel_replay_', dir=ROOT / 'build') as temp:
        scratch = Path(temp)
        for entry in profiles['fleet']:
            name = entry['name']
            if name not in task_index: continue
            task = tasks[task_index[name]]
            waypoints = [entry['spawn'], entry['charger_exit'], task['pickup'], task['dropoff'], entry['charger']]
            kind = name.split('_')[0]
            params = yaml.safe_load((ROOT / f'src/hetero_fleet_bringup/config/nav2_params_{kind}.yaml').read_text())
            planner = params['planner_server']['ros__parameters']['GridBased'].copy()
            planner['lattice_filepath'] = planner['lattice_filepath'].replace('<bringup_share>', str(ROOT / 'src/hetero_fleet_bringup'))
            costmap = params['global_costmap']['global_costmap']['ros__parameters']
            cases = [{'name': f'{name}:{i}', 'start': start, 'goal': goal} for i, (start, goal) in enumerate(zip(waypoints, waypoints[1:]))]
            spec = {'map': str(ROOT / 'src/hetero_fleet_bringup/maps/geomap.yaml'), 'planner': planner,
                    'footprint': costmap['footprint'], 'padding': costmap['footprint_padding'],
                    'scaling': costmap['inflation_layer']['cost_scaling_factor'],
                    'inflation_radius': costmap['inflation_layer']['inflation_radius'],
                    'cases': cases, 'output': str(scratch / 'paths.json')}
            spec_path = scratch / 'spec.yaml'; spec_path.write_text(yaml.safe_dump(spec))
            run = subprocess.run([str(binary), str(spec_path)], capture_output=True, text=True,
                                 env={**os.environ, 'ROS_DOMAIN_ID': '139'}, timeout=120)
            if run.returncode: raise RuntimeError(run.stderr[-2500:])
            missions[name] = []
            for request, path in zip(cases, json.loads((scratch / 'paths.json').read_text())):
                if not path['ok'] or not path['nav2_footprint_valid']:
                    raise AssertionError(f'{request["name"]}: invalid native path {path.get("error", "")}')
                l, w, padding = bodies[name]
                full_path = [request['start'], *path['poses'], request['goal']]
                _, collision = scene.swept_check(full_path, (l, w), padding)
                if collision: raise AssertionError(f'{request["name"]}: static body collision {collision}')
                missions[name].append(samples(full_path, math.hypot(l / 2 + padding, w / 2 + padding)))
                planner_results.append({'name': request['name'], 'ok': True, 'planning_seconds': path['planning_seconds']})
    config['robot_motion_limits'] = {
        r['name']: (profiles['robot_types'][r['type']]['max_speed'],
                    profiles['robot_types'][r['type']]['max_omega']) for r in profiles['fleet']}
    strategies = {}
    for mode in ('fifo', 'sipp_pbs'):
        strategy = copy.deepcopy(config); strategy['scheduling'] = mode
        strategies[mode] = replay(strategy, bodies, poses, missions)
    report = {'scope': __doc__, 'native_plans': len(planner_results), 'planner_results': planner_results,
              'comparison': strategies, 'ok': True, **strategies[config.get('scheduling', 'fifo')]}
    report_path.parent.mkdir(parents=True, exist_ok=True)
    # Persist identical native routes for repeatable timing-only comparisons.
    (report_path.parent / 'channel_native_routes.json').write_text(json.dumps(
        {'config': config, 'bodies': bodies, 'spawn_poses': poses, 'missions': missions}))
    report_path.write_text(json.dumps(report, indent=2) + '\n')
    print(f'PASS: {len(planner_results)} native plans; FIFO and SIPP/PBS complete all missions with exclusive entry and physical release')
    for mode, result in strategies.items():
        print(f"{mode}: {result['frames']} frames, sum completion {result['sum_completion_seconds']:.1f}s, arbiter CPU {result['step_cpu_seconds']:.3f}s")
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--planner', type=Path, default=ROOT / 'build/hetero_fleet_navigation/scene_planner_probe')
    parser.add_argument('--report', type=Path, default=ROOT / 'build/channel_replay_report.json')
    args = parser.parse_args()
    return check(args.planner, args.report)


if __name__ == '__main__':
    raise SystemExit(main())
