#!/usr/bin/env python3
"""Prepare Nav2-generated diff/omni control sets for swept-body State Lattice.

Generate inputs with Navigation2's Apache-2.0 lattice_primitives utility, using
motion_model diff / omni, grid_resolution .05, turning_radius .5,
stopping_threshold 5, num_of_headings 64. This preparation includes the origin,
fixes small in-place rotation endpoints and adds yaw-preserving omni reverse.
"""
import argparse
import copy
import json
import math
from pathlib import Path


def prepare(diff, omni):
    if diff['lattice_metadata']['heading_angles'] != omni['lattice_metadata']['heading_angles']:
        raise ValueError('Diff and omni inputs must use identical lattice headings')
    for key in ['grid_resolution', 'turning_radius']:
        if diff['lattice_metadata'][key] != omni['lattice_metadata'][key]:
            raise ValueError('Diff and omni inputs must use identical resolution and radius')
    output = {}
    for model, source in [('diff', diff), ('omni', omni)]:
        data = copy.deepcopy(source)
        if data['lattice_metadata']['motion_model'] != model:
            raise ValueError(f'Expected {model} control set')
        if model == 'omni':
            additions = []
            for primitive in data['primitives']:
                if primitive['trajectory_length'] > 0 and primitive['start_angle_index'] == primitive['end_angle_index']:
                    reverse = copy.deepcopy(primitive)
                    reverse['poses'] = [[-x, -y, yaw] for x, y, yaw in primitive['poses']]
                    additions.append(reverse)
            additions.extend(copy.deepcopy(p) for p in diff['primitives'] if p['arc_length'] > 0)
            data['primitives'].extend(additions)
        angles = data['lattice_metadata']['heading_angles']
        seen, primitives = set(), []
        for primitive in sorted(data['primitives'], key=lambda p: p['start_angle_index']):
            start, end = primitive['start_angle_index'], primitive['end_angle_index']
            if primitive['trajectory_length'] < 1e-6:
                primitive['poses'] = [[0., 0., angles[start]], [0., 0., angles[end]]]
            elif math.hypot(*primitive['poses'][0][:2]) > 1e-6:
                primitive['poses'].insert(0, [0., 0., angles[start]])
            key = start, end, json.dumps(primitive['poses'], separators=(',', ':'))
            if key in seen:
                continue
            seen.add(key)
            primitive['trajectory_id'] = len(primitives)
            primitives.append(primitive)
        data['primitives'] = primitives
        data['lattice_metadata']['number_of_trajectories'] = len(primitives)
        data['fleet_preparation'] = {'origin_included': True, 'rotation_endpoints_corrected': True,
                                     'duplicate_poses_removed': True}
        output[model] = data
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--diff-source', type=Path, required=True)
    parser.add_argument('--omni-source', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    data = prepare(json.loads(args.diff_source.read_text()), json.loads(args.omni_source.read_text()))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for model, control_set in data.items():
        path = args.output_dir / f'{model}_{control_set["lattice_metadata"]["num_of_headings"]}.json'
        path.write_text(json.dumps(control_set, separators=(',', ':')) + '\n')
        print(f'{path}: {len(control_set["primitives"])} primitives')


if __name__ == '__main__':
    main()
