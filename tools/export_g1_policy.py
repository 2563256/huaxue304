#!/usr/bin/env python3
"""Export the vendored official TorchScript weights (Torch required only here)."""
from pathlib import Path
import struct
import torch

ROOT = Path(__file__).resolve().parents[1]
POLICIES = ROOT / 'src/hetero_fleet_gz/policies/g1'
policy = torch.jit.load(str(POLICIES / 'motion.pt'), map_location='cpu')
weights = policy.state_dict()
keys = ['memory.weight_ih_l0', 'memory.weight_hh_l0',
        'memory.bias_ih_l0', 'memory.bias_hh_l0',
        'actor.0.weight', 'actor.0.bias', 'actor.2.weight', 'actor.2.bias']
with (POLICIES / 'motion.bin').open('wb') as stream:
    stream.write(b'G1LSTM01')
    for key in keys:
        values = weights[key].flatten().tolist()
        stream.write(struct.pack('<' + 'f' * len(values), *values))
print('Exported official G1 LSTM weights without modifications.')
