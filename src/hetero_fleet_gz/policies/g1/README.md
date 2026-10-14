# G1 locomotion policy

Source: https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/pre_train/g1/motion.pt
Deployment observation and PD gains: https://github.com/unitreerobotics/unitree_rl_gym/blob/main/deploy/deploy_mujoco/configs/g1.yaml
BSD-3-Clause; see LICENSE. Retrieved 2026-10-10.

`motion.pt` is the unmodified official TorchScript artifact; `tools/export_g1_policy.py` reproduces the native weights.

`motion.bin` contains the official TorchScript weights exported as little-endian float32:
magic G1LSTM01, weight_ih (256x47), weight_hh (256x64), bias_ih (256), bias_hh (256), actor.0.weight (32x64), actor.0.bias (32), actor.2.weight (12x32), actor.2.bias (12).
LSTM gates use PyTorch i,f,g,o ordering; actor activation is ELU. Hidden/cell states start at zero. There are no extra policy training or modified weights.
SHA256: `650f4a1afb6a75167db8286d6d93fb35a5b764e840b74bfbcc1e91d7eeb40a31`.

Gazebo native Eigen inference needs no Torch at runtime. The recurrence was compared against official Torch inference for 100 steps (maximum absolute error 8.99e-6).

At geometric scale s=0.90, mass s^3, inertia s^5, torque and proportional gains s^4, derivative gains s^4.5, policy period and gait phase time sqrt(s), measured joint/angular velocity sqrt(s), linear command /sqrt(s), and angular command sqrt(s) preserve dynamic similarity under gravity. The original 23-DOF mesh model remains; its upper-body joints are fixed at the training pose, leaving twelve leg joints under torque control. Position/yaw hold modifies only policy commands; no planar base force, velocity setter or pose override is used.
