# 副本A：联合排程与滚动通道协调

原项目位于 `/home/ubuntu/huaxue304`；本副本位于 `/home/ubuntu/huaxue304_A`。
复制时保留当时已有的全部源码和未提交修改，不复制 `.git`、build/install/log、旧指标与运行生成文件。
原始文件校验值见 `ORIGINAL_MANIFEST.json`。构建、依赖与输出均位于副本自己的目录。

## 启动

本机已安装副本的依赖并完成编译。新终端执行：

```bash
cd /home/ubuntu/huaxue304_A
source setup_A.bash
ros2 launch hetero_fleet_bringup bringup.launch.py
```

每个终端都需要 `source setup_A.bash`，因为 OR-Tools 安装在副本的 `.venv`，
而 ROS 节点仍使用系统 Python。默认 ROS_DOMAIN_ID=43、GZ_PARTITION=huaxue304_A，
避免与原项目互相发现。若修改域，所有相关终端必须一致。

导航就绪后，另开终端发送任务：

```bash
cd /home/ubuntu/huaxue304_A
source setup_A.bash
ros2 run hetero_fleet_scheduler task_publisher \
  --scenario src/hetero_fleet_scheduler/config/tasks/demo_10.yaml --interval 8
```

无界面运行：启动命令末尾添加 `headless:=true rviz:=false`。Ctrl+C 停止。
若迁移到其他路径/机器，在具备 ROS Jazzy 与原项目依赖的环境中执行 `bash bootstrap_A.sh`，
重新建立本地依赖并构建；不要复制旧 build/install 或旧虚拟环境直接运行。

## 实现与职责

| 层级 | 副本A的实现 | 决策权限 |
|---|---|---|
| 任务排程 | `joint_schedule.py`：OR-Tools CP-SAT，选车型、任务序列、时间区间 | 仅建议未执行任务 |
| 滚动更新 | `rolling_schedule.py`：任务/通道资源时域，周期与事件重规划 | 只提交每车近期首项任务；已执行目标保持冻结 |
| 交通协调 | 原 SIPP/PBS 接收 `/fleet_schedule`，使用有界紧迫度权重 | 实际路径、老请求公平性及物理占用优先 |
| 空间规划 | 原 State Lattice＋完整车身扫掠检查 | 生成最终空间路径 |
| 执行安全 | 原 MPPI、速度过滤、本地看门狗、碰撞监测 | 预测窗口不直接授予通行权 |

### CP-SAT 模型

- 能力、载荷、剩余电量及估算路径可达性约束。
- 用可选区间和每车 Circuit 序列建模，计入“上一单卸货点→下一单取货点”的空驶。
- 每台机器人任务区间互斥；每个通道资源上的任务占用包络互斥。
- 一项任务的包络包含可能的空驶路线与载货路线资源并集，**较保守**，不是精准轨迹占用窗。
- 在当前时域内先最大化可安排任务数，再最小化 Makespan，最后按优先级优化完成时间。
  超出时域、能力不符或电量不足的任务保留在队列，不会因为一项不可行任务阻止其他任务。
- 有解即使用（OPTIMAL 或 FEASIBLE），不宣称每次得到全局最优；无解/预算超限没有可行解时保持等待并重试。
- 估算使用静态栅格 A*、带 padding 的内切半径和行驶时间系数，**不替代 State Lattice**，
  不证明转弯可行，也不保证实际能耗和时间等于估计值。

### 滚动更新与反馈

默认每 5 秒 ROS 时间重排，前 2 秒内的每车首项任务可提交，规划时域为 1800 秒，最多考虑 20 项待处理任务。
任务新增、机器人状态/当前任务改变、明显通道延误、仿真时钟回退会使旧的未提交计划失效。
CP-SAT 搜索在单独工作线程执行；默认求解器预算 0.5 秒，不包含静态路线估算与模型构造耗时。
执行前重新检查实时任务状态、导航/步态就绪、取货区域、通道状态新鲜度。
实际占用者及 SIPP/PBS 时间窗反馈进入下一轮估算；预测延误不会取消当前任务或强行释放令牌。

这里实现的是 **RHCR 思路的任务/资源滚动时域框架**，不是对经典 RHCR 窗口化空间 MAPF
算法的完整复刻。空间搜索仍由 Nav2 State Lattice 执行。

### 分段与 G1

配置将北侧与三条连接段分别声明为逻辑资源，原有几何安全合并检查仍然启用。
本地图缺少已验证的中间等待空间，因此四段仍合并为同一个物理互斥资源，
资源名为 `center_connector+east_connector+north_corridor+west_connector`。
相隔足够远的资源可独立预约并行；**当前版本未启用同一资源的同向多车跟随**。
不能为了降低 Makespan 直接关闭安全合并；进一步拆分需要先验证可停放整车的等待区。

副本将 G1 纳入预约与物理占用，取消其默认免预约；所有车型继续保留真实车身避障。
旧的固定车型批次顺序、60 秒收集等待和优先让 G1 整批执行的规则在联合排程模式下禁用。

## 配置和观察

主要参数：`src/hetero_fleet_scheduler/config/joint_schedule.yaml`。
可以通过 `schedule_config:=/绝对路径/配置.yaml` 选择另一组参数，
通过 `channels_file:=/绝对路径/通道.yaml` 同时传给排程与交通层。

```bash
ros2 topic echo /fleet_schedule --once
ros2 topic echo /channel_reservations --once
```

`/fleet_schedule` 包含 revision、epoch、status、rows、unscheduled、solve_sec、reason；
rows 中 start/end 是期望任务包络时间，groups 是估算资源集。
`/channel_reservations` 包含真实 owner/occupants/queue、SIPP/PBS 窗口，
以及是否收到有效排程建议。建议超过 12 秒墙钟失效，绝不因此解除物理占用。

## 验证

```bash
source setup_A.bash
/usr/bin/python3 -m pytest src/hetero_fleet_scheduler/test src/hetero_fleet_bringup/test -q
/usr/bin/python3 tools/verify_joint_schedule.py
ctest --test-dir build/hetero_fleet_navigation --output-on-failure
ctest --test-dir build/hetero_fleet_gz --output-on-failure
```

`tools/verify_joint_schedule.py` 检查原 demo_10 的离线可行排程、车型和资源互斥，
输出 `work/joint_schedule_check.json`。它不是 Gazebo 运行结果，不能与原批次的
1552.046 秒直接作性能比较。完整运行对比仍需相同任务、速度、地图下重复实验。

CP-SAT 建模参考：[OR-Tools Job Shop](https://developers.google.com/optimization/scheduling/job_shop)。

## 本次验证记录

- 全部 6 个 ROS 包已在副本内构建；调度与 bringup 在最终修改后重新构建。
- Python 单元/回归及真实 ROS 消息联调通过；精确数量见随附验证结果。
- 导航原生检查：首次 9/10 通过；发现退让朝向用例的旧起点与 G1 重叠。
  在原文件的只读快照中复现同样失败后，仅在副本修正验证用例：保留旧起点应拒绝的安全检查，
  新增有效起点的倒退朝向检查。该项复测通过，现有 10 项原生导航检查均有通过记录。
- Gazebo 原生检查 2/2 通过。
- 完整 headless 启动观测约 210 秒，四套 Nav2 日志均显示 ACTIVE，联合调度节点启动成功。
  主动停止验证时出现继承节点的 SIGINT/重复 shutdown 回溯及一个 collision_monitor 退出错误；
  启动观测阶段未见对应致命错误。本次没有向该物理仿真提交任务。
- 独立真实 ROS 联调验证了：CP 排程消息到达中央预约器，而缺少导航服务时任务仍保持 PENDING。
- 原 demo_10 的离线排程覆盖 10 项任务，未出现机器人任务重叠或共享资源区间重叠。
- 尚未完成副本A的 Gazebo 十任务批次；尚不能报告新的实测 Makespan 或改进百分比。

## 后续故障修复

- omni 控制器恢复时重新计算路径，见 `OMNI_STALL_FIX.md`。
- heavy 出场被停靠车辆阻断：增加首次让位、真实三段路径预检和空载临时阻塞延期，见 `HEAVY_EXIT_FIX.md`。原失败记录保留，恢复任务采用独立 ID、时间和指标目录；实际复跑结果见 `VALIDATION_A.json`。
- 恢复批次四项任务全部完成后的返航堵塞：修正执行中任务计数，增加返航串行准入与可达路线选择，见 `DOCKING_FIX.md`。
- 重新启动后完整十任务的让位、不可达反馈与等待位释放修复，见 `FULL10_FIX.md`；最终连续批次的实测指标以 `VALIDATION_A.json` 的 `full_ten_task_validation` 为准，旧批次和补跑指标分别保留。
