# 副本 C：CBS 完整十任务验证

项目位于 `/home/ubuntu/huaxue304_C`，从已通过 EECBS 验收的副本 B 独立复制。仅在 C 内将默认交通算法设为 CBS；保留 CP-SAT、RHCR 思路的滚动框架、细化车身预约、原生 State Lattice 和 MPPI 执行保护。原项目、A、B 的源文件哈希记录在 `PROTECTED_BASELINES_C.json`。

C 使用自己的构建产物、Python 依赖、ROS 域（默认 68）和 Gazebo 分区，不引用 B 的安装目录或虚拟环境。

## 启动

已构建的本机副本：

```bash
cd /home/ubuntu/huaxue304_C
source setup_C.bash
ros2 launch hetero_fleet_bringup bringup.launch.py
```

重新构建使用 `bash bootstrap_C.sh`。默认配置已设为 CBS。

另开终端提交任务：

```bash
cd /home/ubuntu/huaxue304_C
source setup_C.bash
ros2 run hetero_fleet_scheduler task_publisher \
  --scenario /home/ubuntu/huaxue304_C/src/hetero_fleet_scheduler/config/tasks/demo_10.yaml --interval 8
```

独立完整验收（不能复用已有批次名称）：

```bash
python3 tools/validate_C.py --mode cbs --run cbs_new --domain 68
```

验证器等待四套 Nav2、三个轮式驱动与 G1 就绪，提交原始十任务单；要求同一启动进程连续运行、十个原 ID 全部完成、零失败，并核验 CBS 实际规划证据及物理车身静态地图检查。结果保留在 C 内，仿真在验收结束后由验证器清理。

这里的 CBS 为有限原生路线目录与离散出发时间的适配实现，不能将目录内最优性解释为全连续空间最优性或实际 Makespan 最优性。缺乏安全等待空间的连续瓶颈仍合并预约，实际占用高于预测时间窗。滚动框架为 RHCR 思路的工程实现。

指标中的距离和能耗沿用调度器模型估算；时间以 ROS 仿真时间计，Makespan 不含最后任务完成后的返航。单次成功只能验证当前场景的执行能力。

## 完整验收结果

CBS 原始十任务 10/10 完成，失败 0。Makespan 1501.114 仿真秒。373 项 Python 测试与 9 项原生检查通过。完整指标见 `outputs/副本C_CBS十任务/十任务完整验证与指标.md`，同批次审计证据见 `VALIDATION_C.json`。
