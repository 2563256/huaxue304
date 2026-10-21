# 副本 B：CP-SAT + 滚动规划 + CBS/EECBS

副本位置 `/home/ubuntu/huaxue304_B`，从副本 A 独立复制。原项目和副本 A 的文件校验分别保留在 `ORIGINAL_MANIFEST.json` 和 `COPY_A_BASELINE.json`。B 使用自己的构建产物、Python 依赖、ROS 域和 Gazebo 分区。

| 层级 | 副本 B 的实现 |
|---|---|
| 任务排程 | 保留 CP-SAT，考虑车型能力、载荷、电量、任务先后、空驶和共享通道，以减少 Makespan |
| 在线更新 | 保留 RHCR 思路的任务滚动框架，5 秒更新、2 秒提交窗口、1800 秒任务前瞻；Nav2 每 45 秒刷新尚未执行的路线，也响应目标变化、路径阻塞和控制器失败 |
| 路线与时间协调 | CBS 或 EECBS 约束树，处理完整车身扫掠占用的空间单元和连通瓶颈预约；选择原生候选路线及出发时间 |
| 空间搜索 | State Lattice 生成原路线及在穿越连接通道处施加空间排除后生成的绕行候选；全部保持原车型、运动原语和完整矩形车身扫掠检查 |
| 执行 | Nav2 规划插件返回 CBS/EECBS 选定的实际路线；MPPI 跟踪，通道速度门等待获准的出发时间；下游静态地图车身预测检查按可行方向投影、缩放或停止速度；轮式车型跟踪停滞时仅在整车旋转扫掠及 CBS/EECBS 均通过后调用原生 Nav2 Spin 转正；碰撞监测、实际占用及让位/返航协调继续生效 |

本项目实现为适配实际 Nav2 车队的 CBS/EECBS 变体：低层精确搜索有限原生路线目录及整数出发时间，高层加入可采纳的通道两车代价下界，并采用 CBS 最小代价排序，或 EECBS 的 CLEANUP/OPEN/FOCAL 和在线误差估计。默认时间粒度 0.5 秒、EECBS 的 `w=1.2`、约束树 512 节点/0.35 秒预算。预算耗尽返回无解并交由导航重试，绝不把冲突解作为可执行路线。

最优性与 1.2 倍界限只针对当前固定承诺、有限候选目录（每次最多原路线加一条绕行路线）、离散时间和预测速度下的**行驶时间总和**，不是连续空间所有运动轨迹的界限，也不是整个批次实际 Makespan 的界限。候选没有穷尽所有拓扑路线；只能在原生规划证实可停的起点等待，不在狭窄通道内虚构等待点。这里的滚动框架是工程化的 RHCR 思路，不等同于论文完整实现。

预约同时包含车身扫掠空间单元和瓶颈段时间窗；重复进入同一段保留分开的预约。缺乏整车等待空间的相邻通道仍合并为连续独占区。当前地图不承诺同向并行；真实车身占用、退出滞回和速度预测始终高于名义预约。

执行中的路线按实时位置重新预测；已经排队或尚未获准执行的机器人按停止状态处理，不能用假想的未来前进路线妨碍通道占用者退出。速度输出链使用填充矩形车身（包含内区和角点）的静态地图预测检查，保留车型尺寸、原有 padding 及与 State Lattice 相同的半采样步长余量，避免局部控制器偏转后将起点推入占用区。任务可达性预检使用独立 Probe 规划器，不写入执行预约。CP-SAT 接收交通窗口与延误反馈，偏差过大触发未提交部分重排。

## 启动

首次构建：

```bash
cd /home/ubuntu/huaxue304_B
bash bootstrap_B.sh
source setup_B.bash
```

默认 EECBS（带 GUI）：

```bash
ros2 launch hetero_fleet_bringup bringup.launch.py
```

CBS：

```bash
ros2 launch hetero_fleet_bringup bringup.launch.py \
  channels_file:=/home/ubuntu/huaxue304_B/src/hetero_fleet_scheduler/config/channels_cbs.yaml
```

等待四套 Nav2 激活后，在另一个终端提交同一十任务单：

```bash
cd /home/ubuntu/huaxue304_B
source setup_B.bash
ros2 run hetero_fleet_scheduler task_publisher \
  --scenario /home/ubuntu/huaxue304_B/src/hetero_fleet_scheduler/config/tasks/demo_10.yaml --interval 8
```

完整自动验证（新建批次名称，不复用旧结果）：

```bash
python3 tools/validate_B.py --mode eecbs --run eecbs_new --domain 44
python3 tools/validate_B.py --mode cbs --run cbs_new --domain 45
```

验证器等待十二个 Nav2 生命周期节点就绪与 G1 步行准备完成后提交十任务，要求原始任务 ID、同一调度进程连续执行、零失败，且收到当前算法实际运行的证据。成功后停止自己创建的仿真进程，不影响其他分区。审计证据写入 `work/validation_B/<批次>`，指标写入 `metrics/<批次>`。历史 A 文档是复制基线背景；B 的运行与验收以本文件和 B 验证记录为准。

方法参考：
- [CBS 原论文](https://webdocs.cs.ualberta.ca/~nathanst/papers/sharon2015cbsjournal.pdf)
- [EECBS 原论文](https://jiaoyangli.me/files/2021-AAAI-1.pdf)
- [EECBS 作者实现](https://github.com/Jiaoyang-Li/EECBS)；B 为结合原生车型搜索的适配实现，未声称直接移植该仓库的完整增强功能。

B 还调整了轮式车型的墙边软惩罚权重，用完整车身预测过滤约束输出速度。硬碰撞成本、车型尺寸、原有碰撞监测与安全停车仍然启用。验收对实际里程计位姿逐样本检查物理车身与静态地图，检测到碰撞即拒绝该批次；规划附加 padding 与物理外形在记录中区分。

静态速度过滤和原生 State Lattice 使用同一矩形车身、相同 padding，以及地图分辨率 `1/8` 的扫掠采样、半步的采样余量。四种车型的原生路线与实际东侧转角回放会逐点核对边界一致性，避免规划可行而过滤外形额外扩大导致无法执行。


位姿转发直接从 Gazebo Pose_V 保留模型名，按精确名称选取对应机器人，避免延迟生成模型时借用临近车辆位姿；上游按仿真时间限频 40 Hz。自动验收除十二个 Nav2 生命周期节点和 G1 行走准备状态外，还要求三个轮式驱动及其关节状态广播器均为 ACTIVE。每个批次使用独立 Gazebo 分区，启动进程退出（包含僵尸状态）时立即终止审计，不让旧批次继续订阅新批次数据。

位姿时间戳保留仿真采样时刻；静态车身保护同时检查实际速度的制动扫掠与请求速度的预测扫掠，保留 0.35 秒反应时间并允许低速通过有效的窄余量路径。
