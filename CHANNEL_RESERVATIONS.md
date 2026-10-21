# 通道预约（2026-10-14）

四台机器人共用一个 `channel_reservations` 节点。它读取各车实际 Nav2 `/plan`、
真值 `/odom` 和平滑后的速度，默认通过 SIPP/PBS 优化排队顺序后授予通道独占令牌。没有令牌的小车在
入口前停止；整个带安全边距的车身驶离后才释放。任务完成、取消导航或超时，
都不能提前释放仍被车身占用的通道。

默认配置为 `src/hetero_fleet_scheduler/config/channels.yaml`，坐标属于当前 geomap
的 `map` 帧，包含北侧通道以及东、中、西连接段。这些段之间没有足够的独立等待
空间，因此共享 `upper_loop` 预约组：同一时间仅允许一台小车占用这个瓶颈网络。
G1 默认免预约，可与小车同时使用通道，依靠实际车身动态避障绕行。
南侧充电泊位行作为接近区域，继续由现有遇堵让行和碰撞监测协调，避免停在泊位的
机器人永久锁住通道。修改地图时需同时修改通道区域与等待空间。

`bounds` 的格式为 `[xmin, ymin, xmax, ymax]`。支持多个独立预约组；同组区域和
相距不足“请求距离 + 两倍最大车身半径（含边距）”的区域会自动合并，避免车辆
持有一个通道时还必须等待紧邻的下一个通道。远离且具备等待空间的通道可并行使用。
执行层仍为保守独占预约；时间窗用于选择进场顺序，尚未实现同向编队或同组多车并行。

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `reservation_exempt_robots` | `[g1_1]` | G1 不占用、排队或阻塞小车令牌；保留自身定位、动态避障与碰撞监测 |
| `exit_waypoints` | 当前地图的三个出口外目标 | 占用者先出场，再回桩；目标选择检查完整车身和释放边距 |
| `retreat_waypoints` | 当前地图的四个短退让点 | 排队车在接近区域让开，避免长途回桩穿过另一机器人 |
| `scheduling` | `sipp_pbs` | 时间窗/优先级搜索；`fifo` 恢复原有顺序 |
| `pbs_max_nodes` | 64 | 一轮 PBS 的优先级节点展开上限 |
| `pbs_budget_sec` | 0.025 s | SIPP/PBS 搜索墙钟预算，超限使用可行解或回退 FIFO |
| `reservation_headway_sec` | 0.5 s | 预测占用尾部的时间余量 |
| `max_queue_wait_sec` | 60 s | 老请求取得硬优先级，防止短任务持续插队 |
| `request_distance_m` | 3.5 m | 沿实际路径提前请求；距离包括车身旋转产生的角点运动 |
| `stop_distance_m` | 2.2 m | 预约被拒时，在车身接近入口前停止，给出场车辆留空间 |
| `body_margin_m` | 0.05 m | 加在各车实际 footprint 和 local padding 外的预约边距 |
| `release_margin_m` | 0.10 m | 出场迟滞；防止车尾未清空就释放、边界抖动重抢 |
| `preentry_lease_sec` | 20 s | 未进场的预约超时后回到队尾；已进场的令牌不按时间强制释放 |
| `pose_timeout_sec` | 0.5 s | 定位更新失联后停止放行，保留未知位置车辆的占用 |
| `command_timeout_sec` | 0.35 s | 中央节点停止转发失效的原始速度 |
| `reaction_time_sec` | 0.6 s | 速度预测中的反应时间余量 |
| `prediction_decel_mps2` | 0.4 m/s² | 保守的线速度制动预测 |
| `prediction_angular_decel_rps2` | 0.6 rad/s² | 保守的角速度制动预测 |

所有状态使用完整矩形车身，包含麦轮横移、差速倒车和原地旋转。控制器或 BackUp
产生与规划路径不同的速度时，仍检查其预测扫掠区域，不能借恢复动作越过预约门槛。
车身、速度限制或制动参数变化后需重新检查边距及等待空间。

Nav2 的队友虚拟扫描层使用 `hetero_fleet_navigation::SnapshotObstacleLayer`：
每次代价地图更新重建本层的当前观测，并刷新主地图中旧障碍和旧膨胀区域。
`observation_persistence: 0` 本身不会保证旧栅格消失，远距离稀疏清除射线可能留下
残影。此层只用于当前整队观测；真实激光层和静态地图保留原行为。没有新观测时继续
使用最后缓存观测，避免仅因一帧消息迟到就移除仍在原地的车身。

速度链为：

```text
Nav2 → cmd_vel_nav → velocity_smoother → cmd_vel_smoothed
     → 中央预约过滤 → cmd_vel_channel → 本地看门狗 → cmd_vel_reserved
     → collision_monitor → cmd_vel → 车轮 / G1 控制器
```

本地看门狗使用墙钟，中央节点断开后 0.35 s 内将旧指令置零；暂停仿真时也会执行。
预约租期和调度等待预算使用 ROS 时间，暂停仿真不会消耗预算。时钟回退时清除未来
预约及旧路径，保留物理占用，等待新里程计和新规划恢复。

`fleet_manager` 排队时先确认取消当前 Nav2 目标，再等待放行并重发原目标。
占用者完成任务且有其他小车排队时，暂停接新任务，先导航到通道外的 `exit_waypoints`，
再尝试回桩，避免被 G1 挡住的充电位连带锁住整个通道。排队车从 `retreat_waypoints`
选择短退让点：完整车身处于通道及释放边距之外、离本次排队入口更远、直接退让区域无静态墙体
或队友车身。G1 始终作为真实障碍保留。出口选择还检查排队车预期退让位置，避免出口被退让目标反向堵住。最终路线仍由 Nav2 规划与执行。
回程优先车辆尚未进场时也可触发上述退让；退让车等待其令牌释放，
等待已进入的占用者完整车身驶离，
再重新规划原任务；避免两端队列堵住唯一出场路线。退让导航仍受中央速度过滤、
真实车身避障和碰撞监测约束。退让无法完成会保留物理占用并报告超时，不强行释放。
等待不消耗卡住恢复次数，不会因排队触发盲目倒车。`channel_wait_timeout_sec` 默认
为 300 s（ROS 时间）；取消或等待失败会撤销未来请求，仍在通道内的车保持占用。
嵌套让行导航使用计数保留外层意图；每 0.5 s 的墙钟心跳防止退出的调度器遗留请求。
直接发送的 Nav2 目标则从 action status 识别，失去 action server 后撤销未来请求。

重启完整仿真或 Nav2 后自动启用。参数示例：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch hetero_fleet_bringup bringup.launch.py headless:=true rviz:=false
# 诊断时禁用预约；仍保留本地指令看门狗与 collision_monitor。
ros2 launch hetero_fleet_bringup bringup.launch.py reserve_channels:=false
# 自定义当前地图的通道区域。
ros2 launch hetero_fleet_bringup bringup.launch.py channels_file:=/absolute/path/channels.yaml
```

`/channel_reservations` 是 JSON 状态话题，包含 owner、物理 occupants、调度 queue、
等待时长和未进场租期截止时间。`/<robot>/channel_wait` 表示预约等待；
`/<robot>/reservation_active` 是调度导航意图。RViz 的 **Channel Reservations**
显示 `/channel_markers`，给出区域、占用者和队列；灰色表示未获得定位或已禁用预约。

```bash
ros2 topic echo /channel_reservations --once
python3 -m pytest -q src/hetero_fleet_scheduler/test src/hetero_fleet_bringup/test
ctest --test-dir build/hetero_fleet_navigation --output-on-failure
python3 tools/verify_reservations.py
```

原生退出重放另覆盖 v5 现场的 10 项检查：G1 占住充电位时，直接回桩和长途退让无路；
短退让、先出场再回桩可规划，并检查整车扫掠。此重放不等同于 Gazebo 的 10 任务通过。

## SIPP/PBS 的范围

State Lattice 负责空间路径，`sipp_pbs.py` 在这些固定路径压缩出的通道资源图上
搜索时间和优先级，不搜索空间绕行。低层 SIPP 的状态为“路线节点、最大安全时间窗”，
使用连续事件时间跳转，检查整个转移的资源占用；连续瓶颈被压缩为不能中途等待的边。
高层 PBS 遇到冲突时分别尝试两种优先关系，维护无环的部分优先级图，并按拓扑顺序
对所有祖先预约重新规划，比较加权完成时间。它有节点数和墙钟预算，因此不保证全局
最优或完备。搜索无可行解时回退 FIFO；已存在的物理占用者不会被优先级搜索抢占。

ROS 桥读取 `robot_profiles.yaml` 中各车的 `max_speed` 和 `max_omega` 估算占用时间。
已进场车辆的剩余路径加入固定预约；无法预测离开的静止/取消车辆作为无限占用。
目标位于通道内时，额外计入停留不确定性。`scheduler.windows` 是排队决策的预测信息，
不是执行许可；进度变慢、原地停留或预测窗口结束，都必须继续等待实际车尾驶离。
排队成员、令牌、路径或两秒滚动周期变化时重算；重规划、取消和 ROS 时钟回退会使
缓存失效。路径扫掠接触表按计划缓存，运行时查询剩余接触距离，仍对每条实际速度指令
单独进行完整扫掠检查，支持倒车和横移恢复。

`/channel_reservations` 的 `scheduler` 包含预测窗口、部分优先关系、搜索次数及耗时、
是否回退 FIFO；RViz 标签显示当前策略与实际队列。修改配置后重新构建并重启 Nav2。

算法参考：[SIPP 原论文](https://www.cs.cmu.edu/~maxim/files/sipp_icra11.pdf)、
[PBS 原论文](https://ojs.aaai.org/index.php/AAAI/article/view/4758)。本实现是通道级时间调度，
没有替换 Nav2 空间搜索器或 MPPI 控制器。

验证脚本在同一批 12 条原生 State Lattice 路线上分别回放 FIFO 和 SIPP/PBS，使用相同
车身、每车速度上限与 0.1 s 时钟，比较完成时间和预约 CPU 时间，并检查先预约后进场、
独占占用与最终释放。真实 ROS 话题测试覆盖两种策略的定位失联、中央节点断开、
直接 Nav2 action 和暂停 ROS 时钟。回放采用理想跟踪，不模拟通道外车辆碰撞、MPPI
或 Gazebo；整队诊断尚未达到 10/10。结果见 `build/channel_replay_report.json`。

本次验证：220 项 Python 测试、导航 10 项与 Gazebo 2 项 CTest 全部通过；两种策略都完成三个轮式任务的
12 条原生路径回放，满足独占进场与物理释放。比较采用相同路径、车身和速度上限。

| 指标 | FIFO | SIPP/PBS |
| --- | ---: | ---: |
| 平均任务完成时间 | 117.33 s | 107.20 s |
| 整批完成时间 | 142.9 s | 143.5 s |
| 三车累计等待 | 142.8 s | 112.3 s |
| 预约累计 CPU 时间 | 2.524 s | 0.194 s |
| 单次预约计算墙钟 P95 | 2.881 ms | 0.155 ms |

平均完成时间降低 8.6%，累计等待降低 21.4%，
预约 CPU 时间降低 92.3%。整批完成时间增加 0.6 s；结果体现当前示例上的
平均等待与计算收益，不代表所有任务都能提高吞吐量。

## 任务点作业与 G1 例外（2026-10-15）

任务取货点与卸货点各保持停车 3 秒仿真时间，使用无终点朝向约束的任务规划、位置
到达检查和关闭目标角度评分的任务控制器。停留前通过 `/<robot>/task_hold` 将中央
指令置零；停留中不消费暂停的仿真时间，解除时不重放旧指令。状态的 `task_service`
表示作业停车，`channel_wait` 为 false。作业停止或取消仍不能提前释放小车物理占用。

G1 也使用 State Lattice，以实际车身检查转弯和路径起始朝向，控制器按规划朝向倒退，
避免二维路径切换造成就地转身扫墙。任务末端仍不约束朝向。

G1 已完成任务但下一取货点暂不可派工时，先清空卸货点，驶向配置的 `idle_clearance`
待命点；它不参与小车的退让握手。

G1 不进入任何令牌的 occupants、queue 或 PBS 请求。小车预约就绪只依赖三台小车
定位，G1 指令则单独要求自身定位、活动目标和有效路径。G1 也不再参与原有遇堵拉开
后的等待握手；其任务与普通导航行为树将持续阻塞去抖改为 0.5 秒、重规划最小间隔
改为 1 秒，规划检查频率为 1 Hz。动态队友障碍层及 collision_monitor 均保持启用。
免预约不保证窄处一定能并排通过，实际绕行依赖车身净空。

`python3 tools/verify_task_service.py` 验证任务点不强制终点朝向，以及 G1 规划绕过完整
小车矩形障碍。`verify_reservations.py` 的固定路径预约对比不包含 3 秒作业停留，也不
代表四车动态轨迹跟踪。Gazebo 整队实测已执行，尚未达到 10/10。
G1 任务规划容差为 0、任务控制器到达容差为 0.07 m，避免规划终点偏差与停车容差
叠加后超出调度器的 0.15 m 实际任务半径。

所有导航行为树对临时目标占用错误 206 先等待 5 秒再重试（最多 6 轮），不立即消耗
调度器的恢复次数。行为树中固定距离的 BackUp 已移除，避免暂时无路时盲目倒入
地图边缘；调度器仍提供经静态几何检查、预约速度过滤和碰撞监测的恢复倒退。
