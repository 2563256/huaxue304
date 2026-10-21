# hetero_fleet_geomap_ws — 异构机器人车队 Gazebo + Nav2 + 贪心调度仿真（实景扫描地图版）

四台机器人、四种构型（`light_1` / `heavy_1` / `omni_1` / `g1_1`）在一张**由点云扫描重建的
室内地图**上协同完成「取货 → 送货」任务：Gazebo 负责物理与传感器，每台车跑自己的一套
Nav2，`fleet_manager` 用可解释的成本函数决定「谁去做哪个任务」。

这是 `hetero_fleet_ws`（仓库地图版）的**换地图分支**：世界换成 26.0 × 10.4 m 的实景回型
走廊（扫描模型 `geomap_final_noroof`），车型按通道净空整体缩小；调度层比仓库版多了两件事
——**对峙/卡死处理**（standoff watchdog）、**空闲回充电位**及**通道预约**。

> 换地图、缩车型、物理引擎从 Bullet 换到 DART 的全过程记在 [MAP_SWAP.md](MAP_SWAP.md)，
> 那份文档是**当时的历史记录**，其中若干数字已被后续调整覆盖——**当前状态以本文件与代码
> 为准**。日常命令速查见 [RUNBOOK.md](RUNBOOK.md)。

```
  /task_request ─► fleet_manager ─► /<robot>/navigate_to_pose (Action)
                       │                     │
                       │                     └─ 每车一套 Nav2（独立命名空间）：
                       │                        map_server + AMCL + State Lattice（四种车型）
                       │                        + MPPIController + velocity_smoother
                       │                        + 通道预约过滤 + collision_monitor（安全兜底）
                       │                             │ /<robot>/cmd_vel
                       │                             └─ cmd_vel_stamper ─► diff_drive_controller
                       │                                                  / mecanum_drive_controller
                       ▼
        /fleet_status · /task_markers · metrics/metrics.json + metrics/tasks.csv

  物理层  Gazebo Sim 8.11（DART，1 ms 步长）+ worlds/geomap_fleet.sdf + 4 个 URDF 实例
  地图层  assets/scan_sources/ 里的原始扫描 → 处理成 models/geomap_final_noroof
          （1 个视觉网格 + 501 个盒体碰撞），栅格化成 maps/geomap.pgm
  桥接层  ros_gz_bridge(/clock, 4×scan, /gz/dynamic_pose) + 4× 真值里程计 relay
```

> 验证状态：本文档的命令与数字都在本机（ROS 2 Jazzy / Gazebo 8.11.0 / 2026-09-24）实跑过，
> 未验证的项目在 §13 单独列出。

---

## 任务点到达与 G1 并行通道（2026-10-15）

取货、卸货各使用位置到达判定（0.15 m），不要求任务给定朝向；到达后保持当前实际
朝向并停车 3 秒（ROS 仿真时间），再继续运输或标记完成。暂停仿真不计入停留时间。
任务导航使用 `TaskGridBased`、`TaskFollowPath`、`task_goal_checker` 与独立任务行为树，
关闭任务控制器的目标角度评分；充电、离桩及恢复导航保留原有朝向约束。
`/<robot>/task_hold` 在作业期间将中央输出速度置零，解除时清除旧速度，避免旧指令重放。

`channels.yaml` 默认 `reservation_exempt_robots: [g1_1]`：G1 不持有或排队等待小车令牌，
也不阻塞小车的通道预约；原有 G1/小车的拉到旁边后等待握手已移除。G1 继续使用实际
车身、动态队友障碍和碰撞监测，在路径被挡住后更快重规划绕行；并行需要足够真实
净空，空间不足时仍可能由避障停车。三台小车之间继续使用 SIPP/PBS 独占预约。

重启完整 bringup 生效。2026-10-16 连续 Gazebo 整队实测完成 10/10，20 次停靠均使用原任务标记；
运行中在线修正全向车速度死区，已写入默认配置。具体条件及保守派工策略见下方“十任务实测”。

## 实测后的修复（2026-10-15）

`StableReplan` 将缓存路径与导航目标及请求时间戳绑定，退让后重发同一目标也重新规划；新目标或空路径必须重新计算，失败时清空
路径，异步有效性检查等待实际响应。差速车改为向前离桩，麦轮保留横向离桩。
G1 现也使用包含朝向和完整车身扫掠的 State Lattice，基元匹配前进、倒退和角速度接口；
路径对齐采用规划朝向，反向路线无需先转身。任务规划不约束末端朝向，终点连接到原任务
坐标；控制器位置容差为 0.07 m，为任务的 0.15 m 实际到达
范围留出误差余量。调度器仍检查新鲜实际位置，不因 Nav2 成功就虚报任务到达。

通道占用者完成任务后，如果有小车排队，先驶到 `exit_waypoints` 的通道外目标，再回桩。
排队车先经 Nav2 驶向 `retreat_waypoints` 中较远离入口的短退让点，避免回桩路线被 G1
堵住；调度器检查静态完整车身、队友车身和退出边距。实际 Nav2 规划与速度过滤仍生效，
占用者的完整车身驶离后才申请原任务路径。G1 继续免预约。
入口退让也覆盖尚未进场的回程优先车辆，退让车等待其令牌释放。
G1 完成任务后若下一取货点尚不可派工，先去轮式路线之外的待命点 `(1.2,-1.0,0)`，
不再停在卸货点堵住出入口；可派工时立即接单，仍免预约。
中央预约计算与转发统一为 20 Hz 墙钟定时器，避免每个输入重复计算整队并放大发布率。

队友虚拟扫描使用 `SnapshotObstacleLayer`，每次更新从当前缓存观测重建本层，并更新
主地图的旧障碍与膨胀范围；避免稀疏清除射线漏掉机器人经过后的孤立障碍格。
静态地图、真实激光层与碰撞监测保留。未收到新观测时，仍保留最后观测到的队友。
目标被临时占用（Nav2 206）时，行为树先等待 5 秒再重试，最多 6 轮；取消固定距离
的行为树倒退。调度器保留经过静态几何检查、并受预约和碰撞监测约束的恢复倒退。
路径对齐前视偏移从 20 改为 4 个点，使倒车预测轨迹也能启用路径对齐；只在距终点
0.15 m 内停止该评分。任务终点角度评分仍关闭，不要求额外转向。

G1 的实际低速跟踪另有位置反馈：行走时也累计期望平移并根据实际位置纠偏，限制累计
误差与策略速度；仍通过 LSTM 输出和真实关节力矩运动。此前只有接近零平移时才保持
位置，小速度转弯仍会侧移到墙边。G1 普通导航段结束后停车稳定 3 秒，再从实际姿态
规划下一段；任务点仍只使用原来的 3 秒作业停留。此执行层修正在 v7 失败现场之后加入，
当前整队结果需以新的 Gazebo 复测为准。

## 通道预约（2026-10-14）

默认启用三台小车共享的 SIPP/PBS 排队优化与物理独占预约，读取实际 State Lattice / Nav2 路径和里程计。
北侧及东、中、西连接通道组成 `upper_loop` 预约组；缺少独立等待空间的相邻段合并，
完整车身驶离后才释放。南侧泊位行继续使用现有让行与碰撞监测。
排队车辆被速度过滤器拦停，调度器取消当前导航并在获准后重新规划续行；
预约等待不消耗卡住恢复次数。恢复倒车、横移和旋转同样受预约限制。
定位或预约节点失联时停止放行，已占用通道不会凭超时强制转交。

配置为 `src/hetero_fleet_scheduler/config/channels.yaml`；状态话题为
`/channel_reservations`，RViz 已增加区域、占用者与队列显示。
265 项 Python 测试、导航 10 项与 Gazebo 2 项 CTest 通过，包含真实 ROS 速度链故障检查与 12 条原生
规划路线的并发预约回放；最近受影响的检查已复跑。整队实测条件见下方“十任务实测”。
在固定 State Lattice 路径上计算时间窗，搜索部分优先关系并缓存扫掠结果；配置
`scheduling: fifo` 可恢复原策略。理想回放平均完成时间缩短约 8.6%，预约 CPU
耗时降低约 92%，整批完成时间增加 0.6 s。
参数、开关、覆盖范围及复现命令见 [CHANNEL_RESERVATIONS.md](CHANNEL_RESERVATIONS.md)。

## 离桩与转弯可行性修正（2026-10-14）

三台轮式车使用本工程的 `hetero_fleet_navigation::SweptLattice` 全局规划插件：
在 `(x,y,朝向)` 状态格上运行带启发式权重的 A*，通过预生成运动基元扩展。
控制集分辨率为 0.05 m、64 个非均匀朝向、曲线最小转弯半径 0.50 m。
差速车支持前进、倒车、曲线与原地旋转；麦轮还包含保持车头朝向的横移和后退。
G1 保留原规划器。MPPI 使用路径朝向，差速车禁止横移，麦轮仍使用 Omni 模型。

全局、局部代价地图和规划器的扫掠检查统一使用每台车的真实矩形轮廓；
light/heavy 的 padding 为 0.03 m，omni 为 0.02 m。膨胀半径仍为 1.20 m。
搜索对每个基元检查整个车身扫过的占用格，包含原地旋转；每次规划读取当前代价地图。
角点移动采样间距不超过 1/8 栅格，并增加半采样间距的几何余量，覆盖样本之间的运动。
同时检查 Nav2 所用的 Bresenham 数字轮廓，确保发布的路径能通过运行时 `IsPathValid` 检查。
不使用解析捷径或路径平滑。起终点与离散格之间保留不超过 1 栅格的连接段，
连接段也检查完整车身；实际控制精度仍由 MPPI 与目标容差决定。

启发式权重为 1.5、倒车代价倍数为 2.0、原地旋转代价倍数为 2.0，
每个状态只扩展一次；优先在规划时限内找到可行路线，不保证全局最优成本。
单次规划上限为 8 s / 100 万次扩展。插件与控制集在启动前检查车型、地图分辨率和轮廓一致性。
运动基元来源、预处理和参数说明见 `src/hetero_fleet_bringup/config/lattice/README.md`。

当前轮式出生/充电位均朝东，分别为 light `(-0.30,-0.25)`、heavy `(2.70,-0.45)`、
omni `(4.60,-0.45)`；差速车离桩点分别为 `(-0.10,-0.25)`、`(2.90,-0.45)`，
麦轮离桩点为 `(4.60,-0.25)`，朝向均为 0。
出生点、里程计锚点和充电标记同步。G1 的出生/充电位保留 `(1.20,-0.80)`。
`pickup_E` 调整为 `(5.80,-0.20)`，`dropoff_E` 调整为 `(0.00,4.22)`，任务文件、
世界标记与生成脚本同步。调度器发送目标前检查含朝向的完整车身，拒绝压墙的目标，
避免 Nav2 返回近似路径后车辆仍无法完成末端旋转。

静态验证：两个任务场景合计 49 条去重轮式路线与 3 个旧离桩目标回归检查通过；
另有 18 项原生规划器回归覆盖差速倒车/侧移限制、麦轮横移/后退、角度跨界、
旋转中途障碍、车身内部障碍、取消，以及障碍加入/移除后重规划。
逐段插值检查带局部 padding 的矩形及占用格内部，角点移动量不超过 1/4 栅格。
此验证不包含队友占道和 Gazebo 的轨迹跟踪，整队通行仍需交通预约和运行验证。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 tools/verify_scene.py
python3 tools/verify_lattice.py
# 保留全朝向外接圆的保守诊断；它不代替有朝向的矩形路线验证。
python3 tools/verify_scene.py --all-orientations
```

## 2026-10-16 十任务实测

Gazebo 连续复测完成原配置 10/10，永久失败 0、逾期 0；轻车 3、全向车 3、重车 2、G1 2。
20 次取卸货全部使用原任务标记，不强制终点朝向；停车为 3.000–3.027 秒 ROS 时间，
实际位置距原标记最大约 0.148 m。停靠位静态筛选改为真实矩形车身和规划余量，
修复外接圆过度保守、导致重车任务 01 偏移到旁侧点的问题；20 个原点和 4 条重车路线预检通过。
当前分批派工优先消除死锁，降低多车并发度；G1 仍免通道令牌。
265 项 Python 检查通过，受影响的原生路径、任务点与障碍更新检查已复跑。

本轮中途在线将全向车 `velocity_smoother.deadband_velocity` 从 `[0.008,0.008,0.02]`
改为 `[0,0,0]`，保留 MPPI 转角处微小纠偏；随后实际完成停车。配置已持久化，
速度限制和碰撞监测继续生效。只重载监测程序并将墙钟观察上限改为 4200 秒，
机器人和任务状态连续、十个任务只下发一次，原任务期限仍为 1800 秒 ROS 时间。
尚未另做一次最终默认参数从启动到结束的十任务复跑，也未据此验证 G1 与小车并行运输。

## 轮式机器人尺寸与通道策略（2026-10-15）

三种轮式车型在原放大模型上统一缩小 10%，真实碰撞轮廓：light 1.4318181×1.125 m、heavy
1.4927886×1.125 m、omni 1.2857139×1.125 m；G1 保持原尺寸。视觉、碰撞、轮径、
轮距、惯量、Nav2 局部/全局轮廓和碰撞停止区域已同步。轮式车型膨胀半径已进一步调整为 1.20 m；速度和任务载荷保持原配置。
当前泊位与目标点的进一步修正见上方“离桩与转弯可行性修正”。

抽查地图直通道的横向净宽为 1.65–2.35 m，两台轮式机器人的车身并排至少需要
2.25 m，实际还要加避障余量，因此窄段继续使用单车通道预约。G1 免通道令牌，按实际车身避障绕行。
队列退让不使用其他车的充电位；已经退到安全位置的车辆保持等待，优先短程倒退；按退让方向选择可行朝向，到点不再原地对齐。
排队车在通道占有者执行正常取卸货时也退让，退让后继续记录等待关系。
轮式车已经进入通道后，在同一搬运任务的取货、停留和卸货之间保留通道预约，
避免取货点在通道外时过早交接、导致入口双方互堵；任务结束后仍以完整车身驶离作为释放条件。
清场出口先调用实际 Nav2 规划器验证路线，跳过只有终点空闲、实际道路不通的候选。
执行期间若出口目标被其他机器人持续占用 2 秒 ROS 时间，则取消这一段，重新验证并切换出口。
G1 完成自身任务后，如果整队仍有工作，则离开作业点前往其待命泊位 `(1.2,-1.0,0)`。
当前充电前场采用可配置的轮式分批放行：`light_1 → omni_1 → heavy_1`，收集任务 60 秒 ROS 时间后开始。
同一轮式平台连续执行待处理任务；实际停入停车区后才交接给下一平台。轻车停在 `(-7,-0.25,0)`，
全向车停在 `(-4.8,-0.25,0)`，为中心入口和重车出场留路。G1 不参与该放行顺序。
派单也检查空闲队友的实际车身，暂时被占用的取货点保持待分配。
`wheel_yield_to_g1: true` 开启保守交接：G1 派工前用真实 Nav2 任务规划器检查取货路线，
结果只对新鲜且未明显变化的实体场景有效；检查异步进行，不阻塞调度回调。
G1 工作和离开作业点期间，小车不启动下一任务；已开始的小车任务先完成搬运、停车，再检查 G1。
因此该模式降低了并发度。G1 不申请小车令牌，派工和避碰仍依据实际路径。
`startup_clearance: true` 只为轻车启用启动清场；其他车型首次任务前保持原充电位。
分批策略优先消除前场死锁，牺牲了轮式车辆并发度；整队是否通过以实际任务日志为准。
规划地图使用全部已知队友的填充矩形点云（PointCloud2），覆盖车身内部和背面，
不会因虚拟激光的最近表面遮挡而漏掉后方车辆；原激光保留用于接触与碰撞监测。
点云快照逐帧重建，清除移动后的旧位置，覆盖整张地图内的队友。
G1 派工和清场出口还逐段复核实际规划路径与新鲜队友完整矩形，拒绝穿车路径。
轮式车辆整批完成后保持避让停车，不自动重新挤入原充电队列。
四张滚动局部地图均包含静态墙体层，使 MPPI 在缺少激光返回时仍能避让已知墙体。
G1 的规划和局部轮廓安全余量为 0.15 m，速度上限为 0.3 m/s、角速度为 0.45 rad/s，
用于给步态跟踪误差留出空间；G1 车身尺寸不变。
G1 使用全向 State Lattice 与 MPPI，侧向速度上限为 0.12 m/s；原生 LSTM 步态和真实关节力矩执行侧移，
不改变实体位置。离桩保持车身朝向，避免为横向移出泊位而反复转向。
轻车驱动轮扭矩随模型按 `s^4` 缩放（当前约 67.53 Nm），与已按 `s^3`/`s^5` 缩放的质量和惯量匹配；
原先固定 8 Nm 在实际 Gazebo 测试中无法执行转向，修正后已验证正反方向的真实转动。
临时实体阻塞使用最多 180 秒 ROS 时间的等待，要求自身静态可行且存在新鲜队友位姿；
不会因等待而完成任务，超时仍进入正常恢复和失败流程。
`tools/verify_scene.py` 的全朝向外接圆检查报告多处净空不足及不连通；该检查
比定向矩形通行更保守；当前使用原生 State Lattice 与完整矩形连续扫掠验证定向可达性。
可运行 `python3 tools/verify_robot_geometry.py` 核对碰撞模型、规划轮廓与车轮运动学。
启动方式保持不变，运行中的仿真需要停止并重新启动才加载新模型。

## G1 双足机器人更新（2026-10-10）

新增 `g1_1`，保持真实双足关节力矩控制，模型缩放 0.90，站立外观高度约 1.18 m。
上身保持行走策略的训练姿态，12 个腿部关节执行宇树官方 LSTM 策略；没有轮式底座。
导航最大速度 0.40 m/s，转速 0.65 rad/s，充电位 `(1.20,-0.80)`。
任务按四类机器人预计忙碌时间配平；载荷和装卸沿用车队调度仿真，不包含手臂抓取动作。
详细参数、复现步骤及验证结果见 [G1_INTEGRATION.md](G1_INTEGRATION.md)。

## G1 接入前的控制更新（2026-10-10）

车队调整为 light_1 / heavy_1 / omni_1 各一台，omni 尺寸缩小 16.7%，移除队友靠近减速。
所有车型按实际矩形轮廓在全局和局部代价地图中感知队友，停止指令不再被爬行逻辑覆盖。
任务场景按预计工作时间平衡，重排顺序不应 shuffle。详见 [FLEET_CONTROL_UPDATE.md](FLEET_CONTROL_UPDATE.md)。

下文旧四车运行测量属于历史数据，当前车队/控制参数以该更新与代码为准。

## 当前场景更新（2026-10-10）

四处凸出障碍物已按实际轮廓清除，所有障碍边缘采用 15 cm 开运算削除毛刺，
不增加任何占用格。导航栅格、扫描视觉和 Gazebo 盒体碰撞同步更新；新增自由面积
4.11 m²。所有机器人长、宽、高放大到此前的两倍，轮径/轮距/导航足迹同步调整。
可行驶地面统一平整至 world z=0.117 m，采用原蓝灰色材质，环境光提高至 0.80。
详见 [SCENE_EDITS.md](SCENE_EDITS.md)。原始扫描源保留。

## 1. 环境要求

| 组件 | 版本 | 说明 |
| --- | --- | --- |
| ROS 2 | Jazzy | `/opt/ros/jazzy` |
| Gazebo Sim | 8.11.0（Harmonic） | `gz sim`，物理用 **DART**（见 §12） |
| Nav2 | 全套 | 实际用到 `nav2_mppi_controller`、`nav2_smac_planner`、`nav2_collision_monitor`、`nav2_amcl` |
| ros_gz | `ros_gz_sim` / `ros_gz_bridge` | |
| ros2_control | `gz_ros2_control`、`diff_drive_controller`、`mecanum_drive_controller` | |
| Python | PyYAML、**numpy**、Pillow | `tools/` 下的地图处理脚本要 numpy + Pillow；运行时节点只要 PyYAML（Pillow 可选） |

### ⚠️ 本机特有：conda 环境会破坏编译

本机 conda 环境 `condaecho`（Python 3.14）会遮蔽 `/usr/bin/python3`，而 ROS 2 Jazzy 的
`rosidl` 需要系统 Python 3.12（`empy` 装在那里）。CMake 的 `FindPython3` 会优先选
`$CONDA_PREFIX`，生成接口时报 `ModuleNotFoundError: No module named 'em'`。

**工作空间根目录的 `colcon_defaults.yaml` 已经修好了这个问题**，不需要手动加参数：

```yaml
{ "build": { "cmake-args": ["-DPython3_EXECUTABLE=/usr/bin/python3"] } }
```

> 该文件只对从**工作空间根目录**执行的 `colcon build` 生效。

---

## 2. 编译

```bash
cd /home/xuzhijie/hetero_fleet_geomap_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

实测输出：`Summary: 5 packages finished`（全量约 1.5 s，只有改动时才真正重编）。

> 每个新终端都要 `source install/setup.bash`，否则 `ros2 run hetero_fleet_scheduler ...`
> 会报找不到包。

单元测试（39 个：成本函数 + 对峙/充电行为；需要先 source，因为要导入 `hetero_fleet_msgs`）：

```bash
cd /home/xuzhijie/hetero_fleet_geomap_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
cd src/hetero_fleet_scheduler
/usr/bin/python3 -m pytest test/ -v      # 实测 39 passed
```

---

## 3. 启动

### 3.1 一键启动全部（推荐）

```bash
cd /home/xuzhijie/hetero_fleet_geomap_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash

ros2 launch hetero_fleet_bringup bringup.launch.py
```

启动时序（timer 值来自 `bringup.launch.py` / `nav2.launch.py`）：

| 时刻 | 动作 |
| --- | --- |
| 0 s | Gazebo + `geomap_fleet.sdf` + 4× `robot_state_publisher` + 4× `ros_gz_sim create` |
| 8 s | `bridge.launch.py`：ros_gz bridge（`/clock`、4× `<ns>/scan`、`/gz/dynamic_pose`）+ `scan_frame_relay` + 4× 真值里程计 relay + 4× 静态 `map→<ns>/odom` |
| 13 s | 8 个 controller spawner：每车先 `joint_state_broadcaster`、1 s 后驱动控制器，车与车再错开 2 s |
| 30 s | `nav2.launch.py`：4 套 Nav2，套与套之间再错开 30 s（实测第 1/2/3/4 套的 map_server 在 +30 / +60 / +90 / +120 s 拉起） |
| 150 s | `fleet_manager`（实测 +152 s） |
| 160 s | RViz2（`rviz:=false` 时不启动） |
| 180 s | 根命名空间的 `map_server`（实测 +180 s）：只为 RViz 与 `ros2 topic echo /map` 提供 `/map`，导航用的是每个命名空间自己那份 |
| 205 s | `map_server_watchdog`（一次性；manager 正常工作时 no-op） |
| 215 s | `nav2_stack_watchdog`（一次性） |

整套激活完约 **2.5–3 分钟**，之后才能稳定接任务。启动完自检：

```bash
ros2 action list | grep navigate_to_pose        # 应恰好 4 条
ros2 lifecycle get /omni_1/bt_navigator         # active [3]
ros2 control list_controllers -c /omni_1/controller_manager
#   joint_state_broadcaster   ...  active
#   mecanum_drive_controller  ...  active
```

常用参数：

```bash
# 无显示器 / 省资源：关掉 Gazebo GUI 和 RViz（长跑建议，扫描图渲染很吃 GPU）
ros2 launch hetero_fleet_bringup bringup.launch.py headless:=true rviz:=false

# 只起仿真，不起调度器（手动喂任务）
ros2 launch hetero_fleet_bringup bringup.launch.py scheduler:=false
```

`bringup.launch.py` 自身的参数是 `headless` / `rviz` / `use_sim_time` / `scheduler`；
还透传 include 进来的 `world`、`map`、`controllers_dir`、`params_dir`、`autostart`、
`start_controllers`。全部可用 `ros2 launch ... --show-args` 查看。

### 3.2 长时间运行的资源提示

这张扫描图 + 501 个盒体碰撞 + 4 套 Nav2 会让整机负载明显高于仓库地图版。
实测建议：`headless:=true rviz:=false`；不要在同一个 `GZ_PARTITION` 上同时开第二个
`gz sim`（见 §14「两个 gz sim 实例」条目）。

---

## 4. 注入任务

唯一入口是话题 `/task_request`（`hetero_fleet_msgs/msg/Task`）。`task_publisher` 会等订阅者
出现后连发 4 次以对抗丢包；`fleet_manager` 按 `id` 去重。

**方式 A：命令行发单个任务**

```bash
ros2 run hetero_fleet_scheduler task_publisher --single \
  --capability heavy_payload --payload 120 \
  --pickup 10.60 -0.43 --dropoff 5.40 4.22 --priority 4 --deadline 420
```

**方式 B：回放预设场景**（10 个可完成任务，三车型预计工作量均衡）

```bash
ros2 run hetero_fleet_scheduler task_publisher \
  --scenario src/hetero_fleet_scheduler/config/tasks/demo_10.yaml --interval 8
```

**方式 C：20 任务统计场景**

```bash
ros2 run hetero_fleet_scheduler task_publisher \
  --scenario src/hetero_fleet_scheduler/config/tasks/mixed_20.yaml --interval 6
```

**方式 D：直接发消息**（`id` 必须唯一）

```bash
ros2 topic pub --once /task_request hetero_fleet_msgs/msg/Task \
"{id: 'adhoc_1', type: 'transport', pickup: {x: 1.70, y: 4.22}, dropoff: {x: 8.10, y: -0.43}, payload_kg: 12.0, required_capability: 'light_payload', priority: 2, deadline_sec: 600.0}"
```

字段约定：

* `required_capability`：`light_payload` | `heavy_payload` | `narrow_aisle` |
  `precise_docking` | `omnidirectional` | `""`（空 = 不限）
* 车队与载重：`light_1` / `light_2` 20 kg；`heavy_1` 150 kg；`omni_1` 90 kg
* 任务点坐标（`geomap_fleet.sdf` 里的 10 个 zone 标记，`tools/pick_zones.py` 选的）：

| | A | B | C |
| --- | --- | --- | --- |
| pickup | (10.60, -0.43) | (1.70, 4.22) | (9.10, 4.22) |
| dropoff | (8.10, -0.43) | (5.40, 4.22) | (11.00, 4.22) |

* 充电位（每台车出生点就是自己的充电位）：`charger_A` (-0.30, -0.75) → light_1、
  `charger_B` (1.20, -0.95) → light_2、`charger_C` (2.70, -0.85) → heavy_1、
  `charger_D` (4.20, -0.85) → omni_1

---

## 5. 观察

```bash
ros2 topic echo /fleet_status --once        # 车队快照：状态、电量、队列、makespan、reschedule
ros2 topic echo /task_markers --once        # RViz 面板与任务标签的数据源
ros2 topic echo /light_1/odom               # 真值里程计（Nav2 用的就是它）
ros2 topic echo /light_1/scan               # 激光
ros2 action list                            # 应含 4 条 navigate_to_pose
ros2 topic echo /gz/dynamic_pose            # Gazebo 真值位姿数组（见下方说明）
```

> ⚠️ `bridge.yaml` 里虽然给每台车写了 `/<ns>/gz_pose` 与 `/<ns>/gz_odom`，但 Gazebo 侧的
> `/model/<ns>/pose`、`/model/<ns>/odometry` 在本世界里**不发布数据**，这两组话题实测
> `ros2 topic hz` 无输出。真值来源是 `/gz/dynamic_pose`（`Pose_V → TFMessage`），
> 4 个 relay 从里面按出生点挑出自己那台车，转成 `/<ns>/odom` 和 `odom→base_footprint`。

RViz（`src/hetero_fleet_bringup/rviz/fleet.rviz`）里可以看到：`/map`、TF、3 台车的
RobotModel、4 路激光、规划路径与代价地图，以及 `task_markers`——任务线段（颜色随状态变化）、
任务编号/状态，以及四角固定的每车状态块（电量、载荷、当前任务；颜色与车体一致）。

状态块的位置可以不改代码调：

```bash
ros2 run hetero_fleet_scheduler fleet_manager --ros-args \
  -p status_panel_x_min:=-11 -p status_panel_x_max:=14.4 \
  -p status_panel_y_min:=-3 -p status_panel_y_max:=6.9 \
  -p status_panel_inset_m:=2.6 \
  -p status_panel_pos_overrides:=['omni_1:6.0,0.5']
```

---

## 6. 分层调试

```bash
# 只起 Gazebo + 3 台车（控制器默认不启动，要显式打开）
ros2 launch hetero_fleet_gz gz_sim.launch.py start_controllers:=true

# 只起 Nav2（假设仿真已在跑）
ros2 launch hetero_fleet_bringup nav2.launch.py

# 不重启仿真，只重启调度器并自动回放场景
ros2 launch hetero_fleet_scheduler scheduler.launch.py \
  tasks_file:=$PWD/src/hetero_fleet_scheduler/config/tasks/demo_10.yaml \
  autostart_tasks:=true task_interval_sec:=8

# 绕过调度器，单测某台车的导航
ros2 action send_goal /light_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 8.10, y: -0.43}, orientation: {w: 1.0}}}}"

# 只问规划器「能不能到」，不真的开车
ros2 action send_goal /heavy_1/compute_path_to_pose nav2_msgs/action/ComputePathToPose \
"{goal: {header: {frame_id: map}, pose: {position: {x: 11.0, y: 4.22}, orientation: {w: 1.0}}}, start: {header: {frame_id: map}}, use_start: false}"
```

`tools/` 下的诊断脚本（详见 §7 与各自 docstring）：

```bash
# 场景静态自检：地图 × 车型 × 任务点
python3 tools/verify_scene.py
# 两台车实际靠多近（车队分离度）
python3 tools/fleet_separation.py
# 记录「指令 vs 实际运动」，用于判断车是不是真的卡了
python3 tools/drive_trace.py --robot omni_1 --seconds 60
# TF 说法 vs 真实位姿
python3 tools/tf_probe.py --robot light_1
```

---

## 7. 项目结构

```
hetero_fleet_geomap_ws/
├── colcon_defaults.yaml               # 固定系统 Python（见 §1）
├── README.md / RUNBOOK.md / MAP_SWAP.md
├── assets/scan_sources/               # ★ 原始扫描模型（管线第 1 步的输入，见 §12）
│   ├── geomap_final_noroof/           #   当前地图的源（OBJ 6.5 MB + 贴图 atlas + model.sdf）
│   └── geomap_colour_noroof/          #   换图前的旧地图源（11.7 MB，留作可复现）
├── metrics/                           # 每次运行落盘的指标（§11，不进 git）
├── generated/                         # 每次 launch 渲染出的配置（不进 git）
│   ├── controllers/                   #   每车一份 ros2_control 控制器 YAML
│   └── params/                        #   每车一份 Nav2 参数（已改写命名空间）
├── tools/                             # ★ 离线工具（地图处理 + 诊断），不属于任何 ROS 包
│   ├── prepare_map_mesh.py            #   原始扫描 → 可驱动的地图模型（第 1 步）
│   ├── make_nav2_map.py               #   地图模型 → 占据栅格 maps/geomap.pgm（第 2 步）
│   ├── make_collision_boxes.py        #   栅格 → 盒体碰撞（第 3 步）
│   ├── pick_zones.py                  #   从栅格挑出生点/任务点/充电位
│   ├── verify_scene.py                #   地图 × 车型 × 任务点 的静态自检
│   ├── measure_footprint.py           #   从 URDF 量出真实足迹（robot_profiles 的来源）
│   ├── fleet_separation.py            #   车队最小间距统计
│   ├── drive_trace.py / chain_probe.py / topic_probe.py / tf_probe.py / omni_probe.py
│   ├── analyze_map.py / map_common.py #   栅格/网格分析，被上面几个脚本复用
│   └── （脚本的临时输出写到 generated/，不落 /tmp）
└── src/
    ├── hetero_fleet_msgs/             # 接口（Task / RobotState / FleetStatus / DispatchTask）
    ├── hetero_fleet_description/      # 模型与能力参数
    │   ├── urdf/common.xacro          #   共享宏（ros2_control / 摩擦 / 材质）
    │   ├── urdf/*_amr.xacro           #   三种机器人（几何 = 原设计 × 缩放系数）
    │   ├── config/*_controllers.yaml  #   每型的 ros2_control 控制器模板
    │   └── config/robot_profiles.yaml #   ★ 唯一参数真源（车队/能力/权重/充电位）
    ├── hetero_fleet_gz/               # 场景
    │   ├── models/geomap_final_noroof/#   处理后的地图模型：视觉网格 + 501 个盒体碰撞
    │   ├── models/geomap_colour_noroof/ #  旧地图模型（当前世界不用）
    │   ├── models/geomap_walls/       #   备用墙体模型（当前世界不用）
    │   ├── worlds/source/             #   ★ 桌面上的两个 viewer 世界（原始输入）
    │   ├── worlds/make_geomap_world.py#   生成 geomap_fleet.sdf（第 4 步）
    │   ├── worlds/geomap_fleet.sdf    #   生成结果（已入库）
    │   └── launch/gz_sim.launch.py    #   Gazebo + spawn 4 车 + robot_state_publisher
    ├── hetero_fleet_bringup/          # 桥接 / 导航 / 可视化
    │   ├── config/bridge.yaml         #   ros_gz 话题映射
    │   ├── config/nav2_params_*.yaml  #   ★ 三套差异化 Nav2 参数
    │   ├── launch/bringup.launch.py   #   总入口
    │   ├── launch/nav2.launch.py      #   每车一套 Nav2（含命名空间改写）
    │   ├── maps/geomap.{pgm,yaml}     #   由扫描网格栅格化的占据地图（520 × 207 @ 0.05 m）
    │   ├── rviz/fleet.rviz
    │   └── scripts/                   #   运行时节点：scan_frame_relay / omni_odom_relay /
    │                                  #   cmd_vel_stamper / 两个 watchdog
    └── hetero_fleet_scheduler/        # 调度
        ├── hetero_fleet_scheduler/
        │   ├── task_model.py          #   任务/机器人/指标数据类（无 ROS）
        │   ├── cost.py                #   ★ 可解释成本函数（无 ROS）
        │   ├── fleet_manager.py       #   ROS 节点 + 状态机 + 对峙处理 + 回充电位 + 指标
        │   ├── metrics.py             #   指标记录/导出（JSON + CSV）
        │   └── task_publisher.py      #   任务注入
        ├── config/tasks/*.yaml        #   任务场景（demo_10 / mixed_20）
        └── test/                      #   test_cost.py + test_standoff.py（39 个用例）
```

---

## 8. 异构性体现在哪

**三类机器人参数**（`robot_profiles.yaml`；三台车 = 1 light + 1 heavy + 1 omni）：

| | light_diff_amr | heavy_diff_amr | omni_mecanum_amr |
| --- | --- | --- | --- |
| 实例 | `light_1` | `heavy_1` | `omni_1` |
| max_speed | 0.70 m/s | **0.45 m/s** | 0.55 m/s |
| max_accel / max_omega | 1.0 m/s² / 1.5 rad/s | 0.5 / 0.8 | 0.5 / 1.2 |
| payload | 20 kg | **150 kg** | 90 kg |
| footprint（真实外形） | 0.6888 × 0.5412 | **0.8556 × 0.6448** | 0.672 × 0.588 |
| battery | 480 Wh | 1200 Wh | 640 Wh |
| 运动学 | 差速 | 差速 | **mecanum（可侧移）** |
| 能力 | `light_payload`, `narrow_aisle` | `heavy_payload` | `precise_docking`, `omnidirectional` |
| 每米能耗 | 0.85 + 0.012·kg Wh/m | 1.80 + 0.020·kg Wh/m | 1.10 + 0.015·kg Wh/m |

（2026-10-10 将此前车型整体放大两倍，当前缩放系数 light=0.82、heavy=0.62、omni=0.70；
`tools/measure_footprint.py` 能量出 URDF 的真实外接尺寸，就是这张表的来源。）

**三套 Nav2 参数**（`nav2_params_{light,heavy,omni}.yaml`）：

| 参数 | light | heavy | omni |
| --- | --- | --- | --- |
| 足迹多边形（半长 × 半宽） | 0.3444 × 0.2706 | **0.4278 × 0.3224** | 0.336 × 0.294 |
| `footprint_padding` | 0.06 | 0.03 | 0.02 |
| `inflation_radius` | 0.60 | 0.60 | 0.60 |
| `cost_scaling_factor` | 3.0 | 1.5 | 2.5 |
| `vx_max` | 0.70 | **0.45** | 0.55 |
| `vy_max`（侧移） | 0.0（禁用） | 0.0（禁用） | **0.40** |
| `wz_max` | 1.5 | 0.8 | 1.2 |
| MPPI `motion_model` | DiffDrive | DiffDrive | Omni |

和三型差异有关的两点，容易踩：

* `inflation_radius` 三型**相同（0.60）**，差异化在 footprint、`footprint_padding`、
  `cost_scaling_factor` 与速度上限上。旧文档里那张 0.17/0.20/0.21 的表已经不适用。
* `omni_1` 的 MPPI 使用 `Omni`。标准 X 型滚子接触方向由 Gazebo 插件固定在
  车身坐标系，四轮旋转不会改变摩擦自由方向。侧移与直行响应约为指令值的 100%，
  因此移除旧的 y 符号镜像；yaw 采用 0.95 增益校正。预测与速度平滑采用一致的
  0.5 m/s² 平动加速度，减少反复横向修正。
* 每台车自己的可执行任务队列清空后独立回充电位：任务点附近约 1 s，一般空闲约
  3 s。其他机器人继续执行任务不会阻止返航。

**谁走哪条道不是特判，而是参数的后果**：`tools/verify_scene.py` 用同一套 footprint +
inflation 去检查每个任务点的净空，`tools/pick_zones.py` 也是在这个约束下挑点的——所以
「重型车进不去的窄处」在离线阶段就已经被排除掉了。

---

## 9. 任务模型、状态机与两个附加行为

任务字段见 `msg/Task.msg`；场景 YAML 写法：

```yaml
- id: task_004
  type: transport
  pickup: [10.60, -0.43, 0.0]    # x, y, yaw（pickup_A）
  dropoff: [5.40, 4.22, 0.0]     # dropoff_B
  payload_kg: 120.0
  required_capability: heavy_payload
  priority: 4                     # 1..5
  deadline_sec: 420.0             # 从任务到达算起的软截止
```

状态机：

```
PENDING → ASSIGNED → TO_PICKUP → LOADING → TO_DROPOFF → COMPLETED
                          ↘ FAILED / REQUEUED
```

* 导航失败会**重排队一次**（`REQUEUED` → `PENDING`），再失败才 `FAILED`。
* 电量低于 `min_battery_pct`（默认 15%）的车不会被派新任务。
* 没有任何车能满足的任务会一直留在 PENDING 队列，并在日志给出原因，例如
  `no robot for task_010: ... payload 200.0kg > capacity 150.0kg`。
* 只有 IDLE 的车会被派活；同一个 zone 被两车同时使用时，按 1.0 m 间距换一个 approach
  slot（`zone_slot_spacing`，比仓库版的 1.4 m 小，因为这张图的走廊窄，1.4 m 的偏移
  经常直接落到墙里）。

**① 对峙 / 卡死处理（这个工作空间特有的）**

扫描地图的走廊只够两台车勉强错车，所以「两台车对着顶」和「车已经站在任务点上但 Nav2
一直不宣布成功」是这里的常见故障。`fleet_manager` 用一个停滞看门狗处理：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `stall_timeout_sec` | 15 s | 多久没「更接近目标」就算卡住 |
| `stall_move_eps_m` | 0.10 m | 算作进展的最小改善量 |
| `stall_yaw_weight` | 0.30 m/rad | 把「转正朝向」也算成进展（否则终点原地对齐会被误判成卡死） |
| `stall_wander_m` | 1.20 m | 绕路时离上次进展点超过这个距离 → 重新计时，不算卡死 |
| `stall_arrive_radius_m` | 0.30 m | 卡住时离目标这么近，直接**接受到达**而不是倒退 |
| `stall_backoff_m` | 0.55 m | 解对峙时后退的距离；每次重试按 `stall_backoff_growth`(0.6) 递增，上限 1.30 m |
| `stall_max_recoveries` | 3 | 同一段路最多救几次，之后判失败（任务重排队，车从充电位重新出发） |

**② 空闲回充电位**

每台车的出场点就是它的充电位（`robot_profiles.yaml` 里的 `charger:` 键，与 `spawn:` 相同）。
车做完活**不会立刻回**——任务流每几秒就来一条，中途回家会让每条任务都变成 5–15 m 的
空驶；而是：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `return_to_charger` | true | 总开关 |
| `dock_when_idle_sec` | 20 s | 这台车**手上没活可接**且空闲这么久才回桩 |
| `clear_task_point_sec` | 5 s | 但若它正停在 pickup/dropoff 点上，阈值缩短到这个值（不能占着任务点） |
| `clear_task_point_radius_m` | 1.2 m | 判定「停在任务点上」的半径 |
| `charger_arrive_radius_m` | 0.35 m | 离桩这么近就算已到家，不再折腾控制器 |

充电本身在仿真里是"停着不耗电"的语义（电量由 `battery_drain_per_meter_pct` 按里程扣），
没有实现充电功率曲线。

---

## 10. 调度算法：可解释成本函数

```python
cost(robot, task) = w_travel * 行驶时间
                  + w_energy * 能耗
                  + w_queue  * 排队等待
                  + w_late   * 迟到惩罚
                  + w_cap    * 能力/载荷/电量不满足（大惩罚）
```

权重在 `robot_profiles.yaml` 的 `scheduler.weights` 下：

| 权重 | 当前值 | 说明 |
| --- | --- | --- |
| `travel_time` | 1.5 | 主项：谁近谁去 |
| `energy` | 0.02 | 保留小项，让轻车在小件任务上排在重车前（0.85 vs 1.80 Wh/m） |
| `queue_wait` | 0.30 | **当前无效**：只有 IDLE 车会被派活，`busy_until` 恒等于 now |
| `capability_mismatch` | 1000.0 | 事实上的硬约束 |
| `lateness` | 2.0 | 再乘优先级系数 `1 + 0.25·(priority-1)` |

不可行组合用大惩罚而不是无穷大，所以仍会返回一个数并附带原因，日志里能直接看到
「为什么这台车被拒绝」：

```
ASSIGN light_1 -> task_001: total=... [travel=... energy=... wait=0.00 late=0.00]
no robot for task_005: omni_1 INFEASIBLE for task_005: lacks capability 'heavy_payload'
                       (has ['precise_docking', 'omnidirectional']); payload 95.0kg > capacity 90.0kg
```

**替换点**（后续演进，当前未实现）：`cost.py` 里的 `select_robot()` 是唯一决策入口，
换成拍卖 / OR-Tools CP-SAT / 滚动时域只改这一个函数，ROS 侧不动。

---

## 11. 指标

`fleet_manager` 每 `report_period_sec`（默认 15 s）打印一行汇总，并**每次都落盘**
（中途崩溃也留得下数据；`Ctrl-C` 时还会补写一次）：

```bash
cat metrics/metrics.json    # summary（含 per_robot）+ 每个任务明细
cat metrics/tasks.csv       # 逐任务一行，直接喂 pandas/matplotlib
```

**输出目录默认是 `<workspace>/metrics`**，即
`/home/xuzhijie/hetero_fleet_geomap_ws/metrics/`（与 `src/`、`build/`、`install/` 同级，
已被 `.gitignore` 忽略）。三种改法：

```bash
# 1) 环境变量（对整个 launch 生效）
HETERO_FLEET_METRICS_DIR=/tmp/run_a \
  ros2 launch hetero_fleet_bringup bringup.launch.py
# 2) 只重启调度器时传参
ros2 launch hetero_fleet_scheduler scheduler.launch.py metrics_dir:=/tmp/run_b
# 3) 直接起节点
ros2 run hetero_fleet_scheduler fleet_manager --ros-args -p metrics_dir:=/tmp/run_c
```

记录的量：任务完成率、平均完成时间、makespan、总行驶距离、总能耗、机器人利用率、
重调度次数、迟到任务数，以及每车的完成数 / 距离 / 能耗 / 电量 / 忙碌时间。

**本机一次实测**（`demo_10.yaml`，间隔 8 s 注入，headless，Nav2 真的在开车）：

```
completed=8/10 rate=80% mean_t=141.4s makespan=308.6s
dist=91.8m energy=128.5Wh util=64% resched=19 tardy=1
```

| robot | tasks | distance | energy | battery | busy |
| --- | --- | --- | --- | --- | --- |
| light_1 | 3 | 32.0 m | 29.3 Wh | 92.0% | 298.6 s |
| light_2 | 2 | 29.0 m | 25.8 Wh | 92.8% | 193.4 s |
| heavy_1 | 2 | 21.1 m | 61.4 Wh | 94.7% | 196.3 s |
| omni_1 | 1 | 9.8 m | 12.1 Wh | 97.6% | 103.1 s |

怎么读这组数字：

* **能力/载重过滤是对的**：120 kg 的 `task_004` 只可能给 `heavy_1`；95 kg 的 `task_005`
  明确以 `payload 95.0kg > capacity 90.0kg` 拒掉 `omni_1`；200 kg 的 `task_010`
  从头到尾没被派出去，运行结束时队列里剩的就是它。
* **能耗差异明显**：`heavy_1` 走 21.1 m 花 61.4 Wh，`light_1` 走 32.0 m 只花 29.3 Wh——
  这正是能量模型想要的（重车装重货，每米能耗高一个量级）。
* **迟到统计生效**：`task_008`（60 s 截止、优先级 5）被判 tardy。
* **8/10 不是"两条没人干"**：`task_007` 是 `omni_1` 卡在离 dropoff_C 只有
  0.39–0.44 m 的地方，停滞看门狗连续触发 `recovery 1/3`、`2/3`，最后 requeue 了一次。
  也就是 **9 条可完成任务里完成 8 条**，剩下那条卡在这张窄图的角落——与 §13 记的
  「吞吐/堵车」是同一个已知限制。

> 没有 Nav2 时（例如只想跑调度实验），`fleet_manager` 会退化成按 `距离/最大速度` 的
> 运动学估算，任务仍会「完成」——**那种数字不是导航结果，不要混用**。

**异构比例对比**：改 `robot_profiles.yaml` 的 `fleet:` 列表增删实例（例如 2 heavy），
重跑同一份 `mixed_20.yaml`，对比 `metrics.json` 即可。注意增删实例后要同步
`tools/pick_zones.py` 挑出的出生点数量。

---

## 12. 地图与世界的生成链（全部输入都在仓库内）

扫描地图是**四步**生成的，顺序不能反；前三步的输入现在是仓库里的
`assets/scan_sources/`，不再依赖桌面或别的项目目录。

**下面这组参数就是仓库里现有产物的参数**（2026-09-24 实测：用它们重跑，视觉网格、
碰撞网格、占据栅格、`model.sdf` 四份产物都与入库文件**逐字节一致**）：

```bash
cd /home/xuzhijie/hetero_fleet_geomap_ws

# 1) 原始扫描 → 可驱动的地图模型（拟合地板、剔除地板面、烘焙水平变换）
python3 tools/prepare_map_mesh.py \
  assets/scan_sources/geomap_final_noroof/meshes/geomap_final_noroof.obj \
  src/hetero_fleet_gz/models/geomap_final_noroof \
  --name geomap_final_noroof --keep-above 0.40

# 2) 模型 → 占据栅格（Nav2 用的地图）+ 报告
python3 tools/make_nav2_map.py \
  src/hetero_fleet_gz/models/geomap_final_noroof/meshes/geomap_final_noroof.obj \
  src/hetero_fleet_gz/models/geomap_final_noroof/meshes/geomap_final_noroof_collision.obj \
  src/hetero_fleet_bringup/maps/geomap --report --smooth 1 --structure-dilate 0

# 3) 栅格 → 盒体碰撞（写回 model.sdf；DART 不能用 20 万三角面，见下）
#    必须在仓库根目录执行：--map 默认是相对路径 src/hetero_fleet_bringup/maps/geomap
python3 tools/make_collision_boxes.py \
  --model src/hetero_fleet_gz/models/geomap_final_noroof/model.sdf

# 4) 世界文件（含 10 个 zone 标记）
python3 src/hetero_fleet_gz/worlds/make_geomap_world.py

# 附带：重新挑点 / 静态自检
python3 tools/pick_zones.py src/hetero_fleet_description/config/robot_profiles.yaml \
  --model-name geomap_final_noroof --spacing 2.5 --spot-margin 0.25
python3 tools/verify_scene.py
```

两个容易踩的点：

* `--keep-above` **不是越小越安全**。`1.0`（按"机器人能否从下面钻过去"来选）会让雷达
  扫到大量地图里没有的低矮家具，Nav2 在空旷处直接报 `no valid path found`；
  `0.40`（贴近雷达高度）才是这张图在用的值。
* 第 1 步的**输出目录名会成为 SDF 里的模型名**（`model://geomap_final_noroof`），
  所以别把它改成别的目录名，否则第 4 步生成的世界会指向一个不存在的模型。

改了世界或地图记得 `colcon build`（`models/`、`worlds/`、`maps/` 都要装进 `install/`）。

为什么必须是盒体碰撞：地图的三角网格有 20 万+面，本机 DART 直接用网格碰撞会段错误，
而 Bullet-Featherstone 虽然不崩，却**无法让轮式底盘原地转向**（轮子按指令反向转、车身
yaw 不动，所有需要转向的目标都以 `Failed to make progress` 结束）。现在改成：DART +
由占据栅格生成的 501 个轴对齐盒体，物理与代价地图描述的是同一个房间。
完整排查过程见 `MAP_SWAP.md` 第 6 节。

---

## 13. 已验证 / 未验证

**本次实测通过（2026-09-24，独立 `GZ_PARTITION` 的干净实例）**

* `colcon build` → `Summary: 5 packages finished`
* 单元测试 **39 passed**（成本函数 + 对峙/充电行为）
* 4 台车全部 spawn 成功；8 个控制器（4× `joint_state_broadcaster` + 3× `diff_drive_controller`
  + 1× `mecanum_drive_controller`）全部 `Configured and activated`
* 4 套 Nav2 全部 active，`ros2 action list` 恰 4 条 `navigate_to_pose`
* 世界文件**可通过 `make_geomap_world.py` 重新生成**，结果与入库文件一致，
  且是**合法 XML**（`xmllint` 通过）
* 地图：520 × 207 @ 0.05 m = 26.0 × 10.4 m，origin `[-11.3429, -3.2513]`，可通行 99.1 m²
* 世界：world `geomap_fleet`，1 个 include（`model://geomap_final_noroof`）+ 地面 +
  10 个 zone 标记（3 pickup / 3 dropoff / 4 charger），插件 Physics(DART) / Sensors(ogre) /
  SceneBroadcaster / UserCommands / Contact
* 地图模型：1 个视觉网格 + **501 个盒体碰撞**
* 指标默认落到 `<workspace>/metrics`、渲染配置默认落到 `<workspace>/generated/{controllers,params}`
* **端到端跑通一次 `demo_10`（真实导航，不是运动学估算）**：8/10 完成、0 永久失败、
  makespan 308.6 s、总里程 91.8 m、利用率 64%；9 条可完成任务里 8 条完成，剩下 `task_007`
  卡在离目标 0.4 m 处（停滞看门狗跑了 3 次 recovery 后 requeue）。明细见 §11
* 地图/世界的输入现在**全部在仓库内**：原始扫描在 `assets/scan_sources/`，
  两个 viewer 世界在 `src/hetero_fleet_gz/worlds/source/`；按 §12 的参数重跑前三步，
  **视觉网格 / 碰撞网格 / 占据栅格 / model.sdf 与入库产物逐字节一致**
* 世界文件重新生成后与入库文件一致（sha256 不变），且 `xmllint` 能解析
  （以前那条注释里的 `--render-engine` 是非法 XML，已修）

**尚未验证 / 存疑**

* **GUI 路径**：本次全程 `headless:=true rviz:=false`；扫描图的颜色需要 `ogre`(1) 引擎，
  这条（`gz_sim.launch.py` 已传 `--render-engine ogre`）没有在图形界面里复核。
* **吞吐/堵车**：这张图是窄走廊，调度器不考虑交通，多台车可能在同一角落互堵；
  对峙看门狗会解，但会增加单任务耗时。相关测量见 `MAP_SWAP.md` 6.x，属已知限制。
* **长跑稳定性**：只跑了单次会话。
* `models/geomap_colour_noroof/` 与 `models/geomap_walls/` 当前世界都不用，是换图前的遗留；
  保留是为了让 `MAP_SWAP.md` 记录的旧管线仍可复现。

**已知但无害的小问题**

* `light_*` 的 `planner_server` 会打印 `Inflation layer either not found or inflation is
  not set sufficiently ...` 的 ERROR：`inflation_radius` 0.35 相对足迹偏小。实测不影响
  规划（能正常出路径），想消掉就把它抬到足迹最大半宽（≈0.18）以上更多一些。
* `bt_navigator` 会打印 `Error_code parameters were not set`，用默认值即可。

---

## 14. 常见问题

**`ModuleNotFoundError: No module named 'em'`**
→ 见 §1。确认在**工作空间根目录**编译，且 `colcon_defaults.yaml` 存在。

**`Unable to parse the value of parameter robot_description as yaml`**
→ 已修复。xacro 输出的是 XML 不是 YAML，必须用
`ParameterValue(robot_desc, value_type=str)` 包裹。

**`Failed to load system plugin [gz_ros2_control-system]`**
→ 已修复两处：① 用 `ros_gz_sim` 自带的 `gz_sim.launch.py`（它会设
`GZ_SIM_SYSTEM_PLUGIN_PATH`），不要自己 `ExecuteProcess(["gz","sim",...])`；
② URDF 里 `<gazebo><plugin>` 必须是系统插件类名
`gz_ros2_control::GazeboSimROS2ControlPlugin`，不是 `gz_ros2_control::GazeboSimSystem`。

**`The 'type' param was not defined for 'diff_drive_controller'`**
→ 控制器 YAML 里每个控制器必须同时有 `type:` 和 `ros__parameters:`，同级。

**`Parameter 'left_wheel_names' cannot be empty`**
→ 参数层级放深了，必须在 `<控制器名>/ros__parameters/` 下。

**`Switch controller timed out after 5 seconds`**
→ spawner 起太早，`/clock` 还没通。`bringup.launch.py` 在 13 s 时统一启动；
单独用 `gz_sim.launch.py` 时要加 `start_controllers:=true`（否则控制器不启动）。

**`Failed to acquire lock in 20 seconds` / spawner 连环超时**
→ **不是 `/dev/shm` 的问题**。spawner 之间靠
`~/.ros/locks/ros2-control-controller-spawner.lock` 串行化（源码
`controller_manager/spawner.py`：等 20 s、重试 5 次）。第一个 spawner 长时间占锁
（通常是 `controller_manager` 在等 `/clock`）就会连环超时。真正的根因在工作空间之外：
见下一条。

**两个 `gz sim` 实例 / 车不动 / TF 报 `two or more unconnected trees`，
日志出现 `Found additional publishers on /clock`**
→ 这台机器上常有**别的会话遗留的 `gz sim`**（本机确实有一个来自
`~/Documents/Codex/2026-09-18/pcd-gazebo` 的实例）。两个实例共用同一个 Gazebo
transport 分区时，`/clock`、`/stats`、`/world/*/dynamic_pose/info` 会互相污染：
本空间的表现是 4 个里程计 relay 收到 `transforms: []` → `/<ns>/odom` 与 TF 全空 →
Nav2 一直 `Timed out waiting for transform`、spawner 连环超时。处理：

```bash
ps -eo pid,etimes,args | grep "gz sim"     # etimes 很长 = 上个会话的残留
pkill -INT -f "gz sim"                     # 确认不是自己正在用的实例
```

或者给本次运行一个独立分区：

```bash
export GZ_PARTITION=geomap_$(date +%s)
ros2 launch hetero_fleet_bringup bringup.launch.py headless:=true rviz:=false
```

**车只会直走 / 原地转不动 / 任务总是 `Failed to make progress`**
→ 先确认物理引擎：

```bash
grep -n "engine" src/hetero_fleet_gz/worlds/geomap_fleet.sdf   # 必须是 dartsim
```

Bullet-Featherstone 在本机无法让轮式底盘原地转向（连 Gazebo 官方 diff-drive 示例
强制 Bullet 也一样）。改了引擎或地图后要 `make_geomap_world.py` 重生成世界 + `colcon build`。
另外**不要用 `/joint_states` 判断轮子转没转**（本机 gz_ros2_control 的关节反馈会回显指令）：
用 `gz model -m light_1 -p` 读模型位姿，或用 `tools/drive_trace.py` 对比指令与实测。

**`ros2 run hetero_fleet_scheduler task_publisher` 找不到包**
→ 忘了 `source install/setup.bash`。本工作空间安装后**没有** `install/.../bin/`，
脚本在 `install/hetero_fleet_scheduler/lib/hetero_fleet_scheduler/` 下；实测
`ros2 run` 正常（`ros2 pkg executables` 能列出 `fleet_manager`、`task_publisher`）。

**车悬浮 / 陷进地面**
→ 扫描地板已校平到 z≈0，世界的驱动面在 z = 1.117e-01（见 `make_geomap_world.py`
的 `MAP_Z` / `GROUND_Z`）。换新图时重跑 §12 的四步即可，第 1 步会自己拟合地板平面。

**想用 SLAM 建图替代离线栅格化**
→ 当前 `maps/geomap.pgm` 是把扫描网格按"机器人高度以下"投影得到的。若要真实 SLAM：
起 Nav2 时加 `slam:=true`（需自行加 slam_toolbox 节点），遥控一台车走一圈，
`ros2 run nav2_map_server map_saver_cli -f maps/geomap`。

---

## 15. 后续演进路线

1. **交通 / 让行**：当前调度器不看路况，多车会挤同一角落。可以限制同区并发，或把
   collision_monitor 的"停车"改成"靠边让行"。
2. **充电语义**：现在回桩只是"停着不掉电"，可以加充电功率曲线与 `CHARGING` 状态联动。
3. **时间窗 / 容量 / 任务依赖**：`cost.py` 加约束项，或整体替换为 OR-Tools CP-SAT
   （`select_robot()` 是唯一入口）。
4. **动态插单 / 故障重调度**：滚动时域 = 周期性重跑 `_try_assign_all()` 并允许抢占。
5. **换新图**：把新扫描放进 `assets/scan_sources/`，按 §12 四步重跑，再
   `tools/pick_zones.py` 重新挑点即可（脚本都已参数化）。
