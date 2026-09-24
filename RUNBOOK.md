# RUNBOOK —— 换地图后的运行指令

这份工作空间 = 原来的 `hetero_fleet_ws`（仓库地图）换成了点云扫描地图
`geomap_final_noroof`，车队 **4 台**（light_1 / light_2 / heavy_1 / omni_1，三种构型）。
地图是怎么处理、尺寸怎么定的，见 [`MAP_SWAP.md`](MAP_SWAP.md)
（那是换图当时的历史记录，部分数字已被后续调整覆盖；**当前状态以 [`README.md`](README.md) 为准**）。

工作空间：`/home/xuzhijie/hetero_fleet_geomap_ws`
（原工作空间 `/home/xuzhijie/hetero_fleet_ws` 没有被改动）

## 0. 每个新终端的一次性环境准备

```bash
cd /home/xuzhijie/hetero_fleet_geomap_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 4 台车 = 4 套 Nav2 ≈ 100+ 个 ROS 节点。下面两个变量是**本机建议**：
# 2026-09-24 实测里不加也能完整启动（4 套 Nav2 全部 active、任务正常派发），
# 但如果同时有别的 ROS 程序在 domain 0 上发 /tf，会出现大量 TF_OLD_DATA、
# bt_navigator 报 "Initial robot pose is not available"、目标 0.15 s 内 ABORT
# 这类成批失败 —— 那时再加上它们：
export HETERO_FLEET_DDS_TRANSPORTS=UDPv4   # 绕开 FastRTPS 共享内存
export ROS_DOMAIN_ID=42                    # 与机器上其它 ROS 程序隔离
```

> `ROS_DOMAIN_ID=42` 不是可选项：本机同一个 domain 上还有别的 ROS 程序在发
> `/tf`（实测有 `link01`~`link06` 的机械臂 TF，时间戳停在 18 s），一次运行能刷出
> 9.9 万条 `TF_OLD_DATA`，把本仿真的 TF 缓冲冲垮。换 domain 后这些错误归零。
> 两个变量都要**在 launch 和 task_publisher 的每个终端里**设置。

* 不 source `install/setup.bash` 时 `ros2 run hetero_fleet_scheduler ...` 找不到包。
* 本机 conda 的 python3.14 抢 PATH，编译接口需要系统 python 3.12；根目录的
  `colcon_defaults.yaml` 已固定，只要**在根目录**执行 `colcon build` 即可。

## 1. 编译

```bash
cd /home/xuzhijie/hetero_fleet_geomap_ws
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

改 `.xacro / .yaml / .launch.py / worlds/*.sdf / scripts/*.py` 不必重编
（symlink install，重启 launch 即生效）；只有改 `setup.py` 的入口或加新文件才需要。
`models/` 目录（地图网格）必须编译过一次才会进 `install/`。

## 2. 启动

```bash
# 全栈：Gazebo GUI + RViz + 4 套 Nav2 + fleet_manager
ros2 launch hetero_fleet_bringup bringup.launch.py

# 省资源模式（长跑建议）
ros2 launch hetero_fleet_bringup bringup.launch.py headless:=true rviz:=false

# 只起仿真，手动喂任务
ros2 launch hetero_fleet_bringup bringup.launch.py scheduler:=false

# 只起 Gazebo + 车 + 控制器（不带 Nav2）
ros2 launch hetero_fleet_gz gz_sim.launch.py
```

发任务前先确认 Nav2 已就绪（**4 套栈**，每套错开 30 s，全部 active 约需 150 秒）：

```bash
ros2 lifecycle get /light_1/bt_navigator        # 期望 active [3]
ros2 action list | grep navigate_to_pose        # 期望 4 条
```

否则会出现 `goal rejected`，任务会被重排队。

## 3. 任务点与充电位（新地图）

六个取货/卸货点**沿回型东半圈交替铺开**——沿 +4.2 m 那条边自西向东，再沿南侧走廊
往西回来。相邻两点 1.9~3.7 m，排得比较松（原来取货点和卸货点是一对一对紧挨着的，
只有 2.2 m，四台车挤在同一个小角落正是实测中所有卡死的来源）。两处**不放点**：

* 南侧走廊 x≈5.5~6.8 那个最窄处（净空只有 0.6 m）；
* **东侧连接段**（x≈11.6~12.4，y −0.3~4.2）。它是回型走廊东端**唯一**的南北通道，
  `dropoff_C` 原来在 (12.05, 2.40) 正好堵在中间——实测一轮 42 次卡死里有 **25 次**
  发生在这里：有车在那儿作业就封了通道，南北往返必须跟它正面相遇，然后僵住。
  已挪到北侧走廊东端 (11.0, 4.22)，连接段空出来。

**四个充电位在南侧走廊贴南墙排成一排**（间距 1.5 m，都朝东），在 x≈5.5~6.8 那个
最窄处**西边**：那段走廊有 2.2~2.4 m 宽，停一列车之后上方还留 ≥1.0 m 的通行道，
所以不挡路；而且每台车能直接往东/往西拐出来，不用倒着从一排车后面挪出来
（原来那种首尾相接的摆法就有这个问题）。东段南墙能排开车的地方只有 3.2 m，只放得下
3 台。

| 区域 | 坐标 (x, y) | 净空 (m) |
| --- | --- | --- |
| pickup_A | (10.60,-0.43) | 0.96 |
| pickup_B | ( 1.70, 4.22) | 0.85 |
| pickup_C | ( 9.10, 4.22) | 0.85 |
| dropoff_A | ( 8.10,-0.43) | 1.10 |
| dropoff_B | ( 5.40, 4.22) | 0.95 |
| dropoff_C | (11.00, 4.22) | 0.97 |
| charger_A | (-0.30,-0.75) | 0.72 |
| charger_B | ( 1.20,-0.95) | 0.70 |
| charger_C | ( 2.70,-0.85) | 0.70 |
| charger_D | ( 4.20,-0.85) | 0.70 |

这些坐标全部经过 `tools/verify_scene.py` 校验：在自由格上、净空足够四种机型
（最宽的 omni 需要 0.53 m）、且落在同一个连通的可行驶区域里。四个充电位相邻间距
1.50 m。

**出生点 = 充电位**，一车一位（`robot_profiles.yaml` 里每台车同时写了
`spawn` 和 `charger`，两个值必须一致）：

| 机器人 | 充电位 |
| --- | --- |
| light_1 | charger_A (-0.30,-0.75) 排头（西端） |
| light_2 | charger_B ( 1.20,-0.95) |
| heavy_1 | charger_C ( 2.70,-0.85) |
| omni_1 | charger_D ( 4.20,-0.85) 排尾（东端） |

地图是 `geomap_final_noroof`：26.0 × 10.3 m。

## 4. 发任务

唯一入口是话题 `/task_request`（`hetero_fleet_msgs/msg/Task`）。

```bash
# A. 单条（pickup_A -> dropoff_C，走廊全长）
ros2 run hetero_fleet_scheduler task_publisher --single \
  --capability light_payload --payload 12 \
  --pickup 4.00 -0.50 --dropoff 11.79 0.50 --priority 2 --deadline 900

# B. 预设 10 任务场景（含 1 条故意不可完成的任务）
ros2 run hetero_fleet_scheduler task_publisher \
  --scenario src/hetero_fleet_scheduler/config/tasks/demo_10.yaml --interval 8

# C. 20 任务统计场景
ros2 run hetero_fleet_scheduler task_publisher \
  --scenario src/hetero_fleet_scheduler/config/tasks/mixed_20.yaml --interval 6 --shuffle

# D. 直接 pub（id 必须唯一）
ros2 topic pub --once /task_request hetero_fleet_msgs/msg/Task \
"{id: 'adhoc_1', type: 'transport', pickup: {x: 4.00, y: -0.50}, dropoff: {x: 11.79, y: 0.50}, payload_kg: 12.0, required_capability: 'light_payload', priority: 2, deadline_sec: 600.0}"
```

字段约定：

* `required_capability`：`light_payload` | `heavy_payload` | `narrow_aisle` |
  `precise_docking` | `omnidirectional` | `""`（空 = 不限）
* 车队（4 台）：`light_1` / `light_2` 各 20 kg & 0.70 m/s；`heavy_1` 150 kg & 0.45 m/s；
  `omni_1` 90 kg & 0.55 m/s（麦轮，可横移）
* 任务 `id` 重复会被忽略（日志出现 `duplicate task id`）

## 4.5 全部任务做完之后回充电位

机器人**出生在充电位**；任务期间就停在原地接着干下一单，**不是**每单回一次充电位。
只有同时满足下面三条才回：

1. **这辆车自己**没有能接的任务（`_work_for`；注意不是"整队都没活"）；
2. 空闲时间够了——一般是 `dock_when_idle_sec`（默认 **20 s**），但如果它正**站在
   取货/卸货点上**，只要 `clear_task_point_sec`（默认 **5 s**）；
3. 它当前不在自己的充电位上。

第 2 条里"站在作业点上"单独设一个更短的阈值是要害：一单干完，车通常就停在刚用过的
那个托盘上，而那正是它**最不该久留**的地方——下一个要用这个点的车只能排在它后面，
区域接近点的槽位逻辑本来就是为了避免这件事。任务全部做完之后，也就是靠这条规则让
车队在几秒内各自回桩，而不是杵在作业点上。

回充时状态显示 `CHARGING`。这段返航记进该车行驶距离与能耗（它确实是真跑），但
**不算进 `util`**——`util` 统计的是任务占用时间，而回充恰恰是"没在做任务"。

> 为什么不是"每单回充"：任务流是每 6 秒来一条，每单之间来回跑一趟充电位（这张图
> 单程 5~15 m）会把每次任务都变成"出去一趟再回来"，吞吐会被这趟空驶吃掉。
> 20 秒的空闲阈值足够跨过脚本任务之间几秒的间隔，又不至于让车在作业点上干等。
> `_work_for` 只统计"**这辆车**接得动"的任务，所以 `demo_10` 里那条永远做不了的
> 200 kg 任务不会把车队永远卡在外面，而只有 heavy_1 能接的重载任务也不会拖着
> light_1 一起留在过道里。

```bash
# 关掉回充位（一直停在原地）
ros2 run hetero_fleet_scheduler fleet_manager --ros-args -p return_to_charger:=false
# 调空闲多久算"没活了"（默认 20 s）
ros2 run hetero_fleet_scheduler fleet_manager --ros-args -p dock_when_idle_sec:=30
# 调"站在作业点上多久就得让位"（默认 5 s）和判定半径（默认 1.2 m）
ros2 run hetero_fleet_scheduler fleet_manager --ros-args \
  -p clear_task_point_sec:=3 -p clear_task_point_radius_m:=1.0
# 调"算不算已经到位"的半径（默认 0.35 m）
ros2 run hetero_fleet_scheduler fleet_manager --ros-args -p charger_arrive_radius_m:=0.5
```

> `spawn` 是里程计的锚点（`world = spawn + R(spawn_yaw) * odom`），`charger` 是返航
> 目标。两者不一致时 `fleet_manager` 会打 `ERROR ... spawn ... and charger ...
> disagree`，并把车停到"锚点算出来的地方"而不是充电位那格——改坐标时两个都要改。

## 4.6 卡死自动脱困（避障 + 反脱困）

两车在 1.2 m 宽的过道里对头、或某一台被碰撞监视器按在墙边时，Nav2 自己的恢复行为
（清代价地图、原地转、后退一点）对"对面真的站着一台车"是没用的，原来的表现是这台车
反复重规划同一条被堵的路，几十秒到一两分钟不动，后面整队排队。

现在 `fleet_manager` 会盯着**里程计真值**：某个目标在跑、但**与目标点的距离**在
`stall_timeout_sec` 内没有缩短 `stall_move_eps_m`，就判定卡死，然后

1. 抢占（cancel）当前目标，
2. 倒车 `stall_backoff_m`（重试越多倒得越远，上限 `stall_backoff_max_m`），方向在
   车尾 ±0°/±26°/±52° 里挑**离队友最远**且落在自由地面的那个，
3. 重新下发同一个目标重新规划。

先试的判据是"一段时间内位移不够"，实测不够：有车**已经站在卸货点上了**，只是
Nav2 一直不宣布成功，于是它在 0.1 m 范围内来回蹭，位移判据认为它在动，结果一条
腿卡了 670 s。所以判据改成了"离目标没有更近"。

另外还加了一条：如果判死的时候车**已经在目标点 `stall_arrive_radius_m` 以内**，
就不倒车——倒车等于让它离开自己已经停好的托盘——而是取消目标、认可这次到达。
这不是"假装到达"：判断用的仍是里程计真值，半径 0.30 m 是 Nav2 自身
`xy_goal_tolerance`（0.15 m）的两倍。实测这一条认可的距离是 0.04~0.25 m。

最多重试 `stall_max_recoveries` 次，之后才让这趟失败、任务回队列。日志里能看到：

```
light_1: no progress for 15s on t03:dropoff at (0.61,3.90), 0.30 m from the goal - recovery 1/3
light_1: reversing 0.88 m to (0.14,3.55) to break the standoff
omni_1:  t06:pickup within 0.08 m of the goal after 15s without progress - accepting the arrival
```

参数（都有默认值，平时不用动）：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `stall_timeout_sec` | 15.0 | 多久没有更接近目标算卡死；≤0 关闭该功能 |
| `stall_move_eps_m` | 0.10 | 算作"有进展"的最小接近量，米 |
| `stall_wander_m` | 1.20 | 离上次有进展的位置超过这个距离，视为在绕路，重新计时 |
| `stall_arrive_radius_m` | 0.30 | 卡死时已在这个半径内就认可到达，不倒车 |
| `stall_backoff_m` | 0.55 | 第一次倒车距离 |
| `stall_backoff_growth` | 0.6 | 每次重试倒车距离的增长系数 |
| `stall_backoff_max_m` | 1.30 | 倒车距离上限 |
| `stall_backoff_sec` | 15.0 | 单次倒车最多等多久 |
| `stall_max_recoveries` | 3 | 每段路最多脱困几次 |

```bash
# 调快/调慢
ros2 run hetero_fleet_scheduler fleet_manager --ros-args \
  -p stall_timeout_sec:=15.0 -p stall_max_recoveries:=4
```

## 5. 观察与取指标

```bash
ros2 topic echo /fleet_status --once        # 车队快照：状态、电量、队列、指标
ros2 topic echo /task_markers --once        # RViz 面板与任务标签的来源
ros2 action list | grep navigate_to_pose
cat metrics/metrics.json   # 默认 metrics_dir=<workspace>/metrics，每 15 s 落盘
cat metrics/tasks.csv
```

RViz 左侧 `task_markers` 显示每台车的状态块（固定在地图四角：
light_1 左上、light_2 右上、heavy_1 左下、omni_1 右下；字号 0.6 m），
以及每条任务的线段与状态标签。

调整面板位置/字号（不必改代码）：

```bash
ros2 run hetero_fleet_scheduler fleet_manager --ros-args \
  -p status_panel_x_min:=-11.0 -p status_panel_x_max:=14.4 \
  -p status_panel_y_min:=-3.0 -p status_panel_y_max:=6.9 \
  -p status_panel_text_height:=0.6
```

面板默认布局：四台车各占矩形一个角，再往矩形中心内缩 `status_panel_inset_m`
（默认 2.6 m）——直接钉在角上会落在未扫描的黑色区域里，和地图完全脱开。
这个矩形默认贴着地图边界（x -11.0~14.4，y -3.0~6.9），所以四个块都在地图内、
且离最近的任务点 2.1~7.5 m，不会压住机器人正在作业的托盘。要单独挪某一块：

```bash
# 绝对坐标（不是相对角点），名字用机器人名
-p status_panel_pos_overrides:="['omni_1:6.0,0.5']"
```

## 6. 分层调试

```bash
# 只有 Gazebo + 车 + 控制器
ros2 launch hetero_fleet_gz gz_sim.launch.py

# 只有 Nav2（假设仿真已在跑）
ros2 launch hetero_fleet_bringup nav2.launch.py

# 单测某台车的导航（绕过调度器）
ros2 action send_goal /omni_1/navigate_to_pose nav2_msgs/action/NavigateToPose \
"{pose: {header: {frame_id: map}, pose: {position: {x: 11.79, y: 0.50}, orientation: {w: 1.0}}}}"

# 控制器状态
ros2 control list_controllers -c /light_1/controller_manager
ros2 control list_controllers -c /omni_1/controller_manager

# 离线自检：地图 × 机器人 × 任务点是否自洽
python3 tools/verify_scene.py
```

## 7. 停止与彻底清理

前台运行的用 Ctrl-C；后台运行的直接跑：

```bash
cd /home/xuzhijie/hetero_fleet_geomap_ws
tools/cleanup_fleet.sh
```

它会先 `SIGINT` 掉 launch 树，再按可执行名强杀 Nav2 / Gazebo / 中继脚本，最后清掉
FastRTPS 残留的 `/dev/shm` 段，并打印还剩几个进程（期望 `0` 和 `0`）。

> **这一步不是可选的。** launch 树经常留下孤儿进程，最坏的不是它们占 CPU，而是
> `static_transform_publisher map <robot>/odom` 会继续用**旧的出生点**在同一个
> `ROS_DOMAIN_ID` 上发 TF。下一次启动时同一个 `map->odom` 有两份互相矛盾的广播，
> Nav2 取到哪份看运气，表现是每个目标都在 1 秒内失败：
>
> ```
> controller_server: Resulting plan has 0 poses in it.
> ```
>
> 看着像规划器 bug，其实是上个进程没死。同理，残留的第二个 `gz sim` 会让控制器
> spawner 挂到错误的那一个上。清理脚本专门处理这两类。

用可执行名 `pkill -x` 比 `pkill -f` 安全，不会误杀你自己的 shell。

## 8. 常见问题

| 现象 | 先查什么 |
| --- | --- |
| Gazebo 里没有地图（只有灰地面） | `GZ_SIM_RESOURCE_PATH` 是否含 `install/hetero_fleet_gz/share`（未编译过 `models/` 也会这样） |
| 地图渲染成纯白 | 是否带上 `--render-engine ogre`（`gz_sim.launch.py` 已默认加；本机 ogre2 不认网格材质） |
| 机器人执行任务时长时间杵在原地 | 多半是**互相锁死**：局部代价地图把队友（`scan_frame_relay` 注入的 `robot_obstacles`）膨胀成一大片致命格，MPPI 找不到可行轨迹（日志 `Optimizer reset`），两队谁都不让谁。已按 4 台车调过：队友障碍改用**内切半径**而不是外接半径、碰撞监控改成"提前减速 + 近身才停"（stop 多边形 ~0.3 m）、MPPI `collision_margin_distance` 0.1→0.05。实测静止时间从 90~100 s/150 s 降到 0~21 s |
| 车完全不动 | `ros2 lifecycle get /<robot>/controller_server` 是否 active；`ros2 control list_controllers -c /<robot>/controller_manager` 里驱动控制器和 joint_state_broadcaster 是否都 active（启动竞态会让 spawner 失败 → 重启） |
| 车直走正常、原地转不动 | 世界里的引擎必须是 `gz-physics-dartsim-plugin`（`grep engine src/hetero_fleet_gz/worlds/geomap_fleet.sdf`）。Bullet 在本机转不了，见 `MAP_SWAP.md` 6.0 |
| spawner 报 `Controller already loaded` + `Failed to configure controller` | 同时存在两个 `gz sim` 实例（`ps -eo comm | grep -c '^ruby$'` 应等于 1），先按第 7 节清理再启动 |
| 发了任务没反应 | `/task_request` 有没有订阅者（fleet_manager 在不在）；高负载下仍可能丢包，重发即可 |
| 日志 goal rejected | Nav2 未就绪或目标不可达；任务会自动重排队 |
| 日志 `goal is occupied` / 车原地转 | 任务点不在自由格上：跑 `python3 tools/verify_scene.py`，它会指出是哪一个点 |
| 日志 `no robot for task_x` | 没有车满足能力/载重/电量，任务留在 PENDING（demo 里故意放了一条 200 kg 的） |
| 车只在一条走廊里走来走去 | 正常：这张扫描图只有这一条连通可通行区域，见 `MAP_SWAP.md` 第 1、6 节 |
| 机器很卡 | 用 `headless:=true rviz:=false`；或进一步减少 `robot_profiles.yaml` 里的车队规模 |

## 9. 本项目（换地图前）踩过并已修复的坑

1. 占据栅格极性反了（黑底白障碍 + `negate: 0`）→ 整个地面被当作障碍，所有目标报
   `205 START_OCCUPIED`。新地图沿用正确约定，`tools/make_nav2_map.py` 里有说明。
2. light AMR 质心压在轮轴上、唯一万向轮在车尾 → 车体永久前倾 16°，雷达跟着扫地面。
   已把质心后移（换地图后按 0.70 缩放后仍保留这个偏移）。
3. 三台差速车只有开环里程计 → 已给每台车加 Gazebo 真值里程计 relay（`bridge.launch.py`）。
4. AMCL 的 `odom_frame_id / base_frame_id` 没带命名空间 → 激光全被丢弃。已修。
5. `mecanum_drive_controller` 订阅 `<controller>/reference` 而非 `cmd_vel` → omni 收不到指令。
   已在 `cmd_vel_stamper` 里按机型选话题。
6. omni 的麦轮没有滚子模型、也没设摩擦方向 `fdir1` → 连原地转都做不到。已补 45° 摩擦模型。
7. `fleet_manager` 在 goal 被拒绝时会用运动学模拟"跑完"这一段 → 已改成如实失败并重排队。
8. `task_publisher` 只发一次消息，重负载下会静默丢包 → 已改成等订阅者 + 重发 4 次。

换地图过程中新踩的坑记在 `MAP_SWAP.md`（倾斜地板、竖直面俯视投影退化、俯视投影把
头顶结构也算成障碍、以及 y 轴坐标镜像的换算错误）。
