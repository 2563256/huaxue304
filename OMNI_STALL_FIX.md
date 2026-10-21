# 副本A：omni 转弯停滞诊断与恢复修复

## 现场证据

2026-10-18 对正在运行的副本A诊断：

- 初次观测 omni 位于东侧连接段/北侧走廊转角附近，实际世界位置约 (11.8,3.6)。
- 它持有合并通道令牌，中央预约器 `allowed=true`、`waiting=false`。
- Nav2、平滑器、中央过滤、本地看门狗、最终 cmd_vel 均有消息；速度只有毫米每秒量级，
  真值里程计也显示几乎不动。并非缺少控制器或通道拒绝。
- controller_server 多次报告 `Failed to make progress`；一次恢复还出现 `Resulting plan has 0 poses`。
- 读出的实际姿态 (11.8176,3.6632,1.0882) 在静态地图上的完整 footprint 有效。
  从这个姿态重新求得 263 点路径，1339 个扫掠采样未发现静态碰撞。

现场证据支持“转弯局部跟踪停滞”，但不证明障碍评分中的某个单一权重是唯一原因。
诊断中曾临时将 MPPI critical_weight 40 改为 10 观察运动；之后已恢复为 40，配置文件未做对应修改。

## 确认的恢复缺陷与修复

旧的 FollowPath 上下文恢复仅清局部代价地图，然后重试同一条路径。
StableReplan 只检查几何有效性；跟踪已失败但仍几何有效的路径，会继续被保留。
因此反复发生“跟踪失败→清图→旧路径重试”，无法及时利用实际姿态重新接入可行路线。

在副本A的三个行为树中，在清局部代价地图后增加 ComputePathToPose：

| 行为树 | 恢复规划器 |
|---|---|
| fleet_navigation.xml | GridBased |
| task_navigation.xml | TaskGridBased |
| retreat_navigation.xml | RetreatGridBased |

新的行为是：可恢复的控制失败→清局部地图→从当前 TF 实际姿态重新规划同一目标→成功后再重试跟踪。
未提供 start，所以 Nav2 使用实际当前姿态；规划失败会清空输出路径并阻止本轮控制重试。
正常跟踪不触发额外规划，原有“有效路径不定时刷新”的稳定策略保留。

修改没有改变机器人尺寸、速度上限、Nav2 footprint、避障评分、通道令牌或碰撞停止区域。
原项目 `/home/ubuntu/huaxue304` 保持不变。

## 验证

- 导航/bringup 构建通过。
- 6 项相关原生测试通过，包括 3 项使用生产 XML 和真实 Nav2 RecoveryNode 的新增回归。
  覆盖正常路径不重算、控制失败后先重算再重试、规划失败不继续执行旧路径、不可恢复错误停止。
- 10 项相关 Python 检查通过。
- 实时运行中，omni 已完成 balanced_10_02 和 balanced_10_09；不把此结果解释为整批任务全部完成，
  也不能把运动恢复完全归因于临时参数试验。
- 热修改 XML 不会强制中断已加载的行为树；后续切换任务/返充树或重启导航时读取修复版本。
  当前仿真保持运行，未重置机器人位置、强制发底盘速度或强制释放通道令牌。

原生测试命令：

```bash
cd /home/ubuntu/huaxue304_A
source setup_A.bash
ctest --test-dir build/hetero_fleet_navigation --output-on-failure \
  -R 'controller_failure_replan|stable_route_behavior|goal_bound_route_behavior|reverse_path_heading_control'
```

Nav2 对规划失败清空路径及默认从实际位置规划的行为依据：
[ComputePathToPose 官方源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp)。
