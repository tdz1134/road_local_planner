---
name: road-local-planner
description: 差速底盘沿路路径规划系统架构与设计规范。当涉及路径规划、边界管理、走廊构建、模式路由、代价评估时参考此 skill。适用于理解系统分层、修改或新增候选生成算法、调试边界状态机或扩展新规划方法。
---

# 沿路路径规划系统

## 系统概述

差速底盘（30km/h）的局部路径规划系统，输入为局部占据栅格、道路边界、全局终点、定位质量。采用分层架构，根据道路/定位情况自动选择规划方法。

## 四层架构

```
rlp_common ← rlp_road ← rlp_planner ← rlp_node
```

| 层 | 包名 | 职责 | ROS 依赖 |
|----|------|------|----------|
| 基础 | `rlp_common` | 类型定义（Point2D/GridMap/Path/PlannerParams）+ 几何工具 | 无 |
| 道路 | `rlp_road` | 边界管理 + 走廊构建 | 无 |
| 规划 | `rlp_planner` | 三种规划方法 + 路由 + 代价 + 安全 | 无 |
| 节点 | `rlp_node` | ROS 消息转换 + tf + 定时调度 + 自定义消息 | 是 |

## 核心设计原则

### 1. 策略模式分离规划方法（方法层 + 算法层两级分离）

不同情况用不同规划方法（继承 `MethodPlannerBase`，实现 `PlannerBase` 接口），避免 if-else 大杂烩。每个方法内部又可注册多个候选生成算法（实现 `MethodAlgorithm` 接口），运行时按参数（`follow_alg` / `search_alg` / `free_alg`）按名字切换，便于不同算法对比实验；未知名字回退到第一个注册的默认算法。

算法支持两种工作模式：
- **采样式**：实现 `candidates()` 返回路径族，由 `CostEvaluator` 统一评价选最优。
- **搜索式**：实现 `directPlan()` 直接返回最终路径（如 A\*/RRT），跳过 `CostEvaluator`。默认 `directPlan()` 返回空路径 → 回退到采样式流程。

```
PlannerRouter → 方法层（follow/search/free，由道路+定位情况路由）
                    └── MethodPlannerBase（注册多个算法，参数切换）
                          └── 算法层 MethodAlgorithm（offset / hybrid / astar / fan / 新增算法）
```

| 情况 | 方法 | 默认算法 | 模式 | 策略 |
|------|------|----------|------|------|
| 有路 + 定位差 | `FollowPlanner` | `offset` | 采样 | 走廊内横向偏移族，忽略终点 |
| 有路 + 定位好 | `SearchPlanner` | `hybrid` | 采样 | 走廊偏移族 + 终点扇形族，代价权衡 |
| 有路 + 定位好 | `SearchPlanner` | `astar`（可选） | **搜索** | 栅格上 A\* 直接搜索，走廊作为软约束 |
| 无路 + 定位好 | `FreePlanner` | `fan` | 采样 | 纯终点方向扇形直线 |

算法实现位于 `rlp_planner/src/algorithms/`，可复用 `candidate_gen` 中的生成原语（`corridorFamily` / `goalFan` / `lookaheadLength`）。`PlanResult.algorithm` 与 `~status` 话题会输出实际生效的算法名。

### 2. 边界状态机（每侧独立超时）

```
BOTH → LEFT_ONLY/RIGHT_ONLY → MISSING_SHORT → MISSING_TIMEOUT
```

- `BOTH`：双侧有效，置信度 1.0
- `*_ONLY`：单侧超时，用历史路宽补全，置信度 0.6
- `MISSING_SHORT`：双侧超时但在记忆期内，沿用旧走廊+膨胀半宽，置信度指数衰减
- `MISSING_TIMEOUT`：超过记忆期，走廊不可信

### 3. 模式仲裁滞回设计

定位质量在阈值附近波动时，双阈值滞回 + 驻留时间防止模式跳变：

- `q_high=0.7`：进入 SEARCH 模式
- `q_low=0.5`：退回 FOLLOW 模式
- `mode_dwell=2.0s`：最小驻留时间

### 4. 增量规划

全局终点永远在局部栅格外，采用滚动时域：每周期（10Hz）重新规划前瞻距离内的局部路径。

## 关键组件

### RoadModel（rlp_road 门面类）

聚合 `BoundaryManager` + `CorridorBuilder`，对外只暴露 `update()` + `state()` + `corridor()`。

### PlannerRouter

路由表：

| 走廊 | 模式 | 选中方法 |
|------|------|----------|
| 有 | FOLLOW | FollowPlanner |
| 有 | SEARCH | SearchPlanner |
| 无 | SEARCH + 终点可用 | FreePlanner |
| 无 | FOLLOW | nullptr → 停车保护 |

### CostEvaluator

加权求和：

| 代价项 | 说明 |
|--------|------|
| 碰撞 | 硬代价：路径上任意点占据 → collision_cost |
| 越廊 | 软代价：超出 (半宽 - 安全边距) 的部分 |
| 平滑 | 相邻段航向变化量平方和 |
| 进度 | FOLLOW：沿走廊弧长；SEARCH：朝终点投影 |
| 一致性 | 与上一周期路径偏差 |

### SafetyChecker

- 制动包络：`v²/(2a) + v·T_reaction + margin ≤ d_obs`
- 推荐速度：取 v_max、障碍约束、曲率约束三者最小

## 构建与测试

```bash
# 编译
catkin_make

# 运行节点
source devel/setup.bash
roslaunch rlp_node planner.launch

# 离线单测（不依赖 ROS）
./devel/lib/rlp_node/core_test
```

## 话题接口

### 输入

| 话题 | 类型 | 说明 |
|------|------|------|
| `~local_map` | `nav_msgs/OccupancyGrid` | 局部栅格（车体系） |
| `~boundaries/left` | `rlp_node/Boundary` | 左边界 |
| `~boundaries/right` | `rlp_node/Boundary` | 右边界 |
| `~localization/quality` | `rlp_node/LocalizationQuality` | 定位质量 [0,1] |
| `~global_goal` | `geometry_msgs/PoseStamped` | 全局终点 |
| `~odom` | `nav_msgs/Odometry` | 车速 |

### 输出

| 话题 | 类型 | 说明 |
|------|------|------|
| `~plan` | `nav_msgs/Path` | 局部路径 |
| `~speed_cmd` | `geometry_msgs/Twist` | 推荐速度 + 角速度 |
| `~status` | `std_msgs/String` | 状态调试信息 |

## 扩展指南

### 为已有方法新增候选生成算法（最常用）

以给 follow 方法新增算法 "A" 为例，共 4 步：

1. **实现算法类**：新建 `include/rlp_planner/algorithms/follow_a.h` 与 `src/algorithms/follow_a.cpp`，继承 `MethodAlgorithm`，实现 `name()`（返回 `"A"`）与 `candidates(ctx)`（不适用时返回空，不抛异常）。可参照 `follow_offset.cpp`，复用 `candidate_gen` 原语
2. **注册**：在 `FollowPlanner` 构造函数中 `addAlgorithm(std::make_unique<FollowAAlg>(p))`
3. **构建**：把新 `.cpp` 加入 `rlp_planner/CMakeLists.txt` 的 `add_library` 列表
4. **启用**：`params.yaml` 设 `follow_alg: A`；`~status` 话题的 `alg=` 字段可确认实际生效算法

方法层（路由、代价、安全校验）无需改动；新旧算法共存，改参数即可对比实验。

### 新增规划方法（新的道路/定位情况）

1. 在 `rlp_planner/include/rlp_planner/` 创建 `xxx_planner.h`，继承 `MethodPlannerBase`
2. 实现 `mode()`、`name()`、`algorithmName()`（返回 `PlannerParams` 中对应参数字段），构造函数中注册至少一个算法
3. 在 `PlannerRouter::select()` 添加路由条件
4. 在 `PlannerCore` 构造函数中注册

### 调整代价权重

修改 `rlp_node/config/params.yaml` 中的 `w_offroad`、`w_smooth`、`w_progress`、`w_consistency`。

### 修改边界超时策略

调整 `boundary_timeout`、`corridor_hold_max`、`corridor_inflate_rate`。
