---
name: road-local-planner
description: 差速底盘沿路路径规划系统（rlp_*）架构与设计规范，含 unk_nav（无定位/未知环境导航）边界。涉及路径规划、边界管理、走廊构建、模式路由、代价评估，或需区分 rlp_* 与 unk_nav 两套独立系统时参考此 skill。
---

# 沿路路径规划系统

## 系统概述

差速底盘（30km/h）的局部路径规划系统，输入为局部占据栅格、道路边界、全局终点、定位质量。采用分层架构，根据道路/定位情况自动选择规划方法。

## 与 unk_nav 的边界（先认清在改哪套）

工作区里有**两套完全独立**的局部规划系统，不复用任何代码，别混用概念：

| | `rlp_*`（本 skill） | `unk_nav` / `unk_nav_sim` |
|---|---|---|
| 场景 | **有道路先验**的沿路规划 | **无道路先验**的未知环境局部反应式导航 |
| 输入 | 栅格 + 左右边界 + 定位质量 + 终点 | 栅格 + 位姿 + 车速 + 终点（无边界/走廊概念） |
| 主流程 | 边界状态机 → 走廊 → 模式路由 → 候选/搜索 → 代价 | 10Hz 重规划：子目标选取（扇形打分+方向迟滞）→ 链式前瞻跳点 → Catmull-Rom 样条（或直线）→ 限速（limit_by 诊断）→ 行为 FSM；50Hz PurePursuit+斜率限幅出 /cmd_vel（A\*/平滑保留为库，主链路已不调用） |
| 配置 | `rlp_node/config/params.yaml`，rosparam 加载 | `unk_nav/config/nav_params.yaml`（归算法层），经 `params_io::loadNavParams` 读，**不走 rosparam** |
| 依赖 | 无 ROS 内核 + `rlp_node` ROS 壳 | 纯 C++14 + yaml-cpp（仅 `params_io` 一处），核心零 ROS；`unk_nav_sim` 为 ROS/Gazebo 胶水层 |

改 `unk_nav` 前先看 `src/unk_nav/README.md`（本 skill 不展开其细节）。现状要点：

- **加参数三处同步**：`types.h::NavParams` 字段 + `params_io.cpp` 绑定表 + yaml 一行（沿路有两份 yaml：`nav_params_road.yaml` / `nav_params_road_speed.yaml`，键集必须一致）；未知 key / 类型错直接拒绝启动。**params_io 还有跨字段取值约束（如 `footprint_clear_radius < inflation_radius`），任一违反 FATAL 拒启动；core_test 不加载真实 yaml，单测全绿 ≠ 配置可用，改完 yaml 必须人工复跑全部约束**。
- **能力边界**：凸障碍可绕；凹槽深于前瞻（`lookahead_ratio × perception_range`）落入局部极小 → RECOVERY → ABORT（绕行/脱困排除在范围外）。沿路走廊封死场景仍有 1 条已知 FSM 回归失败（单测基线 329 过 / 1 已知失败）。
- **两种模式**：终点模式 `subgoal::project` 单射线投影、被截断才展扇形候选（车头朝向居中，打分 = 终点对齐 + 饱和自由距离 + `subgoal_prev_w` 迟滞），另有 `stop_horizon` 停车视距限速、`goal_clear_radius` 终点清洞默认关；沿路模式（`follow_road=true`）无全局定位，`road_follow::lookAheadChain` 链式前瞻（RLA）逐跳 `scanFan`（打分 = free + 对齐 cosθ + 可选 goal_align + `road_prev_w` hop-1 方向迟滞），跳点串成 Catmull-Rom 弦长参数化样条（`curve_fit_enable=false` 退回直线）。
- **看多深与速度耦合**：基准 `road_lookahead_ratio × perception_range`，`L(v) = 基准 + lookahead_speed_k·v`（上限 perception_range；k=0 零回归）；“走多近”由 `chain_hops × step_walk` 定，与“看多深”解耦。
- **无 /odom 环境事实**：`scout_skid_steer_controller` 不发布 /odom，沿路模式 `localization_node` 也是关的 → nav_node 用最近下发指令速度估计 `current_speed`（`speed_valid` 仅真有 odom 时为真），改速度相关逻辑时别假设有真 odom。
- **诊断与工具**：`/unk_nav/state`（含 `limit_by=`、curve 标记）、`/unk_nav/chain` 扇形可视化（蓝线长 = min(L, 到墙自由距离)，附 v/L 文本）；丝滑度分析用 `scripts/plot_cmd_vel.py 120`（判读基准：换向 <5 次/分且无 <0.3s 微换向 = 丝滑）；高速环路场景 `loop_speedloop_road.launch`。

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
                          └── 算法层 MethodAlgorithm（offset / hybrid / astar / rrt / fan / 新增算法）
```

| 情况 | 方法 | 算法（`*_alg` 可选，标★为默认） | 模式 | 策略 |
|------|------|-----------------------------------|------|------|
| 有路 + 定位差 | `FollowPlanner` | ★`offset` | 采样 | 走廊内横向偏移族，忽略终点 |
| 有路 + 定位好 | `SearchPlanner` | ★`hybrid` / `astar` / `rrt` | 采样→搜索 | `hybrid`：走廊偏移族 + 终点扇形族代价权衡；`astar`/`rrt`：栅格直接搜索，走廊作软约束 |
| 无路 + 定位好 | `FreePlanner` | ★`fan` / `astar` / `rrt` | 采样→搜索 | `fan`：纯终点方向扇形直线；`astar`/`rrt`：无走廊时自动退化为纯栅格搜索 |

> **搜索式算法 `astar` / `rrt` 同时注册在 `SearchPlanner` 与 `FreePlanner`**：有走廊时把走廊当软约束，无走廊时跳过走廊约束退化为纯栅格搜索——同一份搜索代码覆盖“有路绕行”与“无路朝终点”两种场景。参数写错时回退到该方法第一个注册的默认算法。

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
