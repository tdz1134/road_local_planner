# rlp_planner

规划层。实现"不同道路情况 × 不同定位情况 → 不同规划方法"的核心逻辑。

## 设计理念

### 为什么拆成多个规划方法？

传统做法是把所有逻辑塞进一个函数，用 if-else 分支处理各种情况。问题在于：

- 每种情况的策略差异大（沿路走 vs 朝终点搜索 vs 纯方向推进），混在一起难以理解和测试
- 新增或替换某种情况的策略时，容易影响其他分支
- 代价函数、安全校验等公共逻辑被重复编写

因此采用**策略模式**：每种道路情况对应一个独立的规划方法类（实现 `PlannerBase` 接口），由 `PlannerRouter` 根据当前情况路由选择。公共的代价评估和安全校验抽出来作为独立组件。

### 增量规划

全局终点永远在局部栅格地图之外，不可能一次算出全局路径。系统采用滚动时域规划：每个周期（默认 10Hz）重新规划一段前瞻距离内的局部路径，下一周期基于新的感知数据重新规划。这自然实现了增量式推进。

### 模式仲裁的滞回设计

定位质量在阈值附近波动时，如果直接用单阈值切换，会导致模式频繁跳变。采用双阈值滞回（`q_high=0.7` 进入 SEARCH，`q_low=0.5` 退回 FOLLOW）+ 最小驻留时间（`mode_dwell=2.0s`），确保切换稳定。

## 内容

### planner_base.h — 公共定义

- **`PlannerInput`** — 一个规划周期的全部输入（时间、边界、栅格、定位质量、终点、车速）
- **`PlanResult`** — 规划输出（路径、推荐速度、模式、边界状态、置信度、急停标志、方法名、原因）
- **`PlanningContext`** — 传给各规划方法的上下文（输入指针、走廊指针、安全边距、上一周期路径）
- **`PlannerBase`** — 规划方法抽象接口：`mode()`、`name()`、`candidates(ctx)`

### method_algorithm.h — 算法层

规划方法（方法层）与候选生成算法（算法层）两级分离：

- **`MethodAlgorithm`** — 候选生成算法接口：`name()` + `candidates(ctx)`。每个算法独立实现，便于单独替换与对比实验
- **`MethodPlannerBase`** — 带算法管理的方法基类：内部注册多个算法，运行时按参数（`PlannerParams::follow_alg / search_alg / free_alg`）按名字选择一个生效；未知名字回退到第一个注册的默认算法

分层关系：

```
PlannerRouter → PlannerBase 方法（follow/search/free，由道路+定位情况路由）
                    └── MethodPlannerBase（注册多个算法，参数切换）
                          └── MethodAlgorithm（offset / hybrid / fan / 未来的 A、B、C...）
```

### 三种规划方法

| 类 | 适用条件 | 默认算法 | 策略 |
|----|----------|----------|------|
| `FollowPlanner` | 有走廊 + 定位差 | `offset` | 走廊内横向偏移族采样，进度 = 沿中线弧长，完全忽略全局终点 |
| `SearchPlanner` | 有走廊 + 定位好 | `hybrid` | 走廊偏移族（路面优先）+ 朝终点扇形族，由代价函数权衡"沿路"与"抄近路" |
| `FreePlanner` | 无走廊 + 定位好 | `fan` | 纯终点方向扇形直线族，依赖栅格避障 |

### algorithms/ — 各方法的候选生成算法实现

| 算法类 | 名字 | 所属方法 | 说明 |
|--------|------|----------|------|
| `FollowOffsetAlg` | `offset` | follow | 走廊中线裁剪后按 `lateral_offsets` 横向偏移成族 |
| `SearchHybridAlg` | `hybrid` | search | 走廊偏移族 + 终点扇形族合并输出 |
| `FreeFanAlg` | `fan` | free | 终点方向扇形直线族 |

### planner_router — 方法路由

路由逻辑：

```
定位质量 → 模式（双阈值滞回 + 驻留时间）
模式 + 走廊有效性 → 具体规划方法
```

路由表：

| 走廊 | 模式 | 选中方法 |
|------|------|----------|
| 有 | FOLLOW | FollowPlanner |
| 有 | SEARCH | SearchPlanner |
| 无 | SEARCH + 终点可用 | FreePlanner |
| 无 | FOLLOW | nullptr → 停车保护 |
| 无 | SEARCH + 终点不可用 | nullptr → 停车保护 |

### candidate_gen — 候选生成原语

被不同规划方法共享的路径生成工具：

- **`corridorFamily`** — 走廊中线裁剪到前瞻距离后，按 `lateral_offsets` 横向偏移生成一组候选
- **`goalFan`** — 从原点朝终点方位展开扇形直线族（`n_goal_bearings` 条，半角 `goal_fan_deg`）
- **`lookaheadLength`** — 前瞻距离 = max(min_lookahead, speed × lookahead_time)

所有候选路径会在起点前拼接车辆原点 (0,0)，形成从当前位置出发的完整 Path。

### cost_evaluator — 代价评估

对所有候选路径统一评分，加权求和：

| 代价项 | 说明 |
|--------|------|
| 碰撞 | 硬代价：路径上任意点占据 → 直接返回 collision_cost |
| 越廊 | 软代价：路径点距走廊中线的距离超出 (半宽 - 安全边距) 的部分 |
| 平滑 | 相邻段航向变化量的平方和 |
| 进度 | FOLLOW 模式：沿走廊弧长（越长越好）；SEARCH 模式：朝终点方向投影 |
| 一致性 | 与上一周期路径的平均偏差，防止逐帧抖动 |

### safety_checker — 安全校验

- **制动包络**：`v²/(2a) + v·T_reaction + margin ≤ d_obs`，不满足则判定不可行
- **推荐速度**：取 `v_max`、障碍约束速度（制动方程正根）、曲率约束速度（`√(a_lat/k_max)`）三者最小值

### planner_core — 总调度

一个 `plan()` 调用完成完整规划周期：

```
1. RoadModel.update()          → 更新道路模型
2. Router.select()             → 选择规划方法
3. method.candidates()         → 生成候选路径
4. CostEvaluator.evaluate()    → 逐条评分，选最优
5. SafetyChecker.check()       → 校验制动包络，计算推荐速度
6. 输出 PlanResult             → 路径 + 速度 + 状态
```

内部维护跨周期状态：上一周期路径（用于一致性代价和 warm start）。

## 扩展：为某个方法新增候选生成算法

以给 follow 方法新增算法 "A" 为例，共 4 步：

1. **实现算法类**：新建 `include/rlp_planner/algorithms/follow_a.h` 与 `src/algorithms/follow_a.cpp`，继承 `MethodAlgorithm`，实现 `name()`（返回 `"A"`）与 `candidates(ctx)`（不适用时返回空）。可复用 `candidate_gen` 中的生成原语
2. **注册**：在 `FollowPlanner` 构造函数中 `addAlgorithm(std::make_unique<FollowAAlg>(p))`
3. **构建**：把新 `.cpp` 加入 `CMakeLists.txt` 的 `add_library` 列表
4. **启用**：`params.yaml` 中设 `follow_alg: A`（不配置或名字写错则回退默认算法；`~status` 话题会输出实际生效的算法名，便于确认）

方法层（路由、代价、安全校验）无需任何改动；新旧算法共存，改参数即可对比实验。

## 依赖

- `rlp_road`（→ `rlp_common`）
