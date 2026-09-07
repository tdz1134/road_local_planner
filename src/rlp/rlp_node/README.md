# rlp_node

ROS 节点层。负责与 ROS 生态的全部交互，不含任何规划算法逻辑。

## 设计理念

算法层（`rlp_planner`、`rlp_road`、`rlp_common`）全部是纯 C++ 实现，不依赖 ROS。这样做的好处：

- 可以脱离 ROS 进行离线单元测试（`core_test.cpp`）
- 算法层可移植到非 ROS 平台
- ROS 相关的消息转换、tf 查询、话题发布等集中在一个地方，便于维护

`rlp_node` 的职责边界很清晰：

1. **消息转换**：将 ROS 消息（OccupancyGrid、Boundary、LocalizationQuality 等）转为算法层的数据结构
2. **坐标变换**：通过 tf 将全局终点转换到车体系；tf 不可用时标记 `goal_valid=false`
3. **定时调度**：按 `plan_freq`（默认 10Hz）定时调用 `PlannerCore::plan()`
4. **参数加载**：从 `params.yaml` 加载参数到 `PlannerParams`
5. **结果发布**：将 PlanResult 转为 ROS 消息发布

## 内容

### planner_node.h/.cpp

节点主体类 `PlannerNode`：

- 构造函数中完成参数加载、PlannerCore 创建、tf 监听器创建、话题订阅/发布、定时器创建
- 输入回调只缓存最新一帧（加锁），不做任何计算
- `onTimer()` 中组装 `PlannerInput`、调用 `plan()`、发布结果
- 速度控制 v1：对路径上约 2m 预瞄点做纯跟踪，输出 `linear.x` + `angular.z`

### main.cpp

入口点，创建 ROS 节点并启动。

### msg/

自定义消息定义：

- **`Boundary.msg`** — 道路单侧边界：header + Point32 点列 + confidence
- **`LocalizationQuality.msg`** — 定位质量：header + quality [0,1]

### config/params.yaml

全部可调参数，与 `rlp::PlannerParams` 字段一一对应。分为：

- 车辆能力/安全（v_max、a_decel_max、a_lat_max 等）
- 规划参数（频率、采样间距、前瞻时间等）
- 边界/走廊（超时、记忆时长、膨胀率等）
- 模式仲裁（滞回阈值、驻留时间）
- 代价权重（越廊、平滑、进度、一致性）

### launch/planner.launch

启动文件，加载参数并运行节点。话题可在 launch 中 remap。

### test/core_test.cpp

离线单测，不依赖 ROS 运行环境，直接驱动 `PlannerCore`。8 个测试用例：

| # | 场景 | 预期 |
|---|------|------|
| 1 | 双侧有效 + 定位差 | FollowPlanner 输出可行路径 |
| 2 | 右边界超时 → 仅左边界 | LEFT_ONLY，走廊由路宽记忆补全 |
| 3 | 双侧短时缺失 | MISSING_SHORT，沿用旧走廊，置信度下降 |
| 4 | 双侧超时 + 定位差 | 无可用方法，急停保护 |
| 5 | 双侧超时 + 定位好 + 终点可用 | FreePlanner 接管 |
| 6 | 定位质量从高到低 | SEARCH → FOLLOW 滞回切换 |
| 7 | 横贯走廊的障碍墙 | 全部候选碰撞，急停 |

## 依赖

- `rlp_planner`（→ `rlp_road` → `rlp_common`）
- ROS：`roscpp`、`std_msgs`、`geometry_msgs`、`nav_msgs`、`tf2`、`tf2_ros`、`tf2_geometry_msgs`
- 消息生成：`message_generation`（编译期）/ `message_runtime`（运行期）

## 输入话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `~local_map` | `nav_msgs/OccupancyGrid` | 局部占据栅格（车体系） |
| `~boundaries/left` | `rlp_node/Boundary` | 左边界点列 |
| `~boundaries/right` | `rlp_node/Boundary` | 右边界点列 |
| `~localization/quality` | `rlp_node/LocalizationQuality` | 定位质量 [0,1] |
| `~global_goal` | `geometry_msgs/PoseStamped` | 全局终点（全局系，经 tf 转到车体系） |
| `~odom` | `nav_msgs/Odometry` | 当前车速 |

## 输出话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `~plan` | `nav_msgs/Path` | 局部路径（车体系） |
| `~speed_cmd` | `geometry_msgs/Twist` | 推荐速度（linear.x）+ 角速度（angular.z） |
| `~status` | `std_msgs/String` | 状态调试信息（method/mode/boundary/conf/v_rec/estop/reason） |

## 运行

```bash
# 编译
catkin_make

# 运行节点
source devel/setup.bash
roslaunch rlp_node planner.launch

# 运行离线测试
./devel/lib/rlp_node/core_test
```
