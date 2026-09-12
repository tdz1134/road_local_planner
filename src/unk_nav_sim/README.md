# unk_nav_sim

unk_nav 规划核心的 **Gazebo 仿真环境 + ROS 接口层**。

本包 **不含任何算法**。所有规划与控制逻辑在 `unk_nav` 库中（纯 C++，零 ROS 依赖）。
本包只做三件事：
1. 把 Gazebo 传感器数据转成 `unk_nav` 的输入接口（`NavInput`）
2. 调用 `unk_nav` 的规划和控制器
3. 把输出（`TwistCmd`）转成 ROS 话题发布

三个节点通过标准 ROS 话题通信，松耦合、可独立替换。

---

## 快速启动

可直接复制的完整命令清单见仓库根目录 `快速启动.md`。

```bash
source ~/projects/road_local_planner/devel/setup.bash
roslaunch unk_nav_sim open_goal.launch
```

启动后在 RViz 里用 **2D Nav Goal** 工具点击目标点，Scout 开始自主导航避障。

可选参数：
```bash
roslaunch unk_nav_sim open_goal.launch world:=walls   # 换 30×30m 墙壁世界（等价于 walls_goal.launch）
roslaunch unk_nav_sim open_goal.launch gui:=false     # 无头模式
```

入口按「世界_模式」命名：`road` = 沿路模式（`nav_params_road.yaml`，自动跟路、不需点目标），
`goal` = 终点模式（`nav_params.yaml`，必须手动点 2D Nav Goal）。世界有 `open`（开阔地）、`walls`、
`road`（直走廊，沿路版叫 `road_follow.launch`）、`loop`（6m 闭合环）、`loop_extreme`（3m + S 弯
+ 180° 发夹）、`obstacle_mid` / `obstacle_tight`（闭合环 + 路上静态障碍）。

### 沿路模式（无定位，走廊即道路）

不点目标、不依赖定位，Scout 沿两侧路缘夹出的走廊自动前行、绕路中障碍、跟弯：

```bash
roslaunch unk_nav_sim road_follow.launch
```

- 世界 `road_world.world`：直走廊 + 两个路中圆柱 + 90° 弯。
- 导航配置 `unk_nav/config/nav_params_road.yaml`（`follow_road=true`）：NavCore 从栅格
  走廊几何直接推车体系前瞻子目标，**完全不读定位/全局终点**。
- **默认不启动 localization_node**（体现“无定位”）：规划只用 base 系栅格，路径以 base_link 系
  发布。RViz 与终点模式共用一份 `unk_nav.rviz`（Fixed Frame=odom），沿路模式需**手动改成
  base_link**才能正确看到路径。
- 可选 `use_localization:=true`：提供 body 系车速（本体感知）与 odom 真值可视化；沿路规划仍不读 pose。

```bash
roslaunch unk_nav_sim road_follow.launch use_localization:=true   # 供车速 + 真值可视化
roslaunch unk_nav_sim road_follow.launch gui:=false               # 无头
```

---

## 架构

```
┌─────────────────────────────────────────────────────────────────┐
│                         Gazebo 仿真                              │
│  /gazebo/model_states        /velodyne_points                   │
└────────┬──────────────────────────────┬─────────────────────────┘
         │                              │
         ▼                              ▼
┌──────────────────┐          ┌──────────────────┐
│ localization_node│          │    grid_node     │   ← unk_nav_sim（ROS 接口）
│                  │          │                  │
│ 提取 Scout 位姿  │          │ 3D点云→2D栅格    │
│ 发布 /odom + TF  │          │ Z切片 + 射线追踪  │
└────────┬─────────┘          └────────┬─────────┘
         │ /odom                       │ /local_grid
         │                             │
         ▼                             ▼
┌─────────────────────────────────────────────────┐
│                   nav_node                       │   ← unk_nav_sim（ROS 接口）
│                                                 │
│  ROS 话题 → unk::NavInput                       │
│         → unk::NavCore::plan()      ← unk_nav   │
│         → unk::PurePursuitController ← unk_nav  │
│  unk::TwistCmd → geometry_msgs::Twist           │
└────────┬──────────────────────────┬─────────────┘
         │ /cmd_vel                 │ /unk_nav/path
         ▼                          ▼
┌──────────────────┐      ┌──────────────────┐
│   Scout v2 底盘   │      │   RViz 可视化     │
└──────────────────┘      └──────────────────┘
```

### unk_nav_sim 与 unk_nav 的接口边界

| 层 | 包 | 内容 | 依赖 |
|----|-----|------|------|
| **算法层** | `unk_nav` | NavCore（规划）、PurePursuitController（控制）、`config/nav_params.yaml`（参数事实源） | 纯 C++14 + yaml-cpp（仅 `params_io` 一处），零 ROS |
| **接口层** | `unk_nav_sim` | 3 个 ROS 节点：话题订阅/发布、类型转换、参数加载 | roscpp, pcl_ros, tf2 |

`nav_node.cpp` 中 **唯一的算法调用**：
```cpp
// 规划
unk::NavResult result = core_->plan(nav_input);
// 控制
unk::TwistCmd tc = controller_->compute(result.path, result.recommended_speed);
// 转 ROS 类型
cmd.linear.x = tc.v;
cmd.angular.z = tc.w;
```

换算法 → 改 `unk_nav` 库；换仿真环境 → 改 `unk_nav_sim` 节点。两边互不影响。

---

## 话题一览

启动任一 launch 后的完整话题列表：

### 传感器 / 仿真

| 话题 | 类型 | 说明 |
|------|------|------|
| `/gazebo/model_states` | gazebo_msgs/ModelStates | Gazebo 所有模型的真值位姿 |
| `/velodyne_points` | sensor_msgs/PointCloud2 | Velodyne HDL-32E 点云（32线，10Hz） |
| `/clock` | rosgraph_msgs/Clock | 仿真时间 |
| `/joint_states` | sensor_msgs/JointState | 轮关节状态 |

### 定位

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/odom` | nav_msgs/Odometry | localization_node 发布 | Scout 位姿 + 速度（Gazebo 真值） |
| `/tf` | tf2_msgs/TFMessage | localization_node 发布 | odom → base_link 变换 |
| `/tf_static` | tf2_msgs/TFMessage | robot_state_publisher 发布 | base_link → velodyne 等静态变换 |

### 感知

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/local_grid` | nav_msgs/OccupancyGrid | grid_node 发布 | base_link 系局部栅格（340×340，0.1m/格，窗口 34m） |

### 导航

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/move_base_simple/goal` | geometry_msgs/PoseStamped | RViz → nav_node | 用户点击的目标点（odom 系） |
| `/cmd_vel` | geometry_msgs/Twist | nav_node → Scout | 速度指令（linear.x + angular.z） |
| `/unk_nav/path` | nav_msgs/Path | nav_node 发布 | 当前规划路径（目标模式 odom 系，沿路模式 base_link 系） |
| `/unk_nav/state` | std_msgs/String | nav_node 发布 | 导航状态（GO/IDLE/ARRIVED/ABORT + 原因） |
| `/unk_nav/work_grid` | nav_msgs/OccupancyGrid | nav_node 发布 | 调试：膨胀后 A* 实际搜索的栅格（规划器眼中的世界） |
| `/unk_nav/fan_candidates` | visualization_msgs/Marker | nav_node 发布 | 调试：子目标扇形展开的候选 |
| `/unk_nav/goal_marker` | visualization_msgs/Marker | nav_node 发布 | 调试：当前车体系子目标（绿色圆柱） |

### 电机控制（底层）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/scout_motor_fl_controller/command` | std_msgs/Float64 | 前左轮速度 |
| `/scout_motor_fr_controller/command` | std_msgs/Float64 | 前右轮速度 |
| `/scout_motor_rl_controller/command` | std_msgs/Float64 | 后左轮速度 |
| `/scout_motor_rr_controller/command` | std_msgs/Float64 | 后右轮速度 |

---

## 三个节点详解

### 1. localization_node

**职责**：从 Gazebo 真值获取位姿，替代实车的定位系统。

| 项目 | 值 |
|------|-----|
| 订阅 | `/gazebo/model_states` |
| 发布 | `/odom`、`/tf`（odom → base_link）|
| 参数 | `model_name`（默认 `scout/`）|

**替换方式**：实车上换成 wheel_odom + EKF 或 UWB 定位，只要输出相同的 `/odom` + TF。

### 2. grid_node

**职责**：将 3D 点云投影为 2D 占据栅格。

| 项目 | 值 |
|------|-----|
| 订阅 | `/velodyne_points` |
| 发布 | `/local_grid` |
| 坐标系 | 输出栅格在 base_link 系 |

**算法**：
1. TF 变换 velodyne → base_link（Z 偏移 0.39m）
2. Z 切片过滤 [0.1, 1.5]m（排除地面点）
3. 距离裁剪 > 12m 的点
4. 射线追踪（Bresenham）：原点→命中点沿途标 free(0)
5. 命中点标 occupied(100)
6. 未扫到区域保持 unknown(-1)

**替换方式**：实车换 2D 激光雷达时，改订阅 `/scan`（LaserScan），去掉 Z 过滤。

### 3. nav_node

**职责**：ROS 接口壳，调用 unk_nav 库完成规划与控制。

| 项目 | 值 |
|------|-----|
| 订阅 | `/odom`、`/local_grid`、`/move_base_simple/goal` |
| 发布 | `/cmd_vel`、`/unk_nav/path`、`/unk_nav/state` |
| 频率 | 10 Hz（由 `plan_freq` 参数控制）|

**内部流程**（每周期）：
1. `nav_msgs/OccupancyGrid` → `unk::GridMap`（字段一一对应，直接拷贝）
2. 组装 `unk::NavInput`（pose + grid + goal + speed）
3. `unk::NavCore::plan()` → `unk::NavResult`（**算法在 unk_nav 库中**）
4. `unk::PurePursuitController::compute()` → `unk::TwistCmd`（**算法在 unk_nav 库中**）
5. `TwistCmd{v,w}` → `geometry_msgs::Twist` → 发布 `/cmd_vel`

**本文件不含算法**。换控制器改 `unk_nav/controller.h`；换规划器改 `unk_nav/nav_core.h`。

---

## 参数配置

导航算法参数（同一份 yaml 供离线 demo、Gazebo 仿真、实车/MDC 三方复用，由 `unk::loadNavParams()` 直接读取，**不走 ROS 参数服务器**）：

| 文件 | 场景 | 说明 |
|------|------|------|
| `unk_nav/config/nav_params.yaml` | 终点导航（默认） | 规划/控制全量参数，`follow_road:false` |
| `unk_nav/config/nav_params_road.yaml` | 沿路模式 | 启用 `follow_road:true`，调整膨胀与走廊前瞻参数 |

仿真层参数（rosparam → 仿真节点）：

| 文件 | 归属 / 加载方式 |
|------|----------|
| `unk_nav_sim/config/grid_params.yaml` | 仿真层，rosparam → grid_node |

高频调参入口（完整说明看 yaml 内注释）：

| 症状 | 首选参数 |
|------|----------|
| 撞障碍/贴边太近 | `inflation_radius`↑、`pursuit_lookahead`↓ |
| 窄通道过不去/老 ABORT | `inflation_radius`↓ |
| 不绕障碍直冲墙 | `lookahead_ratio`↑ |
| 路径抖动 | `pursuit_lookahead`↑、`smooth_laplacian_iters`↓ |
| 到点不停 | `goal_tolerance`↓ |

注：`/unk_nav/state` 话题里的 `subgoal_trunc` 表示子目标被膨胀区截断（终点方向有墙，
本周期只走到带子边缘），属正常行为；持续出现且不前进才需要调参。

---

## 文件结构

```
unk_nav_sim/
├── src/
│   ├── localization_node.cpp   ← 定位（Gazebo真值）
│   ├── grid_node.cpp           ← 感知（点云→栅格）
│   └── nav_node.cpp            ← ROS 接口壳（调用 unk_nav 库）
├── config/
│   ├── grid_params.yaml        ← 栅格参数（仿真层）
│   └── unk_nav.rviz            ← RViz 配置（所有 launch 共用；沿路模式需手动把 Fixed Frame 改成 base_link）
│   （导航参数在算法层 ../unk_nav/config/：nav_params.yaml 终点、nav_params_road.yaml 沿路）
├── launch/                     ← 命名规则见上文「快速启动」
│   ├── open_goal.launch        ← 开阔地，world:=walls 可切墙壁世界
│   ├── walls_goal.launch       ← 30×30m 墙壁世界
│   ├── road_follow.launch      ← 沿路模式（无定位）
│   ├── road_nav.launch         ← 同一走廊的终点模式
│   ├── loop_{road,goal}.launch           ← 6m 闭合环，无障碍
│   ├── loop_extreme_{road,goal}.launch   ← 3m 环 + S 弯 + 发夹
│   └── obstacle_{mid,tight}_{road,goal}.launch  ← 闭合环 + 路上静态障碍
├── worlds/
│   ├── large_world.world       ← 200×200m，24 个障碍（柱/块/L 形/墙）
│   ├── walls_world.world       ← 30×30m，纯墙壁
│   ├── road_world.world        ← 走廊道路（直路+路中两圆柱+90°弯）
│   ├── unk_world.world         ← 30×30m，混合障碍（无对应 launch，留作手工测试）
│   ├── loop_road.world         ← 6m 闭合环，纯道路      ┐ scripts/gen_road_world.py
│   ├── loop_extreme.world      ← 3m 极端闭合环          ┘
│   ├── obstacle_mid.world      ← 6m 环 + 19 个可绕障碍   ┐ scripts/gen_obstacle_world.py
│   └── obstacle_tight.world    ← 3.6m 环 + 含一个全封死断面 ┘
├── gazebo笔记.md               ← Gazebo 世界搭建教程
├── CMakeLists.txt
└── package.xml
```

---

## 依赖

| 包 | 用途 |
|----|------|
| `unk_nav` | 规划 + 控制算法库（纯 C++，同仓库） |
| `scout_gazebo_sim` | Scout v2 Gazebo 仿真 |
| `scout_description` | Scout v2 URDF/mesh |
| `velodyne_description` | Velodyne HDL-32E URDF |
| `velodyne_gazebo_plugins` | Velodyne Gazebo 传感器插件 |
| `pcl_ros` / `pcl_conversions` | 点云处理 |
| `tf2_ros` | 坐标变换 |
