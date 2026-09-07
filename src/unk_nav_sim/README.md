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

```bash
source ~/projects/road_local_planner/devel/setup.bash
roslaunch unk_nav_sim unk_nav_test.launch
```

启动后在 RViz 里用 **2D Nav Goal** 工具点击目标点，Scout 开始自主导航避障。

可选参数：
```bash
roslaunch unk_nav_sim unk_nav_test.launch world:=walls   # 30×30m 墙壁世界
roslaunch unk_nav_sim unk_nav_test.launch gui:=false     # 无头模式
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

启动 `unk_nav_test.launch` 后的完整话题列表：

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
| `/local_grid` | nav_msgs/OccupancyGrid | grid_node 发布 | base_link 系局部栅格（408×408，0.05m/格） |

### 导航

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/move_base_simple/goal` | geometry_msgs/PoseStamped | RViz → nav_node | 用户点击的目标点（odom 系） |
| `/cmd_vel` | geometry_msgs/Twist | nav_node → Scout | 速度指令（linear.x + angular.z） |
| `/unk_nav/path` | nav_msgs/Path | nav_node 发布 | 当前规划路径（odom 系，RViz 可视化） |
| `/unk_nav/state` | std_msgs/String | nav_node 发布 | 导航状态（GO/IDLE/ARRIVED/ABORT + 原因） |

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

**两个 yaml 是参数的唯一事实源**，已覆盖算法层（`NavParams` 每个字段）与仿真层的全部
旋钮，每个参数带「作用 + 调大/调小的后果」注释，文件顶部还有「症状 → 该动哪个参数」
速查表。调试时直接改 yaml、重启 launch 即生效，本 README 不再重复参数表（避免两处
维护漂移）：

| 文件 | 作什么 | 归属 / 加载方式 |
|------|--------|----------|
| `unk_nav/config/nav_params.yaml` | 规划/控制全量参数（车辆能力、膨胀、滚动时域、A*、限速、平滑、卡死） | **算法层，与 ROS 解耦**；launch 用 `config_file` 私有参数把路径传给 nav_node，由 `unk::loadNavParams()` 直接读取（**不走 ROS 参数服务器**），同一份配置 demo/实车复用 |
| `unk_nav_sim/config/grid_params.yaml` | 点云→栅格（分辨率、窗口、Z 切片、量程） | 仿真层，rosparam → grid_node |

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
│   └── unk_nav.rviz            ← RViz 配置
│   （导航参数已移到算法层 ../unk_nav/config/nav_params.yaml）
├── launch/
│   ├── unk_nav_test.launch     ← 全链路一键启动
│   ├── large_world.launch      ← 只启动 200×200m 世界
│   └── walls_world.launch      ← 只启动 30×30m 墙壁世界
├── worlds/
│   ├── large_world.world       ← 200×200m，20个障碍
│   ├── walls_world.world       ← 30×30m，纯墙壁
│   └── unk_world.world         ← 30×30m，混合障碍
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
