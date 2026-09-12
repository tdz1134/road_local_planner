# Gazebo 世界搭建笔记

> 对应文件：`worlds/unk_world.world`  
> 机器人：Scout v2（Velodyne HDL-32E，32线，100m量程）

---

## 一、SDF 世界文件结构

Gazebo 的世界文件用 **SDF**（Simulation Description Format）格式，后缀 `.world`。
骨架如下：

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="unk_world">

    <physics type="ode">          <!-- 物理引擎参数 -->
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <include>                      <!-- 内置光照 -->
      <uri>model://sun</uri>
    </include>

    <model name="ground_plane">    <!-- 地面 -->
      <static>true</static>
      ...
    </model>

    <model name="wall_1">          <!-- 障碍物（静态） -->
      <static>true</static>
      ...
    </model>

  </world>
</sdf>
```

**关键原则**：  
- 每个障碍物是一个独立的 `<model>`  
- `<static>true</static>` = 不受重力/碰撞影响，不会动  
- `<link>` 里必须同时有 `<collision>` 和 `<visual>`（见第三节）

---

## 二、坐标系与 pose

### 右手坐标系

```
         Z（上）
          |
          |
          +---- Y（左）
         /
        /
       X（前）
```

### pose 格式

```xml
<pose>x y z roll pitch yaw</pose>
```

| 参数 | 单位 | 含义 |
|------|------|------|
| x | m | 东西方向，+X = 东 |
| y | m | 南北方向，+Y = 北 |
| z | m | 高度，物体**几何中心**的 Z 坐标 |
| roll | rad | 绕 X 轴旋转 |
| pitch | rad | 绕 Y 轴旋转 |
| yaw | rad | 绕 Z 轴旋转（平面朝向）|

> **注意**：`z` 是物体**中心**的高度，不是底面！  
> 高 1.5m 的柱子放在地面上：`z = 1.5 / 2 = 0.75`

---

## 三、基本几何体

### 长方体（box）

```xml
<geometry>
  <box>
    <size>长(X) 宽(Y) 高(Z)</size>
  </box>
</geometry>
```

**示例**：一面东西走向的墙（长 6m，厚 0.2m，高 1.5m）：
```xml
<pose>7 6 0.75 0 0 0</pose>
<geometry><box><size>6.0 0.2 1.5</size></box></geometry>
```

**示例**：一面南北走向的墙（长 5m，厚 0.2m，高 1.5m）：
```xml
<pose>11 -4 0.75 0 0 0</pose>
<geometry><box><size>0.2 5.0 1.5</size></box></geometry>
```

### 圆柱体（cylinder）

```xml
<geometry>
  <cylinder>
    <radius>半径</radius>
    <length>高度</length>   <!-- 注意是 length 不是 height -->
  </cylinder>
</geometry>
```

**示例**：半径 0.3m，高 1.5m 的柱子：
```xml
<pose>4 2 0.75 0 0 0</pose>
<geometry><cylinder><radius>0.3</radius><length>1.5</length></cylinder></geometry>
```

### 地面（plane）

```xml
<geometry>
  <plane>
    <normal>0 0 1</normal>   <!-- 法线朝上 -->
    <size>100 100</size>     <!-- 渲染大小，不影响物理 -->
  </plane>
</geometry>
```

---

## 四、collision 与 visual 的区别

```xml
<link name="link">
  <!-- 物理引擎用的（碰撞检测、激光雷达射线检测） -->
  <collision name="collision">
    <geometry><box><size>1 1 1</size></box></geometry>
  </collision>

  <!-- 渲染引擎用的（你在 Gazebo 窗口看到的） -->
  <visual name="visual">
    <geometry><box><size>1 1 1</size></box></geometry>
    <material>...</material>
  </visual>
</link>
```

| 情况 | 结果 |
|------|------|
| 有 collision，无 visual | 物体存在但看不见（隐形障碍！） |
| 无 collision，有 visual | 看得见但穿过去（幽灵墙） |
| **两者都有** | **正常，激光雷达检测到碰撞体** |

> **激光雷达检测的是 `<collision>`，不是 `<visual>`！**  
> 两者尺寸必须一致，否则仿真会出错。

---

## 五、材质颜色

```xml
<material>
  <script>
    <uri>file://media/materials/scripts/gazebo.material</uri>
    <name>Gazebo/Red</name>
  </script>
</material>
```

常用预设颜色：

| 名称 | 颜色 |
|------|------|
| `Gazebo/Red` | 红 |
| `Gazebo/Orange` | 橙 |
| `Gazebo/Yellow` | 黄 |
| `Gazebo/Green` | 绿 |
| `Gazebo/Blue` | 蓝 |
| `Gazebo/Grey` | 灰（围墙用这个）|
| `Gazebo/White` | 白 |
| `Gazebo/Black` | 黑 |
| `Gazebo/Wood` | 木纹 |
| `Gazebo/Bricks` | 砖墙 |

---

## 六、unk_world 障碍物布局

```
   北 (Y=+15)
   ┌──────────────────────────────────────────┐
   │                                          │
   │  P3(-3,10)          P4(9,10)             │
   │                                          │
   │         ══════════ wall_horiz(7,6)        │
   │                                          │
   │  BOX(-6,4)     P1(4,2)                   │
   │                                          │
   │          ● 出生点(0,0)                    │
   │                                          │
   │         ┌──┐                             │
   │         │  │ L_shape(3,-7)               │
   │  P2(-8,-6)                               │
   │                   ║ wall_vert(11,-4)      │
   │                   ║                      │
   │              P5(0,-12)                   │
   │                                          │
   └──────────────────────────────────────────┘
   南 (Y=-15)
```

| 名称 | 位置 | 类型 | 尺寸 | 测试目的 |
|------|------|------|------|---------|
| `pillar_1` | (4, 2) | 圆柱 | r=0.3m h=1.5m | 近距离绕行 |
| `wall_horiz` | (7, 6) | 长方体 | 6×0.2×1.5m | 横向 LOS 遮挡 |
| `box_1` | (-6, 4) | 长方体 | 1.5³m | 方形障碍 |
| `l_shape` | (3, -7) | L形 | 4+4m臂 | 凸形拐角 |
| `pillar_2` | (-8, -6) | 圆柱 | r=0.4m h=1.5m | 西南区域 |
| `wall_vert` | (11, -4) | 长方体 | 0.2×5×1.5m | 纵向 LOS 遮挡 |
| `pillar_3` | (-3, 10) | 圆柱 | r=0.3m h=1.5m | 北部区域 |
| `pillar_4` | (9, 10) | 圆柱 | r=0.25m h=1.5m | 东北角 |
| `pillar_5` | (0, -12) | 圆柱 | r=0.35m h=1.5m | 远距离感知 |

> **设计原则**：所有障碍均为凸形，unk_nav 可绕行。  
> 禁止 U 形凹坑（unk_nav 已取消 BOUNDARY_FOLLOW，遇凹形会 ABORT）。

---

## 七、Launch 文件

```xml
<!-- 启动 Gazebo + 加载世界 -->
<include file="$(find gazebo_ros)/launch/empty_world.launch">
  <arg name="world_name" value="$(find unk_nav_sim)/worlds/unk_world.world"/>
  <arg name="paused"     value="false"/>
  <arg name="use_sim_time" value="true"/>
  <arg name="gui"        value="true"/>
</include>

<!-- 生成 Scout v2 -->
<include file="$(find scout_gazebo_sim)/launch/spawn_scout_v2.launch">
  <arg name="x" value="0"/>
  <arg name="y" value="0"/>
</include>

<!-- RViz -->
<node name="rviz" pkg="rviz" type="rviz"
      args="-d $(find unk_nav_sim)/config/unk_nav.rviz"/>
```

**启动命令**（上面这份 `unk_nav.launch` 已拆成按「世界_模式」命名的多个文件）：
```bash
source ~/projects/road_local_planner/devel/setup.bash
roslaunch unk_nav_sim open_goal.launch     # 全链路：Gazebo + Scout + 定位 + 栅格 + 导航 + RViz
roslaunch unk_nav_sim open_goal.launch x:=-10 y:=-10 yaw:=1.57   # 改变出生位置
roslaunch unk_nav_sim open_goal.launch world:=walls              # 换墙壁世界
```

只要 Gazebo 世界、不起车与导航（改地图、放障碍时用）：
```bash
roslaunch gazebo_ros empty_world.launch \
  world_name:=$(rospack find unk_nav_sim)/worlds/unk_world.world
```

---

## 八、调试技巧

### 8.1 检查话题是否存在
```bash
rostopic list | grep velodyne
# 应该看到：/velodyne_points
```

### 8.2 检查点云频率
```bash
rostopic hz /velodyne_points
# 期望：~10 Hz
```

### 8.3 查看雷达参数（量程/线数）
```bash
rostopic echo /velodyne_points -n 1 --noarr | grep -E "height|width|point_step"
# width=440（水平采样数），height=32（线数）
```

### 8.4 键盘遥控 Scout
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
# 或直接发速度指令：
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.5}" -r 10
```

### 8.5 检查机器人 TF 树
```bash
rosrun tf view_frames
# 生成 frames.pdf，查看 base_link → velodyne 等变换
```

---

## 九、Scout v2 关键参数

### 机器人本体

| 参数 | 值 |
|------|-----|
| 长 × 宽 × 高 | 0.925 × 0.38 × 0.21 m |
| 质量 | 200 kg |
| 轮距（track） | 0.583 m |
| 轴距（wheelbase）| 0.498 m |
| 轮半径 | 0.165 m |
| 速度指令话题 | `/cmd_vel`（geometry_msgs/Twist）|

### Velodyne HDL-32E 雷达

| 参数 | 值 |
|------|-----|
| 线数 | 32 |
| 水平 FOV | 360° |
| 垂直 FOV | -30.67° ～ +10.67° |
| 量程 | 100 m（unk_nav 可裁剪到 12m）|
| 话题 | `/velodyne_points`（sensor_msgs/PointCloud2）|
| 频率 | 10 Hz |
| 水平采样数 | 440（约 0.82°/点）|
| 安装高度 | 0.3 m（base_link 上方）|
| 驱动插件 | `libgazebo_ros_velodyne_laser.so` |

### 雷达检测高度计算

Velodyne 安装在 base_link 上方 0.3m，垂直 FOV 覆盖：

| 俯角 | 检测距离（地面目标）|
|------|----------------|
| -30.67° | 0.3/tan(30.67°) ≈ **0.5 m** |
| -10° | 0.3/tan(10°) ≈ **1.7 m** |
| 0°（水平）| **无穷远**（只看到水平面以上）|
| +10.67° | 水平面以上，看不到地面 |

> 结论：地面在 0.5m 外都能被检测到，1.5m 高的障碍物在 0.3m 外就能被最低线检测到。

---

## 十、unk_nav 参数换算（sensor_range=12m）

unk_nav 的所有规划距离按 `sensor_range` 无量纲化：

| 参数 | 比例 | 12m 对应值 |
|------|------|-----------|
| `local_window` | 1.7 × sensor_range | **20.4 m** |
| `lookahead_dist` | 0.35 × sensor_range | **4.2 m** |
| `subgoal_dist` | 0.6 × sensor_range | **7.2 m** |
| `goal_snap_dist` | 当前 0.5m（已知偏小）| 建议改为 **1.0 m** |

> **d1snap bug**：goal_snap_dist=0.5m < 0.66m（障碍物膨胀带宽度的1/2），  
> 在 30m 尺度有障碍时子目标无法吸附到可通行区域，导致 ABORT。  
> 修复：`goal_snap_dist` 改为 1.0m。

---

## 十一、修改世界

### 添加新障碍物

在 `unk_world.world` 的 `</world>` 前插入：

```xml
<model name="new_obstacle">
  <static>true</static>
  <pose>X Y 0.75 0 0 0</pose>          <!-- z=0.75 表示底面在地面 -->
  <link name="link">
    <collision name="collision">
      <geometry><cylinder><radius>0.3</radius><length>1.5</length></cylinder></geometry>
    </collision>
    <visual name="visual">
      <geometry><cylinder><radius>0.3</radius><length>1.5</length></cylinder></geometry>
      <material>
        <script>
          <uri>file://media/materials/scripts/gazebo.material</uri>
          <name>Gazebo/Red</name>
        </script>
      </material>
    </visual>
  </link>
</model>
```

### 用 Gazebo GUI 可视化编辑

1. 启动 Gazebo：`roslaunch gazebo_ros empty_world.launch world_name:=$(rospack find unk_nav_sim)/worlds/unk_world.world`
2. `Insert` 标签页 → 选 Box/Cylinder → 点击地面放置
3. 选 `Translate`（移动）/ `Rotate`（旋转）工具调整位置
4. `File → Save World As...` 保存为新的 `.world` 文件

---

## 十二、常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `/velodyne_points` 不存在 | xacro 宏没展开（旧语法） | `<HDL-32E>` 改为 `<xacro:HDL-32E>` |
| 点云存在但 RViz 看不到 | RViz 没加 PointCloud2 显示 | Add → By topic → /velodyne_points |
| 机器人穿墙 | `<collision>` 缺失 | 确认 collision 和 visual 都有 |
| 机器人下沉 | 地面 friction 太低 | `<mu>100</mu><mu2>50</mu2>` |
| 机器人原地打转 | cmd_vel 话题名不对 | `rostopic list` 确认是 `/cmd_vel` |
| Gazebo 崩溃 | 世界文件 XML 语法错误 | 用 `xmllint --noout unk_world.world` 检查 |
| 雷达检测不到障碍 | 障碍高度在 FOV 盲区 | 障碍物高度 ≥ 0.5m 即可被检测到 |
