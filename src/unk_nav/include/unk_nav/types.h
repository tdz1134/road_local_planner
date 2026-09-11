#pragma once
// 未知环境局部反应式导航 —— 基础类型与接口契约。
//
// ── 核心约束（决定了整个模块的形态，任何改动都不得破坏）─────────────
//   1. 规划器只能看到车周有限范围的局部占据栅格（车体系 base_link）；
//   2. 终点在局部窗口**之外**，距离远大于窗口尺寸；
//   3. 因此不存在全局地图，也**不做全局搜索**：每周期把远处终点沿方向投影进
//      窗口内得到一个「子目标」，在局部栅格上滚动规划（滚动时域）；
//   4. 环境未知，栅格中存在 unknown 区，采取乐观策略：unknown 可通行但代价略高；
//   5. 「越界」（窗口之外）与「unknown」严格区分 —— 前者绝不可通行，否则
//      搜索会跑出栅格边界。
//
// ── 依赖 ───────────────────────────────────────────────────────────
//   仅 C++14 标准库，和yaml-cpp。不含任何 ROS / 第三方库引用，可直接交叉编译到 MDC。
//   与 ROS 的对接由将来独立的节点壳完成，本头文件不得引入 ROS 类型。
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace unk {

// ---- 数学常量 ----
// M_PI / M_SQRT2 属于 POSIX 扩展，标准 C++ 并不强制定义。为保证在受限的
// 交叉编译工具链（MDC）上也能直接构建，本模块统一使用下面两个常量，
// 不依赖任何宏是否已被定义。
constexpr double kPi = 3.14159265358979323846;
constexpr double kSqrt2 = 1.41421356237309504880;

// ---- 栅格取值约定（与 nav_msgs/OccupancyGrid 一致，便于将来套 ROS 壳）----
constexpr int8_t kUnknown = -1;          // 未探测：乐观放行
constexpr int8_t kFree = 0;              // 空闲
constexpr int8_t kOccupied = 100;        // 占据
constexpr int8_t kOutOfMap = -2;         // 越界（窗口之外）：绝不可通行
constexpr int8_t kOccupyThreshold = 65;  // >= 该值判为占据

struct Point2D {
  double x = 0.0, y = 0.0;
};

// 平面位姿（全局 / 里程计系）
struct Pose2D {
  double x = 0.0, y = 0.0, yaw = 0.0;
};

// 局部占据栅格，车体系（x 向前，y 向左）。车辆通常位于窗口中心。
// 与 rlp::GridMap 字段刻意保持兼容，将来融入 road_local_planner 时可直接对接。
struct GridMap {
  double resolution = 0.05;
  double origin_x = 0.0, origin_y = 0.0;  // 栅格 (0,0) 格左下角在车体系的位置
  int width = 0, height = 0;
  std::vector<int8_t> data;

  bool empty() const {
    return data.empty() || width <= 0 || height <= 0 || resolution <= 0.0;
  }

  bool inBounds(int gx, int gy) const {
    return gx >= 0 && gy >= 0 && gx < width && gy < height;
  }

  // 车体系坐标 → 栅格索引；越界返回 false
  bool worldToGrid(double x, double y, int* gx, int* gy) const {
    if (empty()) return false;
    const int mx = static_cast<int>(std::floor((x - origin_x) / resolution));
    const int my = static_cast<int>(std::floor((y - origin_y) / resolution));
    if (!inBounds(mx, my)) return false;
    if (gx != nullptr) *gx = mx;
    if (gy != nullptr) *gy = my;
    return true;
  }

  // 栅格索引 → 格中心的车体系坐标
  void gridToWorld(int gx, int gy, double* x, double* y) const {
    if (x != nullptr) *x = origin_x + (gx + 0.5) * resolution;
    if (y != nullptr) *y = origin_y + (gy + 0.5) * resolution;
  }

  // 按索引取值；越界返回 kOutOfMap
  int8_t valueAtCell(int gx, int gy) const {
    if (!inBounds(gx, gy)) return kOutOfMap;
    return data[static_cast<size_t>(gy) * static_cast<size_t>(width) +
                static_cast<size_t>(gx)];
  }

  // 按车体系坐标取值；越界返回 kOutOfMap
  int8_t valueAt(double x, double y) const {
    int gx = 0, gy = 0;
    if (!worldToGrid(x, y, &gx, &gy)) return kOutOfMap;
    return valueAtCell(gx, gy);
  }

  // 严格占据判定（越界不算「占据」，算「越界」，用 blockedAt 统一处理）
  bool occupiedAt(double x, double y) const { return valueAt(x, y) >= kOccupyThreshold; }
  bool freeAt(double x, double y) const { return valueAt(x, y) == kFree; }
  bool unknownAt(double x, double y) const { return valueAt(x, y) == kUnknown; }

  // 不可通行 = 占据 或 越界。unknown 不在此列（乐观放行）。
  bool blockedAt(double x, double y) const {
    const int8_t v = valueAt(x, y);
    return v >= kOccupyThreshold || v == kOutOfMap;
  }

  bool feasibleCell(int gx, int gy) const {
    const int8_t v = valueAtCell(gx, gy);
    return v != kOutOfMap && v < kOccupyThreshold;
  }

  bool feasibleAt(double x, double y) const {
    const int8_t v = valueAt(x, y);
    return v != kOutOfMap && v < kOccupyThreshold;
  }
};

struct PathPoint {
  Point2D p;
  double s = 0.0;  // 累计弧长 m
  double k = 0.0;  // 曲率 1/m（三点外接圆估计）
};
using Path = std::vector<PathPoint>;

// 导航行为状态。上层只需处理这五个，不存在「预留但永不产生」的状态。
// 凹形障碍绕行（Bug 式 BOUNDARY_FOLLOW）与脱困运动原语已明确排除在本模块范围外，
// 见 README「能力边界」。因此遇到凹形障碍局部极小时，行为是 RECOVERY 上报数周期后
// 转 ABORT —— 主动放弃并把决定权交回上层，而不是原地振荡装作还在工作。
enum class NavState {
  IDLE,      // 无有效终点
  GO,        // 朝终点推进
  RECOVERY,  // 规划连续失败 / 无进展，上报等待（v0 不产生脱困运动）
  ARRIVED,   // 到达终点
  ABORT,     // 重试次数用尽，放弃
};

inline const char* navStateName(NavState s) {
  switch (s) {
    case NavState::IDLE: return "IDLE";
    case NavState::GO: return "GO";
    case NavState::RECOVERY: return "RECOVERY";
    case NavState::ARRIVED: return "ARRIVED";
    case NavState::ABORT: return "ABORT";
  }
  return "?";
}

// 全部可调参数。
// 注意「无量纲化」原则：所有与感知尺度相关的距离都表达为 sensor_range 的比例，
// 换车型 / 换雷达时只改 sensor_range 一个值，整定结果按比例自动迁移到实车。
struct NavParams {
  // ---- 车辆能力 ----
  double v_max = 0.22;         // 最大线速度 m/s
  double w_max = 2.84;         // 最大角速度 rad/s
  double a_decel_max = 0.5;    // 最大减速度 m/s^2
  double a_lat_max = 1.0;      // 最大横向加速度 m/s^2
  double robot_radius = 0.105; // 车体半径 m

  // ---- 感知尺度（所有规划距离的基准）----
  double sensor_range = 12.0;  // 激光雷达量程 m

  // ---- 栅格预处理 ----
  // 膨胀半径 = 车体半径 + 余量。这个「余量」是给**下游控制器的跟踪误差**留的，
  // 不是随便给的小数：路径是在膨胀后的栅格上搜出来的，只保证中心线有
  // (inflation_radius - robot_radius) 的净空。若控制器跟踪误差（尤其纯跟踪在拐点
  // 处切内角的弦差）超过这个净空，车就会开进膨胀层被自己的安全边距困死。
  // 因此：要么控制器保证误差不超过净空，要么把这里调大。
  double inflation_radius = 0.16;        // 障碍膨胀半径 m
  double footprint_clear_radius = 0.12;  // 车体足迹强制清空半径（防膨胀吃掉自己）
  bool inflate_unknown = false;          // 是否把 unknown 也膨胀成障碍（默认否，保持乐观）

  // ---- 滚动时域规划 ----
  double lookahead_ratio = 0.35;   // 子目标投影距离 = ratio * sensor_range
  double subgoal_min_ratio = 0.10; // 子目标最小距离 = ratio * sensor_range
  double path_spacing = 0.05;      // 路径等距重采样间距 m
  // 曲率测量基线 m：曲率用弧长相距 ±baseline/2 的两点估计，而非相邻点。
  // 必须 > 0，否则曲率会随 path_spacing 变化（点距 0.05m 时栅格阶梯的微小抖动
  // 就能被放大成 30+/m 的假曲率，把过弯速度压到爬行）。取值应远大于 path_spacing，
  // 大致对应车辆 0.2~0.5s 走过的距离。
  double curvature_baseline = 0.30;
  double goal_tolerance = 0.15;    // 到达判定距离 m
  // 到达判定**只判距离、不判航向**（差速底盘原地转向能力充足，强制对齐会让车在
  // 终点附近来回转圈）。因此这里没有 yaw_tolerance：需要定向停靠时，先加参数再加
  // 判定逻辑，不要留一个「设了但不生效」的旋钮骗集成方。

  // ---- A* ----
  int astar_max_iter = 200000;   // 迭代上限（安全护栏）
  double unknown_cost = 1.4;     // unknown 格代价倍率（>1 偏向已知区，仍可穿越）
  // 启发式权重 f = g + w*h。1.0 = 严格最优（默认；仅靠「f 相等时 h 小者优先」打破
  // 对称，扩展数已骤降）。>1（如 1.05）= 加权 A*，扩展更少但代价次优 ≤ w×。
  double astar_w = 1.0;
  // 障碍距离软代价（批次2）：进入单格代价 = step×(1+obstacle_cost_k·exp(-d/obstacle_cost_sigma))，
  // d 为该格到最近障碍的距离。k<=0 关闭（默认）。启用后可把 inflation_radius 调薄：
  // 软梯度替代厚膨胀带把路径推向通道中央、离墙更远。代价因子恒 ≥1 → 不破坏 A* 最优性。
  double obstacle_cost_k = 0.0;
  double obstacle_cost_sigma = 0.35;   // 衰减尺度 m，约等于「想额外保持的离墙净空」
  // 路径一致性软代价（批次3）：单格代价再加 consistency_k·(1-exp(-d_prev/consistency_sigma))，
  // d_prev 为该格到上一帧路径的距离。解「轴对称镜像 f/g/h 全相等 → 每帧左右翻烧饼」的抖动：
  // 沿上帧走天然更便宜。k<=0 关闭（默认）。只加不减 → 不破坏 A* 对新代价函数的最优性。
  double consistency_k = 0.0;
  double consistency_sigma = 0.4;      // 黏性走廊半宽 m；太大黏过头、该改道时反应慢
  // 子目标落在障碍上时，螺旋吸附到最近可行格的搜索半径（**米**，不是格数）。
  // 用米而非格数：换栅格分辨率时物理含义不变，否则 res 从 0.05 改到 0.10
  // 吸附范围会悄悄翻倍。
  double goal_snap_dist = 0.5;

  // ---- 子目标扇形选取（终点模式专用，沿路模式不读）----
  // 中心方向（θ=goal_bearing）被障碍截断时展开扇形候选，打分选优。
  // subgoal_fan_half_deg=0 → 关闭扇形，退回单射线（与 subgoal_clearance=0 一起可完全复现旧行为）。
  double subgoal_fan_half_deg = 90.0;   // 扇形半角 deg；0=关闭扇形
  double subgoal_fan_step_deg = 5.0;    // 扇形角步长 deg
  double subgoal_align_w    = 3.0;      // 终点对齐权重 cos(θ-θ_goal)；必须 > subgoal_free_w
  double subgoal_free_w     = 1.0;      // 饱和自由距离权重（饱和参考 = subgoalMin()）
  double subgoal_prev_w     = 1.0;      // 上帧方向一致性权重；0=关闭滞后
  double subgoal_clearance  = 0.3;      // 截断时子目标与膨胀带边缘的净空 m；0=复现旧行为
  double goal_clear_radius  = 0.0;      // 终点清洞半径 m；0=关闭（终点贴墙场景才需要）

  // ---- 速度规划 ----
  double safety_margin = 0.12;  // 制动包络附加安全距离 m
  double t_reaction = 0.1;      // 反应时间 s
  // 差速底盘必须同时强制三条运动约束，只限曲率会在高速段侧滑失稳：
  //   角速度 w = v*kappa <= w_max、横向加速度 a_lat = v^2*kappa <= a_lat_max、
  //   曲率变化率 dk/dt <= dk_max
  // kappa_max 是**几何硬门限**，默认 0 = 关闭：差速底盘能原地转向，曲率本身不构成
  // 运动学约束，硬卡它会把栅格 A* 的正常直角拐点（曲率约 sqrt(2)/resolution）全判死。
  // 只有阿克曼底盘或需要禁止原地打方向的平台才应显式设一个正值。
  double kappa_max = 0.0;   // 最大曲率 1/m，0 = 不启用硬门限
  double dk_max = 50.0;     // 最大曲率变化率 1/(m*s)

  // ---- 路径平滑 ----
  // 倒角半径不直接给，而是由「目标过弯速度」反解：
  //   r = max(v_corner / w_max, v_corner^2 / a_lat_max)
  // 同时满足角速度与横向加速度两条约束，换车型时只需给出想要的过弯速度。
  // 注意这是**动力学参数，不随 sensor_range 缩放**（换雷达不该改变过弯半径）。
  double smooth_corner_speed = 0.22;  // 目标过弯速度 m/s；0 = 关闭倒角
  int smooth_laplacian_iters = 2;     // 拉普拉斯松弛迭代次数；0 = 关闭
  double smooth_laplacian_lambda = 0.2;  // 松弛系数 (0,1)
  int smooth_shrink_retry = 3;        // 倒角圆弧碰撞时半径折半重试次数

  // ---- 无进展 / 卡死判定 ----
  // 两个触发源：规划连续失败（checkPlanFail）、规划成功但没能朝终点推进（checkStuck）。
  // 判的是「推进量」而非「位移量」：车在凹槽里横向来回振荡时位移不小但零进展，
  // 位移量判定会一直沉默、持续上报 GO 满速（实测振荡 280 秒不报）。详见 behavior_fsm.cpp。
  double stuck_time = 3.0;         // 判定时长 s
  double stuck_dist = 0.05;        // 该时长内朝终点推进小于此值 → 判无进展 m
  int recovery_max_retry = 3;      // 重试上限，超过即 ABORT

  // ---- 控制器（PurePursuitController，见 controller.h）----
  // 放在 NavParams 里是为了让「一份 yaml 配全部算法参数」成立；
  // 集成层直接 params.pursuit_lookahead 构造控制器，不再单独走参数服务器。
  double pursuit_lookahead = 0.5;  // 纯跟踪前视距离 m：调小贴线紧但抖，调大平滑但切内角深

  double plan_freq = 10.0;  // 规划频率 Hz

  // ---- 沿路模式（无定位，路线 A：两侧路缘/墙夹出的走廊即道路）----
  // follow_road=true 时，NavCore 不再用「全局终点经定位投影」得到子目标，而是直接
  // 从局部栅格的道路走廊几何推出一个车体系前瞻点（见 road_follow.h）。因此该模式
  // **完全不读 vehicle_pose / goal** —— 感知(grid_node)、规划、控制本就只吃 base 系，
  // 定位在整条链路里唯一的实质用途就是放置全局终点，去掉它即得无定位沿路。
  // 前进方向以车头朝向为基准（前向半球扇形扫描），无需任何全局定向。
  // 默认 false = 原终点导航行为逐字节不变；road 模式建议用独立的 nav_params_road.yaml。
  bool   follow_road          = false; // true=道路前瞻子目标；false=原终点投影（默认）
  double road_fan_half_deg    = 75.0;  // 车头前向半球扫描半角 deg（θ=0 为正前方 +x）
  double road_fan_step_deg    = 3.0;   // 扇形扫描角步长 deg
  double road_lookahead_ratio = 0.35;  // 沿路前瞻距离 = ratio * sensor_range
  double road_free_w          = 2.0;   // 打分权重：前方自由距离（越空越想走）。必须 > road_align_w，
                                       // 否则对齐项 cosθ 过强，车会顶着弯道外墙直到几乎撞上才转
  double road_align_w         = 1.0;   // 打分权重：与车头对齐度 cosθ（越想直行，抑制无谓摆动）

  // ---- 派生量（勿手工设置）----
  double lookahead() const { return lookahead_ratio * sensor_range; }
  double subgoalMin() const { return subgoal_min_ratio * sensor_range; }
  double roadLookahead() const { return road_lookahead_ratio * sensor_range; }
};

// 单周期输入
struct NavInput {
  double now = 0.0;
  GridMap local_grid;      // 车体系局部栅格（上游原始栅格，核心内部自行膨胀）
  Pose2D vehicle_pose;     // 车体在全局 / 里程计系的位姿
  Point2D goal;            // 终点，全局系，可在窗口外任意远
  bool goal_valid = false; // 终点是否有效（tf 可用、上游已下发等）
  double current_speed = 0.0;
  // 车速是否来自有效测量（body 系轮速 / odom.twist）。沿路模式的前进位移棘轮依赖它；
  // false（如无定位又不接里程计）时该判定退化为仅「规划连续失败」检测。默认 true，
  // 保持终点模式与既有单测行为不变。
  bool speed_valid = true;
};

// 单周期输出
struct NavResult {
  Path path;                       // 车体系局部路径
  double recommended_speed = 0.0;  // 推荐速度 m/s
  NavState state = NavState::IDLE;
  bool emergency_stop = true;
  Point2D subgoal;                 // 本周期实际使用的子目标（车体系），调试用
  bool subgoal_reachable = false;  // 子目标是否被 A* 搜到
  Point2D goal_base;               // 终点在车体系的位置（调试/可视化用），无效时={0,0}
  bool goal_base_valid = false;    // goal_base 是否有效
  std::string reason = "init";     // 调试说明

  // 扇形候选（调试可视化用。终点模式仅扇形展开时非空；沿路模式每帧展开故总是非空）
  struct FanCandidate {
    double bearing = 0.0;    // 候选方位角 rad
    double d_free = 0.0;     // 沿该射线的自由距离 m
    double reach = 0.0;      // 实际 reach（含净空调整）m
    double score = 0.0;      // 打分
    bool feasible = false;   // 是否通过硬门槛
    bool selected = false;   // 是否被选中
  };
  std::vector<FanCandidate> fan_candidates;
};

}  // namespace unk
