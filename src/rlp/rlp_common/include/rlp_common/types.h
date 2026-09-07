#pragma once
// 公共层：基础几何类型、栅格地图、全局参数。无领域知识。
// 依赖方向：rlp_common <- rlp_road <- rlp_planner <- rlp_node
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace rlp {

struct Point2D {
  double x = 0.0, y = 0.0;
};

// 局部占据栅格（假定在车体系：x 向前，y 向左）
// data: -1 unknown, 0 free, 100 occupied
struct GridMap {
  double resolution = 0.1;
  double origin_x = 0.0, origin_y = 0.0;  // (0,0) 栅格中心在车体系的位置
  int width = 0, height = 0;
  std::vector<int8_t> data;

  // 越界返回 -1
  int8_t valueAt(double x, double y) const {
    if (resolution <= 0 || width <= 0 || height <= 0 || data.empty()) return -1;
    const int mx = static_cast<int>(std::floor((x - origin_x) / resolution));
    const int my = static_cast<int>(std::floor((y - origin_y) / resolution));
    if (mx < 0 || my < 0 || mx >= width || my >= height) return -1;
    return data[static_cast<size_t>(my) * width + mx];
  }
  // 占据阈值取 65；unknown(-1) 在骨架阶段视为可通行（TODO: 未知区域单独代价）
  bool occupiedAt(double x, double y) const { return valueAt(x, y) >= 65; }
  bool empty() const { return data.empty(); }
};

struct PathPoint {
  Point2D p;
  double s = 0.0;  // 弧长 m
};
using Path = std::vector<PathPoint>;

// 规划模式（由定位质量仲裁得到）
enum class PlanMode {
  FOLLOW,  // 定位差：只沿路走，忽略全局终点
  SEARCH,  // 定位好：朝终点推进 + 路面优先
};

inline const char* modeName(PlanMode m) {
  return m == PlanMode::FOLLOW ? "FOLLOW" : "SEARCH";
}

// 全部可调参数（与 config/params.yaml 一一对应）
struct PlannerParams {
  // 车辆能力 / 安全
  double v_max = 8.3;
  double a_decel_max = 2.0;
  double a_lat_max = 2.5;
  double w_max = 0.8;
  double t_reaction = 0.2;
  double safety_margin = 2.0;
  // 规划
  double plan_freq = 10.0;
  double path_spacing = 0.5;
  double lookahead_time = 2.0;
  double min_lookahead = 10.0;
  std::vector<double> lateral_offsets = {0.0, -1.0, 1.0};
  int n_goal_bearings = 5;
  double goal_fan_deg = 25.0;
  // 各规划方法的候选生成算法选择（按名字匹配已注册算法；
  // 未知名字回退到第一个注册的默认算法。可选值见各方法构造函数）
  std::string follow_alg = "offset";
  std::string search_alg = "hybrid";
  std::string free_alg = "fan";
  // A* 搜索参数
  int astar_max_iter = 50000;  // A* 最大迭代次数（安全上限）
  // RRT 搜索参数
  int rrt_max_iter = 2000;     // RRT 最大迭代次数（安全上限）
  double rrt_step_size = 1.0;  // RRT 每次扩展步长 m（太大跳窄通道，太小生长慢）
  double rrt_goal_bias = 0.2;  // RRT 朝终点采样的概率 [0,1]
  // 边界 / 走廊
  double boundary_timeout = 0.5;
  double corridor_hold_max = 3.0;
  double corridor_inflate_rate = 0.3;
  double width_min = 2.0;
  double width_max = 20.0;
  double boundary_margin_base = 0.5;
  double boundary_margin_speed_gain = 0.06;
  // 模式仲裁
  double q_high = 0.7;
  double q_low = 0.5;
  double mode_dwell = 2.0;
  // 代价权重
  double w_offroad = 5.0;
  double w_smooth = 1.0;
  double w_progress = 1.0;
  double w_consistency = 0.5;
  double collision_cost = 1e6;
};

}  // namespace rlp
