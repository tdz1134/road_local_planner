#include "unk_nav/subgoal.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace unk {
namespace subgoal {

// ── 内部辅助（仅本文件使用）──────────────────────────────────────
namespace {

constexpr double kEps = 1e-9;

// 射线 o + t*u (t>=0) 与轴对齐包围盒的出口距离；起点在盒内时即为该方向上的可达距离。
// 无交返回 -1。
double rayBoxExit(double ox, double oy, double ux, double uy, double xmin, double xmax,
                  double ymin, double ymax) {
  double tmin = 0.0;
  double tmax = std::numeric_limits<double>::infinity();

  if (std::fabs(ux) < kEps) {
    if (ox < xmin || ox > xmax) return -1.0;  // 平行于该轴且在盒外
  } else {
    double t1 = (xmin - ox) / ux, t2 = (xmax - ox) / ux;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmin > tmax) return -1.0;
  }

  if (std::fabs(uy) < kEps) {
    if (oy < ymin || oy > ymax) return -1.0;
  } else {
    double t1 = (ymin - oy) / uy, t2 = (ymax - oy) / uy;
    if (t1 > t2) std::swap(t1, t2);
    tmin = std::max(tmin, t1);
    tmax = std::min(tmax, t2);
    if (tmin > tmax) return -1.0;
  }
  return tmax;
}

}  // namespace

// ── 公共接口（在 subgoal.h 中声明）───────────────────────────────

Result project(const GridMap& grid, const Point2D& goal_base, const NavParams& p) {
  Result r;
  if (grid.empty()) return r;

  r.goal_dist = std::hypot(goal_base.x, goal_base.y);
  r.goal_bearing = std::atan2(goal_base.y, goal_base.x);
  // 终点几乎重合于车位：不需要投影，交给上层的到达判定
  if (r.goal_dist < 1e-3) return r;

  const double ux = goal_base.x / r.goal_dist;
  const double uy = goal_base.y / r.goal_dist;

  // 窗口内缩两个格：子目标落在最外圈格上时，A* 的邻域扩展会大量越界，
  // 且螺旋吸附可能把它推到窗口外，白白浪费一个周期。
  const double m = 2.0 * grid.resolution;
  const double xmin = grid.origin_x + m;
  const double ymin = grid.origin_y + m;
  const double xmax = grid.origin_x + grid.width * grid.resolution - m;
  const double ymax = grid.origin_y + grid.height * grid.resolution - m;
  if (xmin >= xmax || ymin >= ymax) return r;  // 窗口太小，无法容纳子目标

  const double t_win = rayBoxExit(0.0, 0.0, ux, uy, xmin, xmax, ymin, ymax);
  if (t_win < 0.0) return r;  // 车体不在窗口内，输入异常

  // 三者取最小：前瞻距离、终点实际距离、窗口在该方向上的可达距离
  const double look = std::max(p.lookahead(), p.subgoalMin());
  double reach = std::min(look, r.goal_dist);
  if (t_win < reach) {
    reach = t_win;
    r.clipped_by_window = true;
  }
  if (reach < 1e-3) return r;  // 数值保护
  // 只有「被窗口截得太短」才算真的无法投影。终点本身就在近处时必须照常输出，
  // 否则会在 [goal_tolerance, subgoalMin*0.5) 之间形成死区：既不满足到达判定，
  // 又拿不到子目标 → 规划连续失败 → 误触发脱困直至 ABORT。
  if (r.clipped_by_window && reach < p.subgoalMin() * 0.5) return r;

  r.reach = reach;
  r.point = Point2D{ux * reach, uy * reach};
  r.valid = true;
  return r;
}

}  // namespace subgoal
}  // namespace unk
