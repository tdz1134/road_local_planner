#include "unk_nav/subgoal.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "unk_nav/geom_util.h"

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

// 沿射线 (ux, uy) 从 step 起逐点查 feasibleAt，返回首个不可行采样点之前的距离。
// 全程可行则返回 t_max。
double rayFreeDistance(const GridMap& grid, double ux, double uy, double t_max, double step) {
  for (double t = step; t <= t_max + kEps; t += step) {
    if (!grid.feasibleAt(ux * t, uy * t)) {
      return std::max(0.0, t - step);
    }
  }
  return t_max;
}

// 从 t 开始沿射线向车侧逐点回退，取首个可行点。
// clearance > 0 时 d_free 已自带净空，0 次迭代即返回。
void snapBackToFeasible(const GridMap& grid, double ux, double uy, double* t, double step) {
  for (; *t > 0.0 && !grid.feasibleAt(ux * (*t), uy * (*t)); *t -= step) {
  }
}

}  // namespace

// ── 公共接口（在 subgoal.h 中声明）───────────────────────────────

Result project(const GridMap& grid, const Point2D& goal_base, const NavParams& p,
               const double* prev_bearing) {
  Result r;
  if (grid.empty()) { r.fail = FailReason::kGridEmpty; return r; }

  r.goal_dist = std::hypot(goal_base.x, goal_base.y);
  r.goal_bearing = std::atan2(goal_base.y, goal_base.x);
  // 终点几乎重合于车位：不需要投影，交给上层的到达判定
  if (r.goal_dist < 1e-3) { r.fail = FailReason::kGoalTooClose; return r; }

  const double ux_center = goal_base.x / r.goal_dist;
  const double uy_center = goal_base.y / r.goal_dist;

  // 窗口内缩两个格：子目标落在最外圈格上时，A* 的邻域扩展会大量越界，
  // 且螺旋吸附可能把它推到窗口外，白白浪费一个周期。
  const double m = 2.0 * grid.resolution;
  const double xmin = grid.origin_x + m;
  const double ymin = grid.origin_y + m;
  const double xmax = grid.origin_x + grid.width * grid.resolution - m;
  const double ymax = grid.origin_y + grid.height * grid.resolution - m;
  if (xmin >= xmax || ymin >= ymax) { r.fail = FailReason::kWindowTooSmall; return r; }

  // 中心射线窗口可达距离
  const double t_win_center = rayBoxExit(0.0, 0.0, ux_center, uy_center,
                                          xmin, xmax, ymin, ymax);
  if (t_win_center < 0.0) { r.fail = FailReason::kVehicleOutside; return r; }

  // reach_clip = min(lookahead, goal_dist, t_win)
  const double look = std::max(p.lookahead(), p.subgoalMin());
  double reach_clip = std::min(look, r.goal_dist);
  if (t_win_center < reach_clip) {
    reach_clip = t_win_center;
    r.clipped_by_window = true;
  }
  if (reach_clip < kEps) { r.fail = FailReason::kWindowTooShort; return r; }

  // 末段收敛：终点比 subgoalMin*0.5 还近时，不管截断与否都必须照常输出，
  // 否则 [goal_tolerance, subgoalMin*0.5) 区间既不判到达又不出路径 → 误触发脱困。
  const bool near_goal = r.goal_dist < p.subgoalMin() * 0.5;

  // 中心射线自由距离
  const double step = 0.5 * grid.resolution;
  const double d_free_center = rayFreeDistance(grid, ux_center, uy_center, reach_clip, step);

  // 中心是否被障碍截短
  const bool center_truncated = d_free_center < reach_clip - kEps;
  // 是否被终点距离限制（末段收敛标志）
  r.goal_limited = (!center_truncated && r.goal_dist <= reach_clip + kEps);

  // ── 中心一路空到 reach_clip → 直接返回（零额外开销，行为与旧版一致）──
  if (!center_truncated) {
    // 窗口截得太短 + 非末段收敛 → 无效（即使中心全空，reach 不足门槛也无意义）
    if (!near_goal && r.clipped_by_window && reach_clip < p.subgoalMin() * 0.5) {
      r.fail = FailReason::kWindowTooShort;
      return r;
    }
    r.reach = reach_clip;
    r.ray_free_dist = d_free_center;
    r.bearing = r.goal_bearing;
    r.point = Point2D{ux_center * r.reach, uy_center * r.reach};
    r.valid = true;
    return r;
  }

  // 中心被障碍截短
  r.truncated_by_obstacle = true;
  r.ray_free_dist = d_free_center;

  // 窗口截得太短 + 非末段收敛 → 无效（旧版保护，保留；必须在扇形展开之前，
  // 否则侧向候选可能在极小窗口内找到“可行但无意义”的近距离落点）
  if (!near_goal && r.clipped_by_window && reach_clip < p.subgoalMin() * 0.5) {
    r.fail = FailReason::kWindowTooShort;
    return r;
  }

  // ── 扇形展开 ──
  const double fan_half = p.subgoal_fan_half_deg * kPi / 180.0;

  if (fan_half < kEps) {
    // 扇形关闭：退回单射线中心截断（旧版行为）
    double t = reach_clip;
    snapBackToFeasible(grid, ux_center, uy_center, &t, step);
    if (t <= 0.0) { r.fail = FailReason::kNoFeasible; return r; }
    r.reach = t;
    r.bearing = r.goal_bearing;
    r.point = Point2D{ux_center * r.reach, uy_center * r.reach};
    r.valid = true;
    return r;
  }

  // 展开扇形候选
  const double dstep = std::max(p.subgoal_fan_step_deg, 1e-3) * kPi / 180.0;
  const int n_steps = static_cast<int>(std::floor(2.0 * fan_half / dstep + 0.5));
  const double min_reach = p.subgoalMin() * 0.5;
  const double subgoal_min = p.subgoalMin();

  double best_score = -1e18;
  double best_abs_dtheta = 1e18;
  bool found = false;
  int best_idx = 0;

  for (int i = 0; i <= n_steps; ++i) {
    const double dtheta = -fan_half + i * dstep;
    const double theta = r.goal_bearing + dtheta;
    const double ux = std::cos(theta);
    const double uy = std::sin(theta);

    // 该方向窗口可达距离
    const double t_win_i = rayBoxExit(0.0, 0.0, ux, uy, xmin, xmax, ymin, ymax);
    if (t_win_i < kEps) continue;

    // 候选 reach_clip：侧向不受 goal_dist 限制（它们本就不是去终点，是找绕行口）
    const double reach_i = std::min(look, t_win_i);

    // 沿射线量自由距离
    const double d_free_i = rayFreeDistance(grid, ux, uy, reach_i, step);
    if (d_free_i < min_reach) continue;  // 硬门槛

    // 实际 reach：受 obstacle clearance 调整
    double reach_actual;
    bool trunc_i;
    if (d_free_i >= reach_i - kEps) {
      // 该方向一路空到 reach_clip
      reach_actual = reach_i;
      trunc_i = false;
    } else {
      // 被障碍截短：留净空
      reach_actual = std::max(0.0, d_free_i - p.subgoal_clearance);
      trunc_i = true;
    }
    if (reach_actual < min_reach) continue;  // 净空后仍不足

    // 打分
    const double align = std::cos(dtheta);  // cos(θ_i - θ_goal) = cos(dtheta)
    const double free_term = std::min(d_free_i / subgoal_min, 1.0);
    double score = p.subgoal_align_w * align + p.subgoal_free_w * free_term;

    // 方向滞后项
    if (prev_bearing) {
      score += p.subgoal_prev_w * std::cos(geom::normalizeAngle(theta - *prev_bearing));
    }

    // 确定性 tie-break：分数相同取 |Δθ| 更小者；仍相同取先遍历到者（负侧）
    const double abs_dt = std::fabs(dtheta);
    if (score > best_score + kEps ||
        (score > best_score - kEps && abs_dt < best_abs_dtheta - kEps)) {
      best_score = score;
      best_abs_dtheta = abs_dt;
      best_idx = i;
      r.bearing = theta;
      r.reach = reach_actual;
      r.truncated_by_obstacle = trunc_i;
      r.ray_free_dist = d_free_i;
      r.clipped_by_window = (t_win_i < reach_i + kEps);
      found = true;
    }
  }

  if (!found) { r.fail = FailReason::kNoCandidate; return r; }

  r.fan_used = (best_idx != 0 || std::fabs(-fan_half + best_idx * dstep) > kEps);
  r.point = Point2D{std::cos(r.bearing) * r.reach, std::sin(r.bearing) * r.reach};
  r.valid = true;
  return r;
}

}  // namespace subgoal
}  // namespace unk
