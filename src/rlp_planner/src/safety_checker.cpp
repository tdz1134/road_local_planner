#include "rlp_planner/safety_checker.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rlp {
namespace planner {
namespace {
constexpr double kInfDist = std::numeric_limits<double>::infinity();

// 相邻三点的外接圆曲率估计
double threePointCurvature(const Point2D& a, const Point2D& b, const Point2D& c) {
  const double ab = std::hypot(b.x - a.x, b.y - a.y);
  const double bc = std::hypot(c.x - b.x, c.y - b.y);
  const double ca = std::hypot(c.x - a.x, c.y - a.y);
  const double denom = ab * bc * ca;
  if (denom < 1e-9) return 0.0;
  const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  return std::fabs(2.0 * cross) / denom;
}

}  // namespace

SafetyChecker::SafetyChecker(const PlannerParams& p) : p_(p) {}

SafetyResult SafetyChecker::check(const Path& path, const GridMap& map,
                                  double current_speed) const {
  SafetyResult res;
  const double v = std::fabs(current_speed);

  // 1) 路径上第一个占据点距离（v1：unknown 视为可通行）
  double d_obs = kInfDist;
  for (const auto& pp : path) {
    if (map.occupiedAt(pp.p.x, pp.p.y)) {
      d_obs = pp.s;
      break;
    }
  }

  // 2) 制动包络校验
  res.obs_distance = d_obs;
  res.brake_distance = v * v / (2.0 * p_.a_decel_max) + v * p_.t_reaction + p_.safety_margin;
  res.pass = (d_obs > res.brake_distance);

  // 3) 推荐速度 = min(v_max, 障碍约束, 曲率约束)
  //    障碍约束：v²/(2a) + v·T + margin <= d_obs 的正根
  const double a = p_.a_decel_max, T = p_.t_reaction;
  const double dd = std::max(0.0, d_obs - p_.safety_margin);
  double v_obs = std::isinf(d_obs) ? p_.v_max
                                   : -a * T + std::sqrt(a * T * a * T + 2.0 * a * dd);

  double k_max = 0.0;
  for (size_t i = 2; i < path.size(); ++i) {
    k_max = std::max(k_max, threePointCurvature(path[i - 2].p, path[i - 1].p, path[i].p));
  }
  const double v_curv =
      k_max > 1e-6 ? std::sqrt(p_.a_lat_max / k_max) : p_.v_max;

  res.max_safe_speed =
      std::max(0.0, std::min({p_.v_max, std::isinf(v_obs) ? p_.v_max : v_obs, v_curv}));
  return res;
}

}  // namespace planner
}  // namespace rlp
