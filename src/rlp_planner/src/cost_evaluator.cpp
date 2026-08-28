#include "rlp_planner/cost_evaluator.h"

#include <algorithm>
#include <cmath>

#include "rlp_common/geometry_util.h"

namespace rlp {
namespace planner {
namespace {

double normalizeAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

std::vector<Point2D> pathToPoints(const Path& path) {
  std::vector<Point2D> pts;
  pts.reserve(path.size());
  for (const auto& pp : path) pts.push_back(pp.p);
  return pts;
}

}  // namespace

CostEvaluator::CostEvaluator(const PlannerParams& p) : p_(p) {}

double CostEvaluator::evaluate(const Path& path, const GridMap& map,
                               const road::Corridor& corridor, PlanMode mode,
                               const Point2D& goal, bool goal_valid,
                               const Path& prev_path, double margin) const {
  if (path.size() < 2) return p_.collision_cost;

  double offroad = 0.0;
  double smooth = 0.0;
  double consistency = 0.0;

  // 1) 碰撞（硬代价）+ 越出走廊（软代价，边界为缩进 margin 后的界限）
  const bool has_corridor = corridor.valid();
  double lim = 0.0;
  if (has_corridor) lim = std::max(0.0, corridor.half_width - margin);
  for (const auto& pp : path) {
    if (map.occupiedAt(pp.p.x, pp.p.y)) return p_.collision_cost;
    if (has_corridor) {
      const double d = pointToPolylineDistance(pp.p, corridor.centerline);
      if (d > lim) offroad += (d - lim) * (d - lim);
    }
  }

  // 2) 平滑性：相邻段航向变化量平方和
  std::vector<double> headings;
  for (size_t i = 1; i < path.size(); ++i) {
    headings.push_back(std::atan2(path[i].p.y - path[i - 1].p.y,
                                  path[i].p.x - path[i - 1].p.x));
  }
  for (size_t i = 1; i < headings.size(); ++i) {
    const double da = normalizeAngle(headings[i] - headings[i - 1]);
    smooth += da * da;
  }

  // 3) 进度：FOLLOW = 沿走廊弧长；SEARCH = 朝终点方向投影
  double progress = 0.0;
  const auto& end = path.back().p;
  if (mode == PlanMode::FOLLOW || !goal_valid) {
    progress = -path.back().s;
  } else {
    const double gd = std::max(std::hypot(goal.x, goal.y), 1e-3);
    progress = -(end.x * goal.x + end.y * goal.y) / gd;
  }

  // 4) 一致性：与上一周期路径的平均偏差（防逐帧抖动）
  if (prev_path.size() > 1) {
    const auto prev_pts = pathToPoints(prev_path);
    double sum = 0.0;
    for (const auto& pp : path) sum += pointToPolylineDistance(pp.p, prev_pts);
    consistency = sum / static_cast<double>(path.size());
  }

  return p_.w_offroad * offroad + p_.w_smooth * smooth +
         p_.w_progress * progress + p_.w_consistency * consistency;
}

}  // namespace planner
}  // namespace rlp
