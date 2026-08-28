#include "rlp_planner/candidate_gen.h"

#include <algorithm>
#include <cmath>

#include "rlp_common/geometry_util.h"

namespace rlp {
namespace planner {
namespace candidate_gen {
namespace {

// 在折线前拼接车辆原点 (0,0)，生成完整 Path（v1：直接连线，
// TODO(后续): 用多项式平滑衔接当前位姿与候选路径）
Path prependOrigin(const std::vector<Point2D>& pts) {
  std::vector<Point2D> full;
  full.reserve(pts.size() + 1);
  full.push_back({0.0, 0.0});
  for (const auto& p : pts) {
    // 跳过离原点过近的点，避免重复点导致切向退化
    if (std::hypot(p.x, p.y) > 0.2) full.push_back(p);
  }
  return toPath(full);
}

}  // namespace

double lookaheadLength(const PlannerParams& p, double speed) {
  return std::max(p.min_lookahead, speed * p.lookahead_time);
}

std::vector<Path> corridorFamily(const PlannerParams& p, const road::Corridor& corridor,
                                 double lookahead) {
  std::vector<Path> out;
  if (!corridor.valid()) return out;
  // 裁剪到前瞻距离，且不超过走廊自身覆盖（滚动时域 ≤ 边界覆盖）
  const double clipped_len =
      std::min(lookahead, polylineLength(corridor.centerline));
  const auto base = clipByArc(corridor.centerline, clipped_len);
  if (base.size() < 2) return out;

  const double max_offset = std::max(0.0, corridor.half_width);
  for (const double off_req : p.lateral_offsets) {
    const double off = std::copysign(std::min(std::fabs(off_req), max_offset), off_req);
    out.push_back(prependOrigin(offsetPolyline(base, off)));
  }
  return out;
}

std::vector<Path> goalFan(const PlannerParams& p, const Point2D& goal, double lookahead) {
  std::vector<Path> out;
  const double dist = std::hypot(goal.x, goal.y);
  if (dist < 1e-3) return out;
  const double base_ang = std::atan2(goal.y, goal.x);
  const double fan = p.goal_fan_deg * M_PI / 180.0;
  const int n = std::max(1, p.n_goal_bearings);
  for (int k = 0; k < n; ++k) {
    const double t = (n == 1) ? 0.5 : static_cast<double>(k) / (n - 1);
    const double ang = base_ang - fan + 2.0 * fan * t;
    const double step = p.path_spacing;
    std::vector<Point2D> line;
    for (double s = step; s <= lookahead; s += step) {
      line.push_back({s * std::cos(ang), s * std::sin(ang)});
    }
    if (!line.empty()) out.push_back(prependOrigin(line));
  }
  return out;
}

}  // namespace candidate_gen
}  // namespace planner
}  // namespace rlp
