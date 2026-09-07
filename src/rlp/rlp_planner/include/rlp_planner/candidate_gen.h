#pragma once
// 候选路径生成工具：走廊横向偏移族 / 朝终点扇形族。
// 被不同规划方法共享的"生成原语"，方法本身负责组合与适用条件。
#include <vector>

#include "rlp_planner/planner_base.h"

namespace rlp {
namespace planner {
namespace candidate_gen {

// lookahead = max(min_lookahead, speed * lookahead_time)
double lookaheadLength(const PlannerParams& p, double speed);

// 走廊中线裁剪到前瞻距离后，按横向偏移族平移生成候选
std::vector<Path> corridorFamily(const PlannerParams& p, const road::Corridor& corridor,
                                 double lookahead);

// 从原点朝终点方位展开扇形直线族（终点在局部图外 → 虚拟目标点方向）
std::vector<Path> goalFan(const PlannerParams& p, const Point2D& goal, double lookahead);

}  // namespace candidate_gen
}  // namespace planner
}  // namespace rlp
