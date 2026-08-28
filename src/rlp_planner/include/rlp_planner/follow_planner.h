#pragma once
// FollowPlanner：定位差 / 只依赖道路边界的规划方法。
// 适用条件：走廊有效（双侧、单侧补全、短时记忆外推均可）。
// 策略：走廊内横向偏移族采样，进度 = 沿中线弧长，完全忽略全局终点。
#include "rlp_planner/planner_base.h"

namespace rlp {
namespace planner {

class FollowPlanner : public PlannerBase {
 public:
  explicit FollowPlanner(const PlannerParams& p) : p_(p) {}
  PlanMode mode() const override { return PlanMode::FOLLOW; }
  const char* name() const override { return "follow"; }
  std::vector<Path> candidates(const PlanningContext& ctx) const override;

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
