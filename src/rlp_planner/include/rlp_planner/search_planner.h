#pragma once
// SearchPlanner：定位好 / 朝全局终点推进的规划方法。
// 适用条件：走廊有效 + 终点可用。
// 策略：走廊偏移族（路面优先） + 朝终点扇形族，由代价函数在"沿路"与
// "抄近路"之间权衡（权重见 PlannerParams::w_offroad / w_progress）。
#include "rlp_planner/planner_base.h"

namespace rlp {
namespace planner {

class SearchPlanner : public PlannerBase {
 public:
  explicit SearchPlanner(const PlannerParams& p) : p_(p) {}
  PlanMode mode() const override { return PlanMode::SEARCH; }
  const char* name() const override { return "search"; }
  std::vector<Path> candidates(const PlanningContext& ctx) const override;

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
