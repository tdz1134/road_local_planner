#pragma once
// FreePlanner：无走廊退化方法（定位好但道路边界完全不可用）。
// 适用条件：走廊无效 + 终点可用（定位差时不允许使用，直接停车）。
// 策略：纯终点方向扇形直线族，依赖栅格避障。
#include "rlp_planner/planner_base.h"

namespace rlp {
namespace planner {

class FreePlanner : public PlannerBase {
 public:
  explicit FreePlanner(const PlannerParams& p) : p_(p) {}
  PlanMode mode() const override { return PlanMode::SEARCH; }
  const char* name() const override { return "free"; }
  std::vector<Path> candidates(const PlanningContext& ctx) const override;

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
