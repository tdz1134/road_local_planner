#pragma once
// search 方法算法 "astar"：在局部栅格上用 A* 搜索，直接输出一条最优路径。
// 搜索式算法，不走候选采样 + CostEvaluator 流程。
// 走廊有效时作为软约束（走廊外栅格增加额外代价），无走廊时纯栅格搜索。
#include "rlp_planner/method_algorithm.h"

namespace rlp {
namespace planner {

class SearchAStarAlg : public MethodAlgorithm {
 public:
  explicit SearchAStarAlg(const PlannerParams& p);
  const char* name() const override { return "astar"; }

  // 采样式：不使用（搜索式算法不走此流程）
  std::vector<Path> candidates(const PlanningContext& ctx) const override {
    (void)ctx;
    return {};
  }

  // 搜索式：A* 在栅格上搜索，直接返回最终路径
  Path directPlan(const PlanningContext& ctx) const override;

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
