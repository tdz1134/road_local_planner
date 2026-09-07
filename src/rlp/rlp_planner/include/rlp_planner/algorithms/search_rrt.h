#pragma once
// search 方法算法 "rrt"：在局部栅格上用 RRT 搜索，直接输出一条可行路径。
// 搜索式算法，不走候选采样 + CostEvaluator 流程。
// 走廊有效时偏向走廊内采样，无走廊时纯随机采样。
#include "rlp_planner/method_algorithm.h"

namespace rlp {
namespace planner {

class SearchRRTAlg : public MethodAlgorithm {
 public:
  explicit SearchRRTAlg(const PlannerParams& p);
  const char* name() const override { return "rrt"; }

  // 采样式：不使用（搜索式算法不走此流程）
  std::vector<Path> candidates(const PlanningContext& ctx) const override {
    (void)ctx;
    return {};
  }

  // 搜索式：RRT 在栅格上搜索，直接返回最终路径
  Path directPlan(const PlanningContext& ctx) const override;

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
