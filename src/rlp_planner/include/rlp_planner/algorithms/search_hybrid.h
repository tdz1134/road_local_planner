#pragma once
// search 方法默认算法 "hybrid"：走廊偏移族 + 终点扇形族混合。
// 路面优先（走廊有效时先出走廊偏移族），再叠加朝终点扇形族，
// 由代价函数在"沿路"与"抄近路"之间权衡。
#include "rlp_planner/method_algorithm.h"

namespace rlp {
namespace planner {

class SearchHybridAlg : public MethodAlgorithm {
 public:
  explicit SearchHybridAlg(const PlannerParams& p) : p_(p) {}
  const char* name() const override { return "hybrid"; }
  std::vector<Path> candidates(const PlanningContext& ctx) const override;

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
