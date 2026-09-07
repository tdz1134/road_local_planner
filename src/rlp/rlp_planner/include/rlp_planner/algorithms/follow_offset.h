#pragma once
// follow 方法默认算法 "offset"：走廊横向偏移族。
// 走廊中线裁剪到前瞻距离后按 lateral_offsets 平移生成一组平行候选，
// 进度 = 沿中线弧长，完全忽略全局终点。
#include "rlp_planner/method_algorithm.h"

namespace rlp {
namespace planner {

class FollowOffsetAlg : public MethodAlgorithm {
 public:
  explicit FollowOffsetAlg(const PlannerParams& p) : p_(p) {}
  const char* name() const override { return "offset"; }
  std::vector<Path> candidates(const PlanningContext& ctx) const override;

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
