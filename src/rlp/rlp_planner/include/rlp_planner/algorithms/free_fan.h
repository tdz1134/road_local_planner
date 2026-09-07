#pragma once
// free 方法默认算法 "fan"：终点方向扇形直线族。
// 从车辆原点朝终点方位展开扇形直线候选，依赖栅格避障。
#include "rlp_planner/method_algorithm.h"

namespace rlp {
namespace planner {

class FreeFanAlg : public MethodAlgorithm {
 public:
  explicit FreeFanAlg(const PlannerParams& p) : p_(p) {}
  const char* name() const override { return "fan"; }
  std::vector<Path> candidates(const PlanningContext& ctx) const override;

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
