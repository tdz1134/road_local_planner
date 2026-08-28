#pragma once
// FollowPlanner：定位差 / 只依赖道路边界的规划方法。
// 适用条件：走廊有效（双侧、单侧补全、短时记忆外推均可）。
// 方法内部可注册多个候选生成算法，运行时按参数 follow_alg 选择：
//   已注册算法见构造函数；默认 "offset"（走廊横向偏移族，忽略全局终点）。
#include "rlp_planner/method_algorithm.h"

namespace rlp {
namespace planner {

class FollowPlanner : public MethodPlannerBase {
 public:
  explicit FollowPlanner(const PlannerParams& p);
  PlanMode mode() const override { return PlanMode::FOLLOW; }
  const char* name() const override { return "follow"; }

 protected:
  const std::string& algorithmName() const override { return p_.follow_alg; }

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
