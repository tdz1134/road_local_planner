#pragma once
// SearchPlanner：定位好 / 朝全局终点推进的规划方法。
// 适用条件：走廊有效 + 终点可用。
// 方法内部可注册多个候选生成算法，运行时按参数 search_alg 选择：
//   已注册算法见构造函数；默认 "hybrid"（走廊偏移族 + 终点扇形族，
//   由代价函数在"沿路"与"抄近路"之间权衡，权重见 w_offroad / w_progress）。
#include "rlp_planner/method_algorithm.h"

namespace rlp {
namespace planner {

class SearchPlanner : public MethodPlannerBase {
 public:
  explicit SearchPlanner(const PlannerParams& p);
  PlanMode mode() const override { return PlanMode::SEARCH; }
  const char* name() const override { return "search"; }

 protected:
  const std::string& algorithmName() const override { return p_.search_alg; }

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
