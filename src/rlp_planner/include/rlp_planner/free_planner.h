#pragma once
// FreePlanner：无走廊退化方法（定位好但道路边界完全不可用）。
// 适用条件：走廊无效 + 终点可用（定位差时不允许使用，直接停车）。
// 方法内部可注册多个候选生成算法，运行时按参数 free_alg 选择：
//   已注册算法见构造函数；默认 "fan"（终点方向扇形直线族，依赖栅格避障）。
#include "rlp_planner/method_algorithm.h"

namespace rlp {
namespace planner {

class FreePlanner : public MethodPlannerBase {
 public:
  explicit FreePlanner(const PlannerParams& p);
  PlanMode mode() const override { return PlanMode::SEARCH; }
  const char* name() const override { return "free"; }

 protected:
  const std::string& algorithmName() const override { return p_.free_alg; }

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
