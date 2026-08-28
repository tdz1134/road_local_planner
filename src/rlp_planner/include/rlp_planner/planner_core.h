#pragma once
// 规划器核心（总调度）：一个 plan() 完成
//   RoadModel(道路) → PlannerRouter(选方法) → 方法.candidates(候选) →
//   CostEvaluator(评分) → SafetyChecker(校验) → PlanResult
// 滚动调用即构成增量式局部规划。纯 C++，无 ROS 依赖。
#include "rlp_planner/cost_evaluator.h"
#include "rlp_planner/follow_planner.h"
#include "rlp_planner/free_planner.h"
#include "rlp_planner/planner_router.h"
#include "rlp_planner/safety_checker.h"
#include "rlp_planner/search_planner.h"
#include "rlp_road/road_model.h"

namespace rlp {
namespace planner {

class PlannerCore {
 public:
  explicit PlannerCore(const PlannerParams& params);

  // 以 params.plan_freq 的固定周期调用；内部维护跨周期状态
  // （道路模型记忆、上一周期路径、模式滞回）。线程模型：单线程调用。
  PlanResult plan(const PlannerInput& in);

 private:
  PlanResult makeStopResult(PlanMode mode, road::BoundaryState st,
                            const std::string& method, const std::string& reason) const;

  PlannerParams p_;
  road::RoadModel road_;
  // 三种规划方法（注意声明顺序：先方法后路由，路由持有方法指针）
  FollowPlanner follow_;
  SearchPlanner search_;
  FreePlanner free_;
  PlannerRouter router_;
  CostEvaluator cost_;
  SafetyChecker safety_;
  Path prev_path_;  // 上一周期选定路径（一致性代价 / warm start）
};

}  // namespace planner
}  // namespace rlp
