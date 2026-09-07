#pragma once
// 规划方法路由：定位质量 → 模式（双阈值滞回 + 驻留时间），
// 模式 + 道路情况 → 具体规划方法。路由表见 planner_base.h 顶部注释。
#include "rlp_planner/planner_base.h"

namespace rlp {
namespace planner {

class PlannerRouter {
 public:
  PlannerRouter(const PlannerParams& p, const PlannerBase* follow,
                const PlannerBase* search, const PlannerBase* free);

  // 返回当前应使用的规划方法；返回 nullptr 表示无可行方法（上层停车）
  const PlannerBase* select(double quality, bool goal_valid, bool road_valid, double now);

  PlanMode mode() const { return mode_; }

 private:
  PlannerParams p_;
  const PlannerBase* follow_;
  const PlannerBase* search_;
  const PlannerBase* free_;
  PlanMode mode_ = PlanMode::FOLLOW;
  double last_switch_ = -1e9;
};

}  // namespace planner
}  // namespace rlp
