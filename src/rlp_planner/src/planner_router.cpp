#include "rlp_planner/planner_router.h"

namespace rlp {
namespace planner {

PlannerRouter::PlannerRouter(const PlannerParams& p, const PlannerBase* follow,
                             const PlannerBase* search, const PlannerBase* free)
    : p_(p), follow_(follow), search_(search), free_(free) {}

const PlannerBase* PlannerRouter::select(double quality, bool goal_valid,
                                         bool road_valid, double now) {
  // ---- 1) 定位质量 → 模式（双阈值滞回 + 最小驻留时间）----
  PlanMode target = mode_;
  if (goal_valid && quality >= p_.q_high) {
    target = PlanMode::SEARCH;
  } else if (quality <= p_.q_low || !goal_valid) {
    target = PlanMode::FOLLOW;
  }
  if (target != mode_ && now - last_switch_ >= p_.mode_dwell) {
    mode_ = target;
    last_switch_ = now;
  }

  // ---- 2) 模式 + 道路情况 → 具体规划方法 ----
  if (mode_ == PlanMode::FOLLOW) {
    // 定位差：只能靠路。无路则无方法可用 → 停车
    return road_valid ? follow_ : nullptr;
  }
  // SEARCH：有路走"走廊+终点"，无路但定位好 → 纯终点方向推进
  if (road_valid) return search_;
  return goal_valid ? free_ : nullptr;
}

}  // namespace planner
}  // namespace rlp
