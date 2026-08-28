#include "rlp_planner/planner_core.h"

#include <algorithm>

namespace rlp {
namespace planner {

PlannerCore::PlannerCore(const PlannerParams& params)
    : p_(params),
      road_(params),
      follow_(params),
      search_(params),
      free_(params),
      router_(params, &follow_, &search_, &free_),
      cost_(params),
      safety_(params) {}

PlanResult PlannerCore::makeStopResult(PlanMode mode, road::BoundaryState st,
                                       const std::string& method,
                                       const std::string& reason) const {
  PlanResult r;
  r.mode = mode;
  r.boundary_state = st;
  r.recommended_speed = 0.0;
  r.emergency_stop = true;
  r.method = method;
  r.reason = reason;
  return r;
}

PlanResult PlannerCore::plan(const PlannerInput& in) {
  // ---- 1. 道路模型更新（边界可用性 + 走廊建模）----
  road_.update(in.boundaries, in.now);
  const road::BoundaryState st = road_.state();
  const road::Corridor& corridor = road_.corridor();
  const bool road_valid = corridor.valid();

  // ---- 2. 路由：定位质量 + 道路情况 → 规划方法 ----
  const PlannerBase* method =
      router_.select(in.localization_quality, in.goal_valid, road_valid, in.now);
  if (method == nullptr) {
    // 无走廊且定位差：无任何可用方法，停车保护
    return makeStopResult(router_.mode(), st, "none",
                          "no corridor and localization poor");
  }

  PlanResult result;
  result.mode = method->mode();
  result.boundary_state = st;
  result.corridor_confidence = corridor.confidence;
  result.method = method->name();

  // ---- 3. 方法生成候选 + 代价评分 ----
  const double margin =
      p_.boundary_margin_base + p_.boundary_margin_speed_gain * in.current_speed;
  PlanningContext ctx;
  ctx.input = &in;
  ctx.corridor = road_valid ? &corridor : nullptr;
  ctx.margin = margin;
  ctx.prev_path = &prev_path_;

  double best_cost = 1e18;
  Path best_path;
  for (const auto& c : method->candidates(ctx)) {
    const double cst = cost_.evaluate(c, in.map, corridor, method->mode(), in.goal,
                                      in.goal_valid, prev_path_, margin);
    if (cst < best_cost) {
      best_cost = cst;
      best_path = c;
    }
  }

  // 全部候选碰撞/无效 → 急停
  if (best_path.empty() || best_cost >= p_.collision_cost * 0.5) {
    return makeStopResult(method->mode(), st, method->name(),
                          "no feasible candidate (blocked)");
  }

  // ---- 4. 安全校验 + 推荐速度 ----
  const auto safety = safety_.check(best_path, in.map, in.current_speed);
  if (!safety.pass) {
    result = makeStopResult(method->mode(), st, method->name(),
                            "braking envelope violated");
    result.path = best_path;
    result.corridor_confidence = corridor.confidence;
    return result;
  }
  // 走廊置信度低时额外限速（v1 线性保守处理）
  const double conf_cap = 0.4 + 0.6 * std::max(0.0, std::min(1.0, corridor.confidence));
  result.path = best_path;
  result.recommended_speed = std::min(safety.max_safe_speed, p_.v_max * conf_cap);
  result.emergency_stop = false;
  result.reason = "ok";

  // ---- 5. 跨周期状态交接 ----
  prev_path_ = best_path;
  return result;
}

}  // namespace planner
}  // namespace rlp
