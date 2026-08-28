#include "rlp_planner/algorithms/search_hybrid.h"

#include "rlp_planner/candidate_gen.h"

namespace rlp {
namespace planner {

std::vector<Path> SearchHybridAlg::candidates(const PlanningContext& ctx) const {
  const double la = candidate_gen::lookaheadLength(p_, ctx.input->current_speed);
  std::vector<Path> out;
  // 路面优先：走廊偏移族（若走廊有效）
  if (ctx.corridor != nullptr && ctx.corridor->valid()) {
    out = candidate_gen::corridorFamily(p_, *ctx.corridor, la);
  }
  // 终点引导：朝终点扇形族（终点在图外 → 虚拟目标点方向）
  if (ctx.input->goal_valid) {
    auto fan = candidate_gen::goalFan(p_, ctx.input->goal, la);
    out.insert(out.end(), fan.begin(), fan.end());
  }
  return out;
}

}  // namespace planner
}  // namespace rlp
