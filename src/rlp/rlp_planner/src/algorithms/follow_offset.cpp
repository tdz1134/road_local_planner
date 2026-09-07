#include "rlp_planner/algorithms/follow_offset.h"

#include "rlp_planner/candidate_gen.h"

namespace rlp {
namespace planner {

std::vector<Path> FollowOffsetAlg::candidates(const PlanningContext& ctx) const {
  // 走廊不可用 → 本算法不适用（返回空，由路由/上层处理）
  if (ctx.corridor == nullptr || !ctx.corridor->valid()) return {};
  const double la = candidate_gen::lookaheadLength(p_, ctx.input->current_speed);
  return candidate_gen::corridorFamily(p_, *ctx.corridor, la);
}

}  // namespace planner
}  // namespace rlp
