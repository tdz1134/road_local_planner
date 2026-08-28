#include "rlp_planner/free_planner.h"

#include "rlp_planner/candidate_gen.h"

namespace rlp {
namespace planner {

std::vector<Path> FreePlanner::candidates(const PlanningContext& ctx) const {
  // 无走廊退化：只用终点方向扇形族
  if (!ctx.input->goal_valid) return {};
  const double la = candidate_gen::lookaheadLength(p_, ctx.input->current_speed);
  return candidate_gen::goalFan(p_, ctx.input->goal, la);
}

}  // namespace planner
}  // namespace rlp
