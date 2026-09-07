#include "rlp_road/road_model.h"

namespace rlp {
namespace road {

RoadModel::RoadModel(const PlannerParams& p) : bm_(p), cb_(p) {}

void RoadModel::update(const BoundarySet& b, double now) {
  bm_.update(b, now);
  state_ = bm_.state(now);
  corridor_ = cb_.build(bm_.latest(), state_, bm_.halfWidth(), bm_.dataAge(now));
}

}  // namespace road
}  // namespace rlp
