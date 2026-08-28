#pragma once
// 道路模型门面：聚合"边界管理 + 走廊构建"，规划层只与本类交互。
// 用法：每周期 road.update(boundaries, now) 后，读 state()/corridor()。
#include "rlp_road/boundary_manager.h"
#include "rlp_road/corridor_builder.h"

namespace rlp {
namespace road {

class RoadModel {
 public:
  explicit RoadModel(const PlannerParams& p);

  void update(const BoundarySet& b, double now);

  BoundaryState state() const { return state_; }
  const Corridor& corridor() const { return corridor_; }

  // 距最近一次任一侧边界更新的秒数
  double dataAge(double now) const { return bm_.dataAge(now); }

 private:
  BoundaryManager bm_;
  CorridorBuilder cb_;
  BoundaryState state_ = BoundaryState::MISSING_TIMEOUT;
  Corridor corridor_;
};

}  // namespace road
}  // namespace rlp
