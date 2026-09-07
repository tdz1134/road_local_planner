#pragma once
// 走廊构建：由（可能是单侧的）边界点列构建中线 + 半宽。
// v1 最简单实现：双侧取中、单侧按历史路宽平移补全、双侧缺失沿用上一帧并膨胀。
// TODO(后续): 样条拟合、鲁棒剔点（边界误差）、逐点距离场、曲率感知外推。
#include "rlp_road/road_types.h"

namespace rlp {
namespace road {

class CorridorBuilder {
 public:
  explicit CorridorBuilder(const PlannerParams& p);

  // b/state/half_width_hint 来自 BoundaryManager；
  // missing_age：双侧缺失已持续的秒数（仅 MISSING_SHORT 时使用）
  Corridor build(const BoundarySet& b, BoundaryState state,
                 double half_width_hint, double missing_age);

 private:
  PlannerParams p_;
  Corridor last_;  // 走廊记忆
};

}  // namespace road
}  // namespace rlp
