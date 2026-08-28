#pragma once
// 边界数据管理：有效性判定（每侧独立超时）、路宽历史估计、可用性状态机。
// 处理四种情况：双侧都有 / 仅一侧 / 双侧短时缺失（记忆）/ 双侧超时缺失。
#include "rlp_road/road_types.h"

namespace rlp {
namespace road {

class BoundaryManager {
 public:
  explicit BoundaryManager(const PlannerParams& p);

  // 收到新边界帧时调用；空的一侧不覆盖旧数据（视为本帧缺失）
  void update(const BoundarySet& b, double now);

  // 当前可用性状态（含超时判定）
  BoundaryState state(double now) const;

  // 最近一次有效的左右边界点列（缺失侧可能为空或为旧数据，配合 state 使用）
  const BoundarySet& latest() const { return latest_; }

  // 双侧都有时的路宽 EMA 估计（半宽），供单侧缺失时补全走廊；无历史返回 0
  double halfWidth() const { return half_width_ema_; }

  // 距最近一次任一侧更新的秒数（用于走廊外推的不确定性计算）
  double dataAge(double now) const;

 private:
  void updateWidthEma();

  PlannerParams p_;
  BoundarySet latest_;
  double left_stamp_ = -1e9;
  double right_stamp_ = -1e9;
  double half_width_ema_ = 0.0;
};

}  // namespace road
}  // namespace rlp
