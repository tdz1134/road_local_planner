#pragma once
// 道路层公共类型：边界点集、边界可用性状态、走廊模型。
// "道路"相关的所有建模都集中在 rlp_road 包，规划层只消费这里的输出。
#include "rlp_common/types.h"

namespace rlp {
namespace road {

// 感知输出的左右边界（车体系点列，各约 20 个点，可能缺失）
struct BoundarySet {
  std::vector<Point2D> left;
  std::vector<Point2D> right;
  double stamp = 0.0;  // 最近一次有效更新时刻（秒）
};

// 边界可用性状态机（由 BoundaryManager 维护）
enum class BoundaryState {
  BOTH,             // 双侧有效
  LEFT_ONLY,        // 仅左边界有效
  RIGHT_ONLY,       // 仅右边界有效
  MISSING_SHORT,    // 双侧缺失，走廊记忆期内（外推+膨胀+降置信度）
  MISSING_TIMEOUT,  // 双侧缺失超时，走廊不可信
};

inline const char* boundaryStateName(BoundaryState s) {
  switch (s) {
    case BoundaryState::BOTH: return "BOTH";
    case BoundaryState::LEFT_ONLY: return "LEFT_ONLY";
    case BoundaryState::RIGHT_ONLY: return "RIGHT_ONLY";
    case BoundaryState::MISSING_SHORT: return "MISSING_SHORT";
    case BoundaryState::MISSING_TIMEOUT: return "MISSING_TIMEOUT";
  }
  return "UNKNOWN";
}

// 道路走廊：中线 + 半宽 + 置信度。这是道路层对外的唯一产物。
struct Corridor {
  std::vector<Point2D> centerline;
  double half_width = 0.0;
  BoundaryState state = BoundaryState::MISSING_TIMEOUT;
  double confidence = 0.0;  // [0,1]

  bool valid() const { return centerline.size() >= 2 && half_width > 0.1; }
};

}  // namespace road
}  // namespace rlp
