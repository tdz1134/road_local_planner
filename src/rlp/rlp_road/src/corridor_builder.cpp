#include "rlp_road/corridor_builder.h"

#include <algorithm>
#include <cmath>

#include "rlp_common/geometry_util.h"

namespace rlp {
namespace road {

CorridorBuilder::CorridorBuilder(const PlannerParams& p) : p_(p) {}

Corridor CorridorBuilder::build(const BoundarySet& b, BoundaryState state,
                                double half_width_hint, double missing_age) {
  Corridor out;
  out.state = state;

  const double hw_limit_max = p_.width_max * 0.5;

  switch (state) {
    case BoundaryState::BOTH: {
      // 双侧都有：重采样后逐点取中
      const auto l = resampleByArc(b.left, p_.path_spacing);
      const auto r = resampleByArc(b.right, p_.path_spacing);
      const size_t n = std::min(l.size(), r.size());
      double hw_sum = 0.0;
      for (size_t i = 0; i < n; ++i) {
        out.centerline.push_back(
            {(l[i].x + r[i].x) * 0.5, (l[i].y + r[i].y) * 0.5});
        hw_sum += std::fabs(l[i].y - r[i].y) * 0.5;
      }
      out.half_width =
          std::max(p_.width_min * 0.5, std::min(hw_sum / std::max(n, size_t(1)), hw_limit_max));
      out.confidence = 1.0;
      break;
    }
    case BoundaryState::LEFT_ONLY:
    case BoundaryState::RIGHT_ONLY: {
      // 单侧：用历史路宽把已知侧平移半宽合成中线。
      // v1 假设道路大致沿车体 x 向、边界点按 x 递增，直接平移 y。
      const double hw = std::max(p_.width_min * 0.5,
                                 std::min(half_width_hint > 0 ? half_width_hint
                                                              : p_.width_min * 0.5,
                                          hw_limit_max));
      const auto& known = (state == BoundaryState::LEFT_ONLY) ? b.left : b.right;
      const double sign = (state == BoundaryState::LEFT_ONLY) ? -1.0 : 1.0;
      auto cl = resampleByArc(known, p_.path_spacing);
      for (auto& pt : cl) pt.y += sign * hw;
      out.centerline = cl;
      out.half_width = hw;
      out.confidence = 0.6;  // 单侧：置信度降级
      break;
    }
    case BoundaryState::MISSING_SHORT: {
      // 双侧短时缺失：沿用上一帧中线，半宽按不确定性随时间膨胀
      if (!last_.valid()) return last_;  // 从未有过走廊
      out.centerline = last_.centerline;
      out.half_width = last_.half_width + p_.corridor_inflate_rate * missing_age;
      out.confidence = 0.3 * std::exp(-missing_age);
      break;
    }
    case BoundaryState::MISSING_TIMEOUT:
    default:
      return Corridor();  // 空走廊，上层退化处理
  }

  last_ = out;
  return out;
}

}  // namespace road
}  // namespace rlp
