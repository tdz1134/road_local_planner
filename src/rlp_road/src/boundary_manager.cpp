#include "rlp_road/boundary_manager.h"

#include <algorithm>
#include <cmath>

namespace rlp {
namespace road {

// ── 内部辅助（仅本文件使用，不对外暴露）──────────────────────────
namespace {
constexpr double kEmaAlpha = 0.5;  // 路宽平滑系数，TODO: 提为参数
}

// ── 公共接口（在 boundary_manager.h 中声明）──────────────────────

BoundaryManager::BoundaryManager(const PlannerParams& p) : p_(p) {}

void BoundaryManager::update(const BoundarySet& b, double now) {
  // 空的一侧不覆盖旧数据：本帧缺失 ≠ 立即失效，由超时判定
  if (!b.left.empty()) {
    latest_.left = b.left;
    left_stamp_ = now;
  }
  if (!b.right.empty()) {
    latest_.right = b.right;
    right_stamp_ = now;
  }
  latest_.stamp = std::max(left_stamp_, right_stamp_);
  if (state(now) == BoundaryState::BOTH) {
    updateWidthEma();
  }
}

BoundaryState BoundaryManager::state(double now) const {
  const bool l_ok = !latest_.left.empty() && (now - left_stamp_) <= p_.boundary_timeout;
  const bool r_ok = !latest_.right.empty() && (now - right_stamp_) <= p_.boundary_timeout;
  if (l_ok && r_ok) return BoundaryState::BOTH;
  if (l_ok) return BoundaryState::LEFT_ONLY;
  if (r_ok) return BoundaryState::RIGHT_ONLY;
  return dataAge(now) <= p_.corridor_hold_max ? BoundaryState::MISSING_SHORT
                                              : BoundaryState::MISSING_TIMEOUT;
}

double BoundaryManager::dataAge(double now) const {
  return now - std::max(left_stamp_, right_stamp_);
}

// v1 最简路宽估计：对左边界每个点找右边界最近点，取距离均值的一半。
// TODO(后续): 按弧长匹配 + 鲁棒统计（中位数/剔外点）应对边界误差。
void BoundaryManager::updateWidthEma() {
  double sum = 0.0;
  int cnt = 0;
  for (const auto& lp : latest_.left) {
    double best = 1e9;
    for (const auto& rp : latest_.right) {
      best = std::min(best, std::hypot(lp.x - rp.x, lp.y - rp.y));
    }
    if (best < 1e9) {
      sum += best;
      ++cnt;
    }
  }
  if (cnt == 0) return;
  double half = 0.5 * sum / cnt;
  half = std::max(p_.width_min * 0.5, std::min(p_.width_max * 0.5, half));
  half_width_ema_ =
      half_width_ema_ <= 0.0 ? half : (1.0 - kEmaAlpha) * half_width_ema_ + kEmaAlpha * half;
}

}  // namespace road
}  // namespace rlp
