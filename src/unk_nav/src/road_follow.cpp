#include "unk_nav/road_follow.h"

#include <algorithm>
#include <cmath>

namespace unk {
namespace road {

// ── 公共接口（在 road_follow.h 中声明）───────────────────────────

Result lookAhead(const GridMap& work_grid, const NavParams& p) {
  Result r;
  if (work_grid.empty()) return r;

  const double L = p.roadLookahead();
  if (L <= 1e-6) return r;

  // 射线采样步长：半格。与 subgoal 的落点截断同尺度，足够命中边界且不超一格误差。
  const double step = std::max(0.5 * work_grid.resolution, 1e-6);

  const double half = p.road_fan_half_deg * kPi / 180.0;
  const double dstep = std::max(p.road_fan_step_deg, 1e-3) * kPi / 180.0;
  if (half < 0.0) return r;

  // 用整数索引遍历 [-half, +half]，避免浮点累加漂移；n_steps 为偶数时必含 θ=0（正前方）。
  const int n_steps = static_cast<int>(std::floor(2.0 * half / dstep + 0.5));

  double best_score = -1e18;
  bool found = false;

  for (int i = 0; i <= n_steps; ++i) {
    const double th = -half + i * dstep;
    const double ux = std::cos(th);
    const double uy = std::sin(th);

    // 沿射线量"前方自由距离" d：从 step 起逐点查 feasibleAt，遇首个不可行即停在其前一格；
    // 全程可行则 d = L。feasibleAt 对 occupied/越界返回 false，对 free/unknown 返回 true
    // （乐观放行，与 A* 一致）。
    double d = L;
    for (double t = step; t <= L; t += step) {
      if (!work_grid.feasibleAt(ux * t, uy * t)) {
        d = t - step;
        break;
      }
    }
    if (d < 0.0) d = 0.0;

    // 打分 = 自由距离（归一化到 [0,1]）× 权重 + 车头对齐 cosθ × 权重。
    const double free_term = std::min(d, L) / L;
    const double score = p.road_free_w * free_term + p.road_align_w * std::cos(th);
    if (score > best_score) {
      best_score = score;
      r.bearing = th;
      r.reach = d;
      found = true;
    }
  }

  if (!found) return r;

  // 选中方向连一格可行落点都没有（车被走廊尽头/障碍围死）→ 无前向可走。
  // 交由上层：nav_core 得不到子目标 → checkPlanFail → RECOVERY → ABORT。
  if (r.reach < step) {
    r.reach = 0.0;
    return r;  // valid 保持 false
  }

  // 落点已在射线行进中保证可行（d 是首个不可行点之前最后一格），无需再回退。
  // reach < L 说明射线被障碍/膨胀带截短 → 标记 truncated_by_obstacle（与 subgoal 语义一致）。
  r.truncated_by_obstacle = r.reach < L - 1e-9;
  r.point = Point2D{std::cos(r.bearing) * r.reach, std::sin(r.bearing) * r.reach};
  r.valid = true;
  return r;
}

}  // namespace road
}  // namespace unk
