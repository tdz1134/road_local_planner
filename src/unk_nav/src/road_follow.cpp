#include "unk_nav/road_follow.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "unk_nav/geom_util.h"

namespace unk {
namespace road {

// ── 内部辅助（仅本文件使用）──────────────────────────────────────
namespace {

constexpr int kMaxHops = 16;     // hops 数组容量，也是 chain_hops 的硬上限
constexpr double kMaxTangentCorr = kPi / 6.0;  // 切向修正量 clamp ±30°（θ₂'/2 的自然上限是 37.5°）

// 单跳扇形扫描结果
struct ScanHit {
  bool found = false;
  double rel_bearing = 0.0;  // 获胜射线相对扫描朝向的方位角 rad
  double reach = 0.0;        // 获胜射线自由距离 m
};

// 通用扇形扫描：以 (ox,oy) 为原点、heading 为朝向，在前向半球 [-half,+half] 撒射线
// 量自由距离并打分。打分/步长/硬门槛语义与历史单跳版本完全一致；
// ox=oy=0、heading=0 时即原 lookAhead 的扫描。
// goal_bearing 非 NAN 时额外加 goal_align_w × cos(射线绝对方向 − goal_bearing)，
// 用于终点模式偏向子目标方向。沿路模式传 NAN 不加该项。
// prev_bearing 非空时额外加 road_prev_w × cos(射线绝对方向 − prev_bearing)，
// 用于沿路第 1 跳偏好上帧方向（迟滞防抖）；接力跳传 nullptr。
// cands != nullptr 时收集候选（bearing 记绝对角 = heading + th），并标记 selected。
ScanHit scanFan(const GridMap& g, double ox, double oy, double heading,
                const NavParams& p, double L, double step,
                std::vector<NavResult::FanCandidate>* cands,
                double goal_bearing = std::numeric_limits<double>::quiet_NaN(),
                const double* prev_bearing = nullptr) {
  ScanHit hit;
  const double half = p.road_fan_half_deg * kPi / 180.0;
  const double dstep = std::max(p.road_fan_step_deg, 1e-3) * kPi / 180.0;
  if (half < 0.0 || L <= 1e-6) return hit;

  // 用整数索引遍历 [-half, +half]，避免浮点累加漂移；n_steps 为偶数时必含 θ=0（扫描朝向）。
  const int n_steps = static_cast<int>(std::floor(2.0 * half / dstep + 0.5));
  const double ch = std::cos(heading), sh = std::sin(heading);
  const bool has_goal = !std::isnan(goal_bearing);

  double best_score = -1e18;
  int best_idx = -1;
  if (cands) cands->reserve(n_steps + 1);

  for (int i = 0; i <= n_steps; ++i) {
    const double th = -half + i * dstep;
    // 绝对方向 = heading + th（旋转到 base 系）
    const double ct = std::cos(th), st = std::sin(th);
    const double ux = ch * ct - sh * st;
    const double uy = sh * ct + ch * st;

    // 沿射线量"前方自由距离" d：从 step 起逐点查 feasibleAt，遇首个不可行即停在其前一格；
    // 全程可行则 d = L。feasibleAt 对 occupied/越界返回 false，对 free/unknown 返回 true
    //（乐观放行）。
    double d = L;
    for (double t = step; t <= L; t += step) {
      if (!g.feasibleAt(ox + ux * t, oy + uy * t)) {
        d = t - step;
        break;
      }
    }
    if (d < 0.0) d = 0.0;

    // 打分 = 自由距离（归一化到 [0,1]）× 权重 + 扫描朝向对齐 cosθ × 权重
    //       + 终点模式子目标偏向 goal_align_w × cos(绝对方向 − goal_bearing)。
    const double free_term = std::min(d, L) / L;
    double score = p.road_free_w * free_term + p.road_align_w * std::cos(th);
    if (has_goal) {
      const double abs_dir = geom::normalizeAngle(heading + th);
      score += p.goal_align_w * std::cos(geom::normalizeAngle(abs_dir - goal_bearing));
    }
    // 上帧方向偏好（迟滞）：边际平分时保持 winner 稳定；方向真变（障碍/弯道推进）时
    // free 项差异压过本项，不会被粘住。幅度分析同 subgoal_prev_w。
    if (prev_bearing) {
      score += p.road_prev_w * std::cos(geom::normalizeAngle(heading + th - *prev_bearing));
    }

    if (score > best_score) {
      best_score = score;
      hit.rel_bearing = th;
      hit.reach = d;
      hit.found = true;
      if (cands) best_idx = static_cast<int>(cands->size());
    }
    if (cands) {
      NavResult::FanCandidate cand;
      cand.bearing = geom::normalizeAngle(heading + th);
      cand.d_free = d;
      cand.reach = d;  // 沿路落点无净空回退，reach 即自由距离
      cand.score = score;
      cand.feasible = (d >= step);  // 硬门槛：至少一格可行落点
      cands->push_back(cand);
    }
  }

  // 标记选中候选（即使随后因 reach < step 判为无效，选中关系仍有调试价值）
  if (cands && best_idx >= 0 && best_idx < static_cast<int>(cands->size())) {
    (*cands)[best_idx].selected = true;
  }
  return hit;
}

}  // namespace

// ── 公共接口（在 road_follow.h 中声明）───────────────────────────

Result lookAhead(const GridMap& work_grid, const NavParams& p, double current_speed) {
  // 单跳扫描：子目标 = 看多深 L 的满距离落点（不受 road_step_ratio 截断）。
  // 与 lookAheadChain 的区别：不截断第一跳距离、不做接力扫描。
  // 保留独立实现（而非委托 lookAheadChain）：lookAheadChain 会用 step_walk 截断第一跳，
  // 而 lookAhead 的语义是"扫满 L、落点在射线末端"，两者在 road_step_ratio<1 时不同。
  Result r;
  if (work_grid.empty()) return r;

  const double L = p.roadLookahead(current_speed);
  if (L <= 1e-6) return r;
  const double step = std::max(0.5 * work_grid.resolution, 1e-6);

  const ScanHit hit = scanFan(work_grid, 0.0, 0.0, 0.0, p, L, step, &r.candidates);
  if (!hit.found) return r;

  r.bearing = hit.rel_bearing;
  r.reach = hit.reach;
  if (r.reach < step) { r.reach = 0.0; return r; }
  r.truncated_by_obstacle = r.reach < L - 1e-9;
  r.point = Point2D{std::cos(r.bearing) * r.reach, std::sin(r.bearing) * r.reach};
  r.valid = true;
  r.tangent_end = r.bearing;
  r.kappa_est = 0.0;
  r.hops[0] = r.point;
  r.hop_count = 1;
  return r;
}

Result lookAheadChain(const GridMap& work_grid, const NavParams& p,
                      double start_heading, double goal_bearing, double current_speed,
                      const double* prev_hop1_bearing) {
  // 第 1 跳扫描：从 (0,0) 朝 start_heading 方向扫看多深 L，选最优方向。
  // 沿路模式 start_heading=0（车头）；终点模式可传 sg.bearing 偏朝子目标。
  // goal_bearing 非 NAN 时 scanFan 打分额外加 goal_align_w 子目标偏向项。
  Result r;
  if (work_grid.empty()) return r;
  const double L = p.roadLookahead(current_speed);
  if (L <= 1e-6) return r;
  const double step = std::max(0.5 * work_grid.resolution, 1e-6);

  const ScanHit first_hit = scanFan(work_grid, 0.0, 0.0, start_heading, p, L, step,
                                     &r.candidates, goal_bearing, prev_hop1_bearing);
  if (!first_hit.found) return r;
  r.bearing = geom::normalizeAngle(start_heading + first_hit.rel_bearing);
  r.reach = first_hit.reach;
  if (r.reach < step) { r.reach = 0.0; return r; }
  r.truncated_by_obstacle = r.reach < L - 1e-9;
  r.valid = true;
  r.tangent_end = r.bearing;
  r.kappa_est = 0.0;

  const int hops = std::min(std::max(p.chain_hops, 1), kMaxHops);

  // 走多近 = min(固定米, 比例 × 看多深)。默认 ratio=1.0 → 走多近=L → 落点=射线末端（旧行为）。
  // "看多深"(L) 只用于 scanFan 选方向防短视；落点位置由"走多近"截断，遇障碍再取 min。
  const double step_walk = std::min(p.road_step_dist, p.road_step_ratio * L);

  // 第一跳落点 P1：距离 = min(走多近, 扫描自由距离)。
  const double first_fwd = std::min(step_walk, r.reach);
  r.point = Point2D{std::cos(r.bearing) * first_fwd, std::sin(r.bearing) * first_fwd};
  r.hops[0] = r.point;
  r.hop_count = 1;

  // 接力扫描：每跳把"虚拟车"放在上一跳落点、朝向 = 到达方向（绝对角），再扫看多深选方向。
  // goal_bearing 透传给每跳 scanFan 的打分（终点模式偏向子目标，沿路模式 NAN 不生效）。
  double psi = r.bearing;
  Point2D cur = r.point;
  double prev_fwd = first_fwd;
  double dpsi_total = 0.0;
  double ds_total = 0.0;
  double first_deflect = 0.0;
  bool have_deflect = false;

  for (int h = 1; h < hops; ++h) {
    const ScanHit hit = scanFan(work_grid, cur.x, cur.y, psi, p, L, step, nullptr, goal_bearing);
    // 链截断：落点处无可行前向（封死/贴窗口边缘）→ 用已有跳数继续，不报错
    if (!hit.found || hit.reach < step) break;

    if (!have_deflect) {
      first_deflect = hit.rel_bearing;
      have_deflect = true;
    }
    dpsi_total += hit.rel_bearing;
    ds_total += prev_fwd;

    const double abs_b = geom::normalizeAngle(psi + hit.rel_bearing);
    const double fwd = std::min(step_walk, hit.reach);  // 走多近，遇墙提前停
    cur = Point2D{cur.x + std::cos(abs_b) * fwd,
                  cur.y + std::sin(abs_b) * fwd};
    psi = abs_b;
    prev_fwd = fwd;
    if (r.hop_count < kMaxHops) r.hops[r.hop_count++] = cur;
  }

  if (have_deflect) {
    // 终点切向 = 弦向 + 首个偏角的一半；修正量 clamp ±30° 防扇形量化噪声放大
    const double corr = std::min(std::max(first_deflect * 0.5, -kMaxTangentCorr),
                                 kMaxTangentCorr);
    r.tangent_end = geom::normalizeAngle(r.bearing + corr);
    if (ds_total > 1e-6) r.kappa_est = dpsi_total / ds_total;
  }
  return r;
}

}  // namespace road
}  // namespace unk
