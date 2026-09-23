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

// ── Top-K+承诺内部辅助：从种子方向生长完整链 ──────────────────────
struct GrownChain {
  Point2D hops[16];
  double bearings[16];   // 每跳的绝对 bearing
  int    hop_count = 0;
  double total_score = 0.0;    // Σ per-hop scan scores
  double progress = 0.0;       // 末跳在 start_heading 上的投影
  double first_bearing = 0.0;  // hop-1 绝对方向
};

GrownChain growChain(const GridMap& g, const NavParams& p,
                     double start_heading, double seed_rel, double seed_score,
                     double seed_reach, double L, double step, double goal_bearing) {
  GrownChain chain;
  const double abs1 = geom::normalizeAngle(start_heading + seed_rel);
  chain.first_bearing = abs1;
  chain.bearings[0] = abs1;

  const double step_walk = std::min(p.road_step_dist, p.road_step_ratio * L);
  const double first_fwd = std::min(step_walk, seed_reach);
  chain.hops[0] = Point2D{std::cos(abs1) * first_fwd, std::sin(abs1) * first_fwd};
  chain.hop_count = 1;
  chain.total_score = seed_score;

  const int hops = std::min(std::max(p.chain_hops, 1), kMaxHops);
  double psi = abs1;
  Point2D cur = chain.hops[0];

  for (int h = 1; h < hops; ++h) {
    ScanHit hit = scanFan(g, cur.x, cur.y, psi, p, L, step, nullptr, goal_bearing);
    if (!hit.found || hit.reach < step) break;
    const double abs_b = geom::normalizeAngle(psi + hit.rel_bearing);
    const double fwd = std::min(step_walk, hit.reach);
    cur = Point2D{cur.x + std::cos(abs_b) * fwd, cur.y + std::sin(abs_b) * fwd};
    psi = abs_b;
    if (chain.hop_count < kMaxHops) {
      chain.bearings[chain.hop_count] = abs_b;
      chain.hops[chain.hop_count] = cur;
      ++chain.hop_count;
    }
    chain.total_score += p.road_free_w * std::min(hit.reach, L) / L +
                         p.road_align_w * std::cos(hit.rel_bearing);
  }

  // progress: 末跳在 start_heading 方向的投影距离
  chain.progress = std::cos(chain.bearings[0] - start_heading) * std::hypot(cur.x, cur.y);
  return chain;
}

}  // namespace

// ── 公共接口（在 road_follow.h 中声明）───────────────────────────

Result lookAhead(const GridMap& work_grid, const NavParams& p, double current_speed) {
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
                      const double* prev_hop1_bearing, ChainCommitState* commit) {
  Result r;
  if (work_grid.empty()) return r;
  const double L = p.roadLookahead(current_speed);
  if (L <= 1e-6) return r;
  const double step = std::max(0.5 * work_grid.resolution, 1e-6);

  // === Top-K+承诺 分支（road_topk>1 且 commit 非空） ===
  if (p.road_topk > 1 && commit != nullptr) {
    // 1) 扫描候选（不加 prev_bearing，链级一致性在评分时处理）
    std::vector<NavResult::FanCandidate> cands;
    const ScanHit first_hit = scanFan(work_grid, 0.0, 0.0, start_heading, p, L, step,
                                       &cands, goal_bearing, nullptr);
    r.candidates = std::move(cands);
    if (!first_hit.found || r.candidates.empty()) {
      commit->valid = false;
      return r;
    }

    // 2) 取 Top-K 种子（按分数降序，角度间距 ≥15° 去重）
    constexpr double kSeedSepRad = 15.0 * kPi / 180.0;
    std::vector<int> order(static_cast<int>(r.candidates.size()));
    for (int i = 0; i < static_cast<int>(order.size()); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
      return r.candidates[a].score > r.candidates[b].score;
    });

    std::vector<int> seeds;
    seeds.reserve(p.road_topk);
    for (int idx : order) {
      if (static_cast<int>(seeds.size()) >= p.road_topk) break;
      if (!r.candidates[idx].feasible) continue;
      bool too_close = false;
      for (int s : seeds) {
        if (std::fabs(geom::normalizeAngle(
                r.candidates[idx].bearing - r.candidates[s].bearing)) < kSeedSepRad) {
          too_close = true;
          break;
        }
      }
      if (!too_close) seeds.push_back(idx);
    }
    if (seeds.empty()) {
      commit->valid = false;
      r.bearing = geom::normalizeAngle(start_heading + first_hit.rel_bearing);
      r.reach = first_hit.reach;
      if (r.reach < step) { r.reach = 0.0; return r; }
      const double fwd = std::min(p.road_step_dist, p.road_step_ratio * L);
      r.point = Point2D{std::cos(r.bearing) * fwd, std::sin(r.bearing) * fwd};
      r.valid = true; r.hops[0] = r.point; r.hop_count = 1;
      r.tangent_end = r.bearing;
      return r;
    }

    // 3) 每个种子生长一条链
    std::vector<GrownChain> chains;
    chains.reserve(seeds.size());
    for (int si : seeds) {
      const auto& cand = r.candidates[si];
      const double rel = geom::normalizeAngle(cand.bearing - start_heading);
      chains.push_back(growChain(work_grid, p, start_heading, rel, cand.score,
                                  cand.reach, L, step, goal_bearing));
    }

    // 4) 链级评分：total_score + consistency(road_prev_w × Σcos(hop−committed))
    std::vector<double> final_scores(chains.size(), 0.0);
    int incumbent_idx = -1;
    constexpr double kIncumbentThresh = kSeedSepRad;

    for (size_t i = 0; i < chains.size(); ++i) {
      final_scores[i] = chains[i].total_score;
      if (commit->valid && commit->hop_count > 0) {
        double consistency = 0.0;
        const int n = std::min(chains[i].hop_count, commit->hop_count);
        for (int h = 0; h < n; ++h) {
          consistency += p.road_prev_w *
              std::cos(geom::normalizeAngle(chains[i].bearings[h] - commit->bearings[h]));
        }
        final_scores[i] += consistency;
        if (std::fabs(geom::normalizeAngle(chains[i].first_bearing - commit->bearings[0]))
            < kIncumbentThresh) {
          incumbent_idx = static_cast<int>(i);
        }
      }
    }

    // 5) 找 best + 承诺规则
    int best_idx = 0;
    for (int i = 1; i < static_cast<int>(final_scores.size()); ++i) {
      if (final_scores[i] > final_scores[best_idx]) best_idx = i;
    }

    int chosen_idx = best_idx;
    if (commit->valid && incumbent_idx >= 0 && incumbent_idx != best_idx) {
      const double threshold = final_scores[incumbent_idx] * (1.0 + p.road_commit_margin);
      if (final_scores[best_idx] < threshold) {
        chosen_idx = incumbent_idx;  // 保持承诺
      }
    }

    // 6) 填充 Result
    const GrownChain& win = chains[chosen_idx];
    r.bearing = win.bearings[0];
    r.reach = std::hypot(win.hops[0].x, win.hops[0].y);
    r.valid = true;
    r.point = win.hops[0];
    r.hop_count = win.hop_count;
    for (int i = 0; i < win.hop_count && i < kMaxHops; ++i) r.hops[i] = win.hops[i];
    r.truncated_by_obstacle = (win.hop_count < std::min(p.chain_hops, kMaxHops));

    // tangent_end & kappa_est
    if (win.hop_count >= 2) {
      const double deflect = geom::normalizeAngle(win.bearings[1] - win.bearings[0]);
      const double corr = std::min(std::max(deflect * 0.5, -kMaxTangentCorr), kMaxTangentCorr);
      r.tangent_end = geom::normalizeAngle(win.bearings[0] + corr);
      double dpsi = 0.0, ds = 0.0;
      for (int h = 1; h < win.hop_count; ++h) {
        dpsi += geom::normalizeAngle(win.bearings[h] - win.bearings[h - 1]);
        ds += std::hypot(win.hops[h].x - win.hops[h - 1].x,
                         win.hops[h].y - win.hops[h - 1].y);
      }
      if (ds > 1e-6) r.kappa_est = dpsi / ds;
    } else {
      r.tangent_end = win.bearings[0];
    }

    // 7) 更新承诺状态
    commit->valid = true;
    commit->hop_count = win.hop_count;
    commit->score = final_scores[chosen_idx];
    for (int i = 0; i < win.hop_count && i < kMaxHops; ++i)
      commit->bearings[i] = win.bearings[i];

    return r;
  }

  // === 旧路径（road_topk<=1 或无 commit 状态）：单链 + prev_hop1_bearing 迟滞 ===
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
  const double step_walk = std::min(p.road_step_dist, p.road_step_ratio * L);

  const double first_fwd = std::min(step_walk, r.reach);
  r.point = Point2D{std::cos(r.bearing) * first_fwd, std::sin(r.bearing) * first_fwd};
  r.hops[0] = r.point;
  r.hop_count = 1;

  double psi = r.bearing;
  Point2D cur = r.point;
  double prev_fwd = first_fwd;
  double dpsi_total = 0.0;
  double ds_total = 0.0;
  double first_deflect = 0.0;
  bool have_deflect = false;

  for (int h = 1; h < hops; ++h) {
    const ScanHit hit = scanFan(work_grid, cur.x, cur.y, psi, p, L, step, nullptr, goal_bearing);
    if (!hit.found || hit.reach < step) break;

    if (!have_deflect) {
      first_deflect = hit.rel_bearing;
      have_deflect = true;
    }
    dpsi_total += hit.rel_bearing;
    ds_total += prev_fwd;

    const double abs_b = geom::normalizeAngle(psi + hit.rel_bearing);
    const double fwd = std::min(step_walk, hit.reach);
    cur = Point2D{cur.x + std::cos(abs_b) * fwd,
                  cur.y + std::sin(abs_b) * fwd};
    psi = abs_b;
    prev_fwd = fwd;
    if (r.hop_count < kMaxHops) r.hops[r.hop_count++] = cur;
  }

  if (have_deflect) {
    const double corr = std::min(std::max(first_deflect * 0.5, -kMaxTangentCorr),
                                 kMaxTangentCorr);
    r.tangent_end = geom::normalizeAngle(r.bearing + corr);
    if (ds_total > 1e-6) r.kappa_est = dpsi_total / ds_total;
  }

  // 当 road_topk<=1 但 commit 非空时，仍更新 commit（便于后续无缝切换）
  if (commit) {
    if (r.valid) {
      commit->valid = true;
      commit->hop_count = r.hop_count;
      commit->score = 0.0;
      commit->bearings[0] = r.bearing;
      for (int h = 1; h < r.hop_count; ++h) {
        commit->bearings[h] = std::atan2(r.hops[h].y - r.hops[h - 1].y,
                                          r.hops[h].x - r.hops[h - 1].x);
      }
    } else {
      commit->valid = false;
    }
  }

  return r;
}

}  // namespace road
}  // namespace unk
