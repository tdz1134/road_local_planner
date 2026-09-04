#include "unk_nav/speed_planner.h"

#include <algorithm>
#include <cmath>

namespace unk {
namespace speed {

// ── 内部辅助（仅本文件使用）──────────────────────────────────────
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kEps = 1e-9;
}  // namespace

// ── 公共接口（在 speed_planner.h 中声明）─────────────────────────

double brakeDistance(double v, const NavParams& p) {
  const double s = std::fabs(v);
  const double a = std::max(p.a_decel_max, kEps);
  return s * s / (2.0 * a) + s * p.t_reaction + p.safety_margin;
}

double speedForDistance(double d, const NavParams& p) {
  const double a = std::max(p.a_decel_max, kEps);
  const double T = std::max(0.0, p.t_reaction);
  const double dd = std::max(0.0, d);
  // v^2/(2a) + v*T = dd 的正根
  const double v = -a * T + std::sqrt(a * T * a * T + 2.0 * a * dd);
  return std::max(0.0, v);
}

Result limit(const Path& path, const GridMap& grid, double current_speed,
             const NavParams& p) {
  Result r;
  if (path.size() < 2) {
    r.emergency_stop = true;
    r.limit_by = "empty_path";
    return r;
  }

  // ---- 1) 路径上第一个阻挡点的弧长 ----
  // 必须沿路径**稠密**采样，不能只查顶点：顶点稀疏时会整段跨过薄障碍，
  // 把 d_obs 误判成 inf，制动包络随之失效。
  // A* 是在同一份膨胀栅格上搜的，正常情况全程无阻挡；这里仍复查，因为下游
  // 可能拿到跨周期的旧路径，或栅格在规划中途被更新。
  const double sample_step = std::max(grid.resolution, kEps);
  for (size_t i = 1; i < path.size(); ++i) {
    if (grid.blockedAt(path[i - 1].p.x, path[i - 1].p.y)) {
      r.obstacle_dist = path[i - 1].s;
      break;
    }
    const double seg = path[i].s - path[i - 1].s;
    if (seg <= kEps) continue;
    const int n = std::max(1, static_cast<int>(std::ceil(seg / sample_step)));
    const double dx = path[i].p.x - path[i - 1].p.x;
    const double dy = path[i].p.y - path[i - 1].p.y;
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n);
      if (grid.blockedAt(path[i - 1].p.x + dx * t, path[i - 1].p.y + dy * t)) {
        r.obstacle_dist = path[i - 1].s + seg * t;
        break;
      }
    }
    if (!std::isinf(r.obstacle_dist)) break;
  }

  // ---- 2) 制动包络硬校验 ----
  r.brake_dist = brakeDistance(current_speed, p);
  if (!(r.obstacle_dist > r.brake_dist)) {
    r.emergency_stop = true;
    r.limit_by = "brake_envelope";
    return r;
  }

  // ---- 3) 路径曲率统计 ----
  double k_max = 0.0;
  double dk_ds = 0.0;
  for (size_t i = 0; i < path.size(); ++i) {
    k_max = std::max(k_max, std::fabs(path[i].k));
    if (i > 0) {
      const double ds = path[i].s - path[i - 1].s;
      if (ds > kEps) {
        dk_ds = std::max(dk_ds, std::fabs(path[i].k - path[i - 1].k) / ds);
      }
    }
  }
  r.kappa_max = k_max;
  r.dk_ds_max = dk_ds;

  // 几何不可行：降速也走不出来，只能急停并让上层重新规划。
  // kappa_max <= 0 表示不启用该硬门限（差速底盘的默认配置，见 types.h 注释）。
  if (p.kappa_max > 0.0 && k_max > p.kappa_max) {
    r.emergency_stop = true;
    r.limit_by = "kappa_exceeded";
    return r;
  }

  // ---- 4) 四条限速取最小 ----
  double v = p.v_max;
  r.limit_by = "v_max";

  const double d_avail = std::isinf(r.obstacle_dist)
                             ? kInf
                             : std::max(0.0, r.obstacle_dist - p.safety_margin);
  const double v_obs = std::isinf(d_avail) ? p.v_max : speedForDistance(d_avail, p);
  if (v_obs < v) {
    v = v_obs;
    r.limit_by = "obstacle";
  }

  if (k_max > kEps) {
    const double v_curv = std::sqrt(std::max(p.a_lat_max, 0.0) / k_max);
    if (v_curv < v) {
      v = v_curv;
      r.limit_by = "lateral_accel";
    }
    const double v_w = p.w_max / k_max;  // w = v * kappa <= w_max
    if (v_w < v) {
      v = v_w;
      r.limit_by = "yaw_rate";
    }
  }

  if (dk_ds > kEps && p.dk_max > 0.0) {
    const double v_dk = p.dk_max / dk_ds;  // dk/dt = (dk/ds) * v <= dk_max
    if (v_dk < v) {
      v = v_dk;
      r.limit_by = "curvature_rate";
    }
  }

  r.v = std::max(0.0, v);
  r.emergency_stop = false;
  return r;
}

}  // namespace speed
}  // namespace unk
