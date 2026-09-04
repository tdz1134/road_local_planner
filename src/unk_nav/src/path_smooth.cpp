#include "unk_nav/path_smooth.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "unk_nav/geom_util.h"
#include "unk_nav/grid_util.h"

namespace unk {
namespace smooth {

// ── 内部辅助（仅本文件使用）──────────────────────────────────────
namespace {

constexpr double kEps = 1e-9;

// 绕圆心 O 从极角 a0 扫过 sweep 生成圆弧采样点（不含起点，含终点）
void appendArc(std::vector<Point2D>* out, const Point2D& O, double r, double a0,
               double sweep, double spacing) {
  const double arc_len = std::fabs(sweep) * r;
  const int m = std::max(1, static_cast<int>(std::ceil(arc_len / std::max(spacing, 1e-6))));
  for (int k = 1; k <= m; ++k) {
    const double a = a0 + sweep * static_cast<double>(k) / static_cast<double>(m);
    out->push_back({O.x + r * std::cos(a), O.y + r * std::sin(a)});
  }
}

}  // namespace

// ── 公共接口（在 path_smooth.h 中声明）───────────────────────────

double filletRadiusForCornerSpeed(double v_corner, double w_max, double a_lat_max) {
  const double v = std::max(0.0, v_corner);
  if (v <= kEps) return 0.0;
  double r = 0.0;
  if (w_max > kEps) r = std::max(r, v / w_max);              // w = v/r <= w_max
  if (a_lat_max > kEps) r = std::max(r, v * v / a_lat_max);  // v^2/r <= a_lat_max
  return r;
}

std::vector<Point2D> filletCorners(const GridMap& g, const std::vector<Point2D>& pts,
                                   const Options& opt) {
  if (pts.size() < 3 || opt.fillet_radius <= 0.0 || g.empty()) return pts;

  std::vector<Point2D> out;
  out.reserve(pts.size() * 2);
  out.push_back(pts.front());

  const int retries = std::max(0, opt.shrink_retry);
  for (size_t i = 1; i + 1 < pts.size(); ++i) {
    const Point2D& prev = pts[i - 1];
    const Point2D& cur = pts[i];
    const Point2D& next = pts[i + 1];
    const double L1 = std::hypot(cur.x - prev.x, cur.y - prev.y);
    const double L2 = std::hypot(next.x - cur.x, next.y - cur.y);
    if (L1 < kEps || L2 < kEps) {
      out.push_back(cur);
      continue;
    }

    const Point2D u1{(cur.x - prev.x) / L1, (cur.y - prev.y) / L1};
    const Point2D u2{(next.x - cur.x) / L2, (next.y - cur.y) / L2};
    // 偏转角 theta ∈ (0, pi)
    const double cos_t = std::max(-1.0, std::min(1.0, u1.x * u2.x + u1.y * u2.y));
    const double theta = std::acos(cos_t);
    if (theta < opt.min_turn_angle || theta > kPi - opt.min_turn_angle) {
      out.push_back(cur);  // 近似直线无需倒角；近似掉头倒角会退化成尖点，直接跳过
      continue;
    }
    const double tan_half = std::tan(theta * 0.5);
    if (tan_half < kEps) {
      out.push_back(cur);
      continue;
    }

    const double cross = u1.x * u2.y - u1.y * u2.x;
    const double s = (cross >= 0.0) ? 1.0 : -1.0;  // 左转为正
    // 相邻两个拐点可能共用中间段，各占 0.45 保证两段倒角不重叠
    const double r_fit = 0.45 * std::min(L1, L2) * tan_half;
    double r = std::min(opt.fillet_radius, r_fit);

    // 入切点 A、出切点 B 都在已验证无碰撞的原线段上，所以只需复验圆弧本身
    // （连同与上一点的衔接段一起查，代价可忽略）
    bool inserted = false;
    std::vector<Point2D> arc;
    for (int attempt = 0; attempt <= retries && r >= 1e-3; ++attempt, r *= 0.5) {
      const double T = r / tan_half;  // 切点到拐点的距离
      const Point2D A{cur.x - u1.x * T, cur.y - u1.y * T};  // 入切点
      // 出切点 B = cur + u2*T 无需显式算出：appendArc 扫过 s*theta 后的末点即 B
      // 圆心 = 入切点沿转向法向偏移 r；法向 n1 = s * (-u1.y, u1.x)
      const Point2D O{A.x - s * u1.y * r, A.y + s * u1.x * r};
      const double a0 = std::atan2(A.y - O.y, A.x - O.x);

      arc.clear();
      arc.push_back(out.back());  // 衔接段起点
      arc.push_back(A);
      appendArc(&arc, O, r, a0, s * theta, opt.spacing);  // 末点即 B

      if (grid::polylineFree(g, arc)) {
        // 去掉临时加进去的衔接起点，把 A + 圆弧追加到结果
        for (size_t k = 1; k < arc.size(); ++k) out.push_back(arc[k]);
        inserted = true;
        break;
      }
    }
    if (!inserted) out.push_back(cur);  // 半径折到底仍碰撞 → 保留尖角，宁尖勿撞
  }

  out.push_back(pts.back());
  return out;
}

std::vector<Point2D> laplacian(const GridMap& g, const std::vector<Point2D>& pts,
                               const Options& opt) {
  if (pts.size() < 3 || opt.laplacian_iters <= 0 || g.empty()) return pts;
  const double lam = std::max(0.0, std::min(0.9, opt.laplacian_lambda));

  std::vector<Point2D> cur = pts;
  std::vector<Point2D> next;
  next.reserve(pts.size());
  for (int it = 0; it < opt.laplacian_iters; ++it) {
    next = cur;
    // 端点固定：起点是当前车位、终点是子目标，都不允许被松弛挪走
    for (size_t i = 1; i + 1 < cur.size(); ++i) {
      next[i].x = cur[i].x + lam * (cur[i - 1].x + cur[i + 1].x - 2.0 * cur[i].x);
      next[i].y = cur[i].y + lam * (cur[i - 1].y + cur[i + 1].y - 2.0 * cur[i].y);
    }
    if (!grid::polylineFree(g, next)) break;  // 松弛撞障 → 停在上一轮结果
    cur.swap(next);
  }
  return cur;
}

std::vector<Point2D> run(const GridMap& g, const std::vector<Point2D>& pts,
                         const Options& opt) {
  if (pts.size() < 2 || g.empty()) return pts;
  auto out = filletCorners(g, pts, opt);
  out = laplacian(g, out, opt);
  // 最终复验：任何意外都退回输入。宁可交出一条尖角路径，也绝不交出撞障路径。
  if (!grid::polylineFree(g, out)) return pts;
  return out;
}

double maxCurvature(const std::vector<Point2D>& pts) {
  double m = 0.0;
  for (size_t i = 1; i + 1 < pts.size(); ++i) {
    m = std::max(m, std::fabs(geom::threePointCurvature(pts[i - 1], pts[i], pts[i + 1])));
  }
  return m;
}

double maxCurvatureRate(const std::vector<Point2D>& pts) {
  if (pts.size() < 3) return 0.0;
  std::vector<double> k(pts.size(), 0.0);
  std::vector<double> s(pts.size(), 0.0);
  for (size_t i = 1; i < pts.size(); ++i) {
    s[i] = s[i - 1] + std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    k[i] = geom::threePointCurvature(pts[i - 1], pts[i], pts[i + 1 < pts.size() ? i + 1 : i]);
  }
  k.front() = k[1];
  k.back() = k[pts.size() - 2];
  double m = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    const double ds = s[i] - s[i - 1];
    if (ds > kEps) m = std::max(m, std::fabs(k[i] - k[i - 1]) / ds);
  }
  return m;
}

}  // namespace smooth
}  // namespace unk
