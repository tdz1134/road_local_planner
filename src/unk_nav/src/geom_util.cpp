#include "unk_nav/geom_util.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace unk {
namespace geom {

// ── 内部辅助（仅本文件使用）──────────────────────────────────────
namespace {
constexpr double kEps = 1e-12;

// 在 s 单调不减的 Path 中找弧长最接近 s_target 的索引
size_t indexNearArc(const Path& p, double s_target) {
  if (p.empty()) return 0;
  if (s_target <= p.front().s) return 0;
  if (s_target >= p.back().s) return p.size() - 1;
  const auto it = std::lower_bound(
      p.begin(), p.end(), s_target, [](const PathPoint& a, double v) { return a.s < v; });
  const size_t i = static_cast<size_t>(it - p.begin());
  if (i > 0 && std::fabs(p[i - 1].s - s_target) < std::fabs(p[i].s - s_target)) return i - 1;
  return i;
}
}

// ── 公共接口（在 geom_util.h 中声明）─────────────────────────────

double normalizeAngle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a <= -kPi) a += 2.0 * kPi;
  return a;
}

Point2D globalToBase(const Point2D& p, const Pose2D& v) {
  const double dx = p.x - v.x, dy = p.y - v.y;
  const double c = std::cos(v.yaw), s = std::sin(v.yaw);
  return {c * dx + s * dy, -s * dx + c * dy};
}

Point2D baseToGlobal(const Point2D& p, const Pose2D& v) {
  const double c = std::cos(v.yaw), s = std::sin(v.yaw);
  return {c * p.x - s * p.y + v.x, s * p.x + c * p.y + v.y};
}

double globalYawToBase(double yaw, const Pose2D& v) {
  return normalizeAngle(yaw - v.yaw);
}

double polylineLength(const std::vector<Point2D>& pts) {
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    len += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  return len;
}

std::vector<Point2D> resampleByArc(const std::vector<Point2D>& pts, double spacing) {
  std::vector<Point2D> out;
  if (pts.empty() || spacing <= kEps) return out;
  out.push_back(pts.front());
  if (pts.size() < 2) return out;

  double next_s = spacing;  // 下一个采样点的目标弧长
  double acc = 0.0;         // 已累积弧长（= 当前段起点的弧长）
  for (size_t i = 1; i < pts.size(); ++i) {
    const Point2D& a = pts[i - 1];
    const Point2D& b = pts[i];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    if (seg < kEps) continue;  // 重合点跳过，避免除零
    const double seg_end = acc + seg;
    while (next_s <= seg_end + 1e-9) {
      const double t = (next_s - acc) / seg;
      out.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
      next_s += spacing;
    }
    acc = seg_end;
  }
  // 末端补齐：与最后一个采样点距离超过半个间距才补，避免产生退化短段
  const double tail = std::hypot(pts.back().x - out.back().x, pts.back().y - out.back().y);
  if (tail > spacing * 0.5) out.push_back(pts.back());
  return out;
}

std::vector<Point2D> resampleKeepCorners(const std::vector<Point2D>& pts, double spacing) {
  std::vector<Point2D> out;
  if (pts.empty() || spacing <= kEps) return out;
  out.push_back(pts.front());
  for (size_t i = 1; i < pts.size(); ++i) {
    const Point2D& a = pts[i - 1];
    const Point2D& b = pts[i];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    if (seg < kEps) continue;
    // 每段独立等分：份数向上取整 → 每份 <= spacing，且 b 必被输出（拐点保留）
    const int n = std::max(1, static_cast<int>(std::ceil(seg / spacing)));
    for (int k = 1; k <= n; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n);
      out.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
    }
  }
  return out;
}

std::vector<Point2D> clipByArc(const std::vector<Point2D>& pts, double s_max) {
  std::vector<Point2D> out;
  if (pts.empty()) return out;
  out.push_back(pts.front());
  if (s_max <= kEps) return out;

  double acc = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    const Point2D& a = pts[i - 1];
    const Point2D& b = pts[i];
    const double seg = std::hypot(b.x - a.x, b.y - a.y);
    if (seg < kEps) continue;
    if (acc + seg <= s_max) {
      out.push_back(b);
      acc += seg;
    } else {
      const double t = (s_max - acc) / seg;
      out.push_back({a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t});
      break;
    }
  }
  return out;
}

double pointToSegmentDistance(const Point2D& q, const Point2D& a, const Point2D& b) {
  const double abx = b.x - a.x, aby = b.y - a.y;
  const double ab2 = abx * abx + aby * aby;
  if (ab2 < kEps) return std::hypot(q.x - a.x, q.y - a.y);  // 退化为点
  // 投影参数夹取到 [0,1]，端点外取端点距离
  double t = ((q.x - a.x) * abx + (q.y - a.y) * aby) / ab2;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(q.x - (a.x + abx * t), q.y - (a.y + aby * t));
}

double pointToPolylineDistance(const Point2D& q, const std::vector<Point2D>& poly) {
  if (poly.empty()) return std::numeric_limits<double>::infinity();
  if (poly.size() == 1) return std::hypot(q.x - poly[0].x, q.y - poly[0].y);
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 1; i < poly.size(); ++i) {
    best = std::min(best, pointToSegmentDistance(q, poly[i - 1], poly[i]));
  }
  return best;
}

double threePointCurvature(const Point2D& a, const Point2D& b, const Point2D& c) {
  const double ab = std::hypot(b.x - a.x, b.y - a.y);
  const double bc = std::hypot(c.x - b.x, c.y - b.y);
  const double ca = std::hypot(c.x - a.x, c.y - a.y);
  const double denom = ab * bc * ca;
  if (denom < 1e-9) return 0.0;
  // 叉积带符号：左转（逆时针）为正
  const double cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
  return 2.0 * cross / denom;
}

Path toPath(const std::vector<Point2D>& pts, double curvature_baseline) {
  Path out;
  out.reserve(pts.size());
  double s = 0.0;
  for (size_t i = 0; i < pts.size(); ++i) {
    if (i > 0) s += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    PathPoint pp;
    pp.p = pts[i];
    pp.s = s;
    out.push_back(pp);
  }
  if (out.size() < 3) return out;
  const double s_end = out.back().s;

  // 路径比基线还短 → 无处放基线，只能用相邻三点
  if (curvature_baseline <= 0.0 || s_end < curvature_baseline) {
    // 退化模式：相邻三点。曲率随采样密度变化，仅用于单测与向后兼容。
    for (size_t i = 1; i + 1 < out.size(); ++i) {
      out[i].k = threePointCurvature(out[i - 1].p, out[i].p, out[i + 1].p);
    }
  } else {
    // 基线模式：取弧长相距 ±baseline/2 的两点，曲率只反映几何、与点距无关。
    //
    // 端点附近放不下完整基线。此时**不能**把基线夹成不对称（一边很短一边很长）：
    // 三点外接圆公式的分母是 ab*bc*ca，短边会把曲率严重放大，端点处会凭空冒出
    // 一个假尖峰。正确做法是只在能放下完整基线的内点区间计算，区间外沿用边界值。
    const double half = curvature_baseline * 0.5;
    size_t i_lo = 1;
    while (i_lo + 1 < out.size() && out[i_lo].s < half) ++i_lo;
    size_t i_hi = out.size() - 2;
    while (i_hi > i_lo && out[i_hi].s > s_end - half) --i_hi;

    for (size_t i = i_lo; i <= i_hi; ++i) {
      const size_t j = indexNearArc(out, out[i].s - half);
      const size_t k = indexNearArc(out, out[i].s + half);
      out[i].k = threePointCurvature(out[j].p, out[i].p, out[k].p);
    }
    for (size_t i = 0; i < i_lo; ++i) out[i].k = out[i_lo].k;
    for (size_t i = i_hi + 1; i < out.size(); ++i) out[i].k = out[i_hi].k;
    return out;
  }
  // 端点沿用相邻内点值，避免端点曲率恒为 0 造成限速漏判
  out.front().k = out[1].k;
  out.back().k = out[out.size() - 2].k;
  return out;
}

}  // namespace geom
}  // namespace unk
