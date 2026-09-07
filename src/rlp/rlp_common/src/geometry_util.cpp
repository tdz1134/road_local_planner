#include "rlp_common/geometry_util.h"

#include <algorithm>
#include <cmath>

namespace rlp {

// ── 内部辅助（仅本文件使用，不对外暴露）──────────────────────────
namespace {

// 输入: 线段端点 a,b 与查询点 q; 输出: q 到线段 ab 的最短距离
// 功能: 点到线段距离（含端点退化处理）
double pointSegDistance(const Point2D& a, const Point2D& b, const Point2D& q) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double l2 = dx * dx + dy * dy;
  if (l2 < 1e-12) return std::hypot(q.x - a.x, q.y - a.y);
  double t = ((q.x - a.x) * dx + (q.y - a.y) * dy) / l2;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(q.x - (a.x + t * dx), q.y - (a.y + t * dy));
}

// 输入: 两点 a,b 与插值参数 t∈[0,1]; 输出: 线性插值点
// 功能: 两点线性插值
Point2D lerp(const Point2D& a, const Point2D& b, double t) {
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

}  // namespace

// ── 公共接口（在 geometry_util.h 中声明）────────────────────────

// 输入: 折线点列 pts; 输出: 折线总长度（米）
// 功能: 计算折线总弧长
double polylineLength(const std::vector<Point2D>& pts) {
  double len = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    len += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  return len;
}

// 输入: 折线 pts 与采样间距 spacing; 输出: 等弧长间距的重采样点列
// 功能: 按弧长等距重采样折线（首尾点保留）
std::vector<Point2D> resampleByArc(const std::vector<Point2D>& pts, double spacing) {
  if (pts.size() < 2 || spacing <= 1e-9) return pts;
  std::vector<Point2D> out;
  out.push_back(pts.front());
  double next = spacing;  // 下一个输出点的弧长位置
  double acc = 0.0;       // 当前累计弧长（段起点处）
  for (size_t i = 1; i < pts.size(); ++i) {
    const double seg =
        std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    while (acc + seg >= next) {
      const double t = (next - acc) / seg;
      out.push_back(lerp(pts[i - 1], pts[i], t));
      next += spacing;
    }
    acc += seg;
  }
  out.push_back(pts.back());
  return out;
}

// 输入: 折线 pts 与最大弧长 s_max; 输出: 截断后的折线点列
// 功能: 保留弧长 [0, s_max] 的部分，末端点插值补齐
std::vector<Point2D> clipByArc(const std::vector<Point2D>& pts, double s_max) {
  std::vector<Point2D> out;
  if (pts.empty() || s_max <= 0.0) return out;
  out.push_back(pts.front());
  double acc = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {
    const double seg =
        std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    if (acc + seg >= s_max) {
      out.push_back(lerp(pts[i - 1], pts[i], (s_max - acc) / std::max(seg, 1e-9)));
      return out;
    }
    out.push_back(pts[i]);
    acc += seg;
  }
  return out;
}

// 输入: 折线 pts 与横向偏移量 offset（正=左法线方向）; 输出: 偏移后的折线
// 功能: 沿折线左法线方向横向平移（offset>0 向左侧）
std::vector<Point2D> offsetPolyline(const std::vector<Point2D>& pts, double offset) {
  const size_t n = pts.size();
  std::vector<Point2D> out;
  if (n < 2) return pts;
  out.reserve(n);
  for (size_t i = 0; i < n; ++i) {
    const Point2D& a = pts[std::max(i, size_t(1)) - 1];
    const Point2D& b = pts[std::min(i + 1, n - 1)];
    double tx = b.x - a.x, ty = b.y - a.y;
    const double norm = std::max(std::hypot(tx, ty), 1e-9);
    tx /= norm;
    ty /= norm;
    // 左法线 = 切向逆时针旋转 90°
    out.push_back({pts[i].x - ty * offset, pts[i].y + tx * offset});
  }
  return out;
}

// 输入: 查询点 q 与折线 poly; 输出: q 到 poly 的最短距离
// 功能: 点到折线的最短距离（遍历所有线段取最小值）
double pointToPolylineDistance(const Point2D& q, const std::vector<Point2D>& poly) {
  if (poly.empty()) return 1e9;
  if (poly.size() == 1) return std::hypot(q.x - poly[0].x, q.y - poly[0].y);
  double best = 1e9;
  for (size_t i = 1; i < poly.size(); ++i) {
    best = std::min(best, pointSegDistance(poly[i - 1], poly[i], q));
  }
  return best;
}

// 输入: 点列 pts; 输出: Path（每个点附带累计弧长 s）
// 功能: 将点列转换为带弧长标记的 Path
Path toPath(const std::vector<Point2D>& pts) {
  Path path;
  path.reserve(pts.size());
  double s = 0.0;
  for (size_t i = 0; i < pts.size(); ++i) {
    if (i > 0) s += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
    path.push_back({pts[i], s});
  }
  return path;
}

}  // namespace rlp
