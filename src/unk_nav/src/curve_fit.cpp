#include "unk_nav/curve_fit.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "unk_nav/geom_util.h"
#include "unk_nav/grid_util.h"

namespace unk {
namespace curve {

// ── 公共接口（在 curve_fit.h 中声明）─────────────────────────────

bool fitHermite(const GridMap& work_grid, const Point2D& p1, double theta0, double theta1,
                double spacing, double curvature_baseline, Path* out) {
  if (out == nullptr || work_grid.empty()) return false;

  // 起点固定为 base 系原点（车位）。距离过短没有拟合意义。
  const double L = std::hypot(p1.x, p1.y);
  if (L < std::max(spacing, 1e-3)) return false;

  // 退化保护：切向差 < 1° 时直线已切向连续，拟合只会引入采样噪声
  if (std::fabs(geom::normalizeAngle(theta1 - theta0)) < kPi / 180.0) return false;

  // 三次 Hermite 基函数：P(t) = h00·P0 + h10·m0 + h01·P1 + h11·m1
  // 切向模长取弦长 L（标准做法：曲线不过冲不塌瘪，弯度由 θ₀/θ₁ 控制）
  const double m0x = L * std::cos(theta0), m0y = L * std::sin(theta0);
  const double m1x = L * std::cos(theta1), m1y = L * std::sin(theta1);

  const int n = std::max(2, static_cast<int>(std::ceil(L / std::max(spacing, 1e-6))));
  std::vector<Point2D> pts;
  pts.reserve(n + 1);
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / n;
    const double t2 = t * t, t3 = t2 * t;
    const double h10 = t3 - 2.0 * t2 + t;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    // h00·P0 = 0（起点为原点）
    pts.push_back(Point2D{h10 * m0x + h01 * p1.x + h11 * m1x,
                          h10 * m0y + h01 * p1.y + h11 * m1y});
  }

  // 端点精确性：首点必须是车位、末点必须是子目标（浮点累积会漂，直接钉死）
  pts.front() = Point2D{0.0, 0.0};
  pts.back() = p1;

  // 碰撞复查：曲线会向侧面鼓包（直线是构造性无碰撞的，曲线不是），
  // 任一段扫进膨胀/障碍区 → 拟合失败，调用方回退直线。
  if (!grid::polylineFree(work_grid, pts)) return false;

  // s 与基线法曲率 k 由现成工具生成（k 非零 → speed_planner 曲率限速自动生效）
  *out = geom::toPath(pts, curvature_baseline);
  return out->size() >= 2;
}

bool fitSpline(const GridMap& work_grid, const std::vector<Point2D>& control_pts,
               double start_tangent, double spacing, double curvature_baseline, Path* out) {
  if (out == nullptr || work_grid.empty()) return false;
  const int n = static_cast<int>(control_pts.size());
  if (n < 2) return false;  // 至少起点 + 一个落点

  const double sp = std::max(spacing, 1e-6);
  double total = 0.0;
  for (int i = 0; i + 1 < n; ++i)
    total += std::hypot(control_pts[i + 1].x - control_pts[i].x,
                        control_pts[i + 1].y - control_pts[i].y);
  if (total < sp) return false;  // 总长过短没有拟合意义

  // 各点切向量（Catmull-Rom 均匀参数、张力 0.5）：
  //   · 起点：方向钉死为车头（start_tangent），模长取首段弦长，避免起步突然加减速；
  //   · 中间点：相邻两点中心差分 0.5·(P[i+1]−P[i−1])，天然 C1 连续；
  //   · 末点：末段方向外推 P[n−1]−P[n−2]（等价于镜像虚拟点后做中心差分）。
  std::vector<Point2D> T(n);
  {
    const double s0 = std::hypot(control_pts[1].x - control_pts[0].x,
                                 control_pts[1].y - control_pts[0].y);
    T[0] = Point2D{s0 * std::cos(start_tangent), s0 * std::sin(start_tangent)};
  }
  for (int i = 1; i + 1 < n; ++i)
    T[i] = Point2D{0.5 * (control_pts[i + 1].x - control_pts[i - 1].x),
                   0.5 * (control_pts[i + 1].y - control_pts[i - 1].y)};
  T[n - 1] = Point2D{control_pts[n - 1].x - control_pts[n - 2].x,
                     control_pts[n - 1].y - control_pts[n - 2].y};

  // 逐段三次 Hermite 拼接（段参数 t∈[0,1]，切向量即该参数化的端点导数）。
  std::vector<Point2D> dense;
  dense.push_back(control_pts[0]);
  for (int i = 0; i + 1 < n; ++i) {
    const Point2D &P0 = control_pts[i], &P1 = control_pts[i + 1];
    const Point2D &M0 = T[i], &M1 = T[i + 1];
    const double seg = std::hypot(P1.x - P0.x, P1.y - P0.y);
    const int k = std::max(1, static_cast<int>(std::ceil(seg / sp)));
    for (int j = 1; j <= k; ++j) {  // j 从 1 起：段起点即上段末点，已入列，跳过
      const double t = static_cast<double>(j) / k;
      const double t2 = t * t, t3 = t2 * t;
      const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
      const double h10 = t3 - 2.0 * t2 + t;
      const double h01 = -2.0 * t3 + 3.0 * t2;
      const double h11 = t3 - t2;
      dense.push_back(Point2D{h00 * P0.x + h10 * M0.x + h01 * P1.x + h11 * M1.x,
                              h00 * P0.y + h10 * M0.y + h01 * P1.y + h11 * M1.y});
    }
  }
  // 端点精确性：首点=车位、末点=最远跳点（浮点累积会漂，直接钉死）
  dense.front() = control_pts[0];
  dense.back() = control_pts[n - 1];

  // 碰撞复查：样条在拐角处仍可能侧向鼓包，任一段扫进膨胀/障碍区 → 失败回退直线。
  if (!grid::polylineFree(work_grid, dense)) return false;

  *out = geom::toPath(dense, curvature_baseline);
  return out->size() >= 2;
}

}  // namespace curve
}  // namespace unk
