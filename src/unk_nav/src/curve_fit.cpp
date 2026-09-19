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

  // 各段弦长 D[i] = |P[i+1]−P[i]|（i=0..n-2）与折线总长。
  std::vector<double> D(static_cast<size_t>(n - 1));
  double total = 0.0;
  for (int i = 0; i + 1 < n; ++i) {
    D[static_cast<size_t>(i)] = std::hypot(control_pts[i + 1].x - control_pts[i].x,
                                           control_pts[i + 1].y - control_pts[i].y);
    total += D[static_cast<size_t>(i)];
  }
  if (total < sp) return false;  // 总长过短没有拟合意义

  // 各点切向量 G[i] = dP/dt（t 为弦长参数，非均匀）。弦长 Catmull-Rom：
  //   · 起点：方向钉死为车头（start_tangent），模长取 1（单位割线量纲，下方缩放后=首段弦长）；
  //   · 中间点：相邻两段单位割线方向平均 0.5·(u[i−1]+u[i])，天然 C1 连续；
  //   · 末点：末段单位方向 u[n−2]。
  // 相比旧版均匀参数化（切向=0.5·(P[i+1]−P[i−1])、直接塞进 t∈[0,1]），此处切向按
  // “每段自身弦长”缩放（见下方采样）。根治短末段过冲：旧版长入段撑出的大切向被原样
  // 用进极短的末段 → tight loop、曲率爆炸（实测峰值 κ 11.8→1.0）；弦长缩放让短段自动
  // 拿到小切向。均匀跳点下与旧版几乎一致（近等距时两者收敛）。
  auto segUnit = [&](int i) -> Point2D {  // 第 i 段单位方向；退化段(长度≈0)返回 0
    const double d = std::max(D[static_cast<size_t>(i)], 1e-9);
    return Point2D{(control_pts[i + 1].x - control_pts[i].x) / d,
                   (control_pts[i + 1].y - control_pts[i].y) / d};
  };
  std::vector<Point2D> G(static_cast<size_t>(n));
  G[0] = Point2D{std::cos(start_tangent), std::sin(start_tangent)};
  for (int i = 1; i + 1 < n; ++i) {
    const Point2D a = segUnit(i - 1), b = segUnit(i);
    G[static_cast<size_t>(i)] = Point2D{0.5 * (a.x + b.x), 0.5 * (a.y + b.y)};
  }
  G[static_cast<size_t>(n - 1)] = segUnit(n - 2);

  // 逐段三次 Hermite 拼接。段参数 s∈[0,1]，端点导数 = 弦长导数 × 本段弦长 D[i]
  //（链式法则 dP/ds = dP/dt · dt/ds，此处 dt/ds = D[i]）。同一节点对相邻两段给出
  // 不同的 s-空间导数（各自按本段弦长缩放），这正是非均匀参数化消除过冲的机制。
  std::vector<Point2D> dense;
  dense.push_back(control_pts[0]);
  for (int i = 0; i + 1 < n; ++i) {
    const Point2D &P0 = control_pts[i], &P1 = control_pts[i + 1];
    const double Di = D[static_cast<size_t>(i)];
    const Point2D &Gi = G[static_cast<size_t>(i)], &Gj = G[static_cast<size_t>(i + 1)];
    const Point2D M0{Gi.x * Di, Gi.y * Di};
    const Point2D M1{Gj.x * Di, Gj.y * Di};
    const int k = std::max(1, static_cast<int>(std::ceil(Di / sp)));
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
