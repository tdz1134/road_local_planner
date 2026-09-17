#pragma once
// 曲线拟合：把「车 → 子目标」的直线段升级为切向连续的光滑曲线。
//
// 为什么需要：差速底盘低速可原地转向，直线段够用；但速度上来后转向不能瞬时完成，
// 「原地转 θ 再直线走」的运动不连续会让纯跟踪切内角、横向误差放大。链式前瞻
// （road_follow.h）量出的跳点 hops 提供了曲线要经过的一系列走廊落点。
//
// 主推方法 fitSpline（Catmull-Rom 样条）：曲线**精确经过**给定的一组控制点
// （起点=车位、各跳跳点），切向由相邻点差分自动估计、起点切向钉死车头方向。
// 滚动重规划下终点只是本帧过渡量，让曲线延伸贴合走廊到最远跳点，比"停在第一跳
// 末端"更能预示道路走向。
//
// 旧方法 fitHermite（单段两点两切向）保留：语义清晰、有独立单测，作为参考/回退。
//
// 安全底线：直线是构造性无碰撞的（子目标落点在自由射线上），曲线不是——拐角大时
// 曲线会向侧面鼓包，可能扫进膨胀区。因此拟合结果必须逐段碰撞复查
//（grid::polylineFree），失败返回 false，调用方回退直线。直线永远保底。
#include <vector>

#include "unk_nav/types.h"

namespace unk {
namespace curve {

// 拟合 (0,0,theta0) → (p1,theta1) 的三次 Hermite 曲线，按 spacing 采样成 Path
//（s 与基线法曲率 k 由 geom::toPath 生成，k 非零 → speed_planner 曲率限速自动生效）。
//
// 返回 false（调用方应回退直线）的情形：
//   · |theta1 - theta0| < 1°：切向已连续，直线即最优，无需拟合；
//   · 曲线穿进膨胀/障碍区（polylineFree 复查失败）；
//   · 距离过短 / 出参为空。
// curvature_baseline：曲率估计基线 m（透传给 geom::toPath，与参数表同源）。
bool fitHermite(const GridMap& work_grid, const Point2D& p1, double theta0, double theta1,
                double spacing, double curvature_baseline, Path* out);

// Catmull-Rom 样条：过 control_pts（车体系，依次 [0]=起点/车位、[1..]=各跳跳点），
// 每段用相邻点差分定切向（起点切向钉死为 start_tangent、末点用末段方向外推），
// 按 spacing 逐段采样成 Path（s 与基线法曲率 k 由 geom::toPath 生成）。
// 与 fitHermite 的区别：样条精确经过所有中间点、支持任意点数；Hermite 只有两端点。
//
// 返回 false（调用方应回退直线）的情形：
//   · control_pts 少于 2 点，或总长 < spacing（无拟合意义）；
//   · 样条任一采样点落进膨胀/障碍区（polylineFree 复查失败）。
bool fitSpline(const GridMap& work_grid, const std::vector<Point2D>& control_pts,
               double start_tangent, double spacing, double curvature_baseline, Path* out);

}  // namespace curve
}  // namespace unk
