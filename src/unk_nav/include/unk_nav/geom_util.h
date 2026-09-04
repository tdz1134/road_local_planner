#pragma once
// 纯几何工具：坐标系变换、折线弧长 / 重采样 / 裁剪、点-折线距离、曲率。
// 本文件不接触栅格，栅格相关查询见 grid_util.h。
#include <vector>

#include "unk_nav/types.h"

namespace unk {
namespace geom {

// 角度归一化到 (-pi, pi]
double normalizeAngle(double a);

// ---- 坐标系变换：vehicle 为车体在全局系的位姿 ----
// 全局系 → 车体系
Point2D globalToBase(const Point2D& p, const Pose2D& vehicle);
// 车体系 → 全局系
Point2D baseToGlobal(const Point2D& p, const Pose2D& vehicle);
// 全局系航向 → 车体系航向
double globalYawToBase(double yaw, const Pose2D& vehicle);

// ---- 折线几何 ----
// 折线总弧长
double polylineLength(const std::vector<Point2D>& pts);

// 按弧长等距重采样；首点保留，末点在间距不足半个 spacing 时舍弃
std::vector<Point2D> resampleByArc(const std::vector<Point2D>& pts, double spacing);

// 按段细分重采样，**强制保留每个拐点**：每段独立切成 ceil(len/spacing) 等份，
// 相邻输出点必落在同一条直段上。
// 与 resampleByArc 的区别：后者按全局弧长打点，相邻点可能跨过拐点，其连线会
// 切掉拐角内侧 —— 若拐点两侧是无碰撞折线，切角后的弦可能穿进障碍。
// 凡是「折线各段已验证无碰撞、需要加密采样后仍保证无碰撞」的场合必须用本函数。
std::vector<Point2D> resampleKeepCorners(const std::vector<Point2D>& pts, double spacing);

// 保留弧长 [0, s_max] 的部分，末端按插值截断
std::vector<Point2D> clipByArc(const std::vector<Point2D>& pts, double s_max);

// 点到线段的最短距离（端点处自动夹取，不做无限延长）
double pointToSegmentDistance(const Point2D& q, const Point2D& a, const Point2D& b);

// 点到折线的最短距离；折线为空返回 +inf，单点返回点距
double pointToPolylineDistance(const Point2D& q, const std::vector<Point2D>& poly);

// 相邻三点的外接圆曲率（带符号：左转为正）；三点共线或重合返回 0
double threePointCurvature(const Point2D& a, const Point2D& b, const Point2D& c);

// 点列 → Path：填充累计弧长 s 与逐点曲率 k。
//
// curvature_baseline 是曲率的**测量基线**（m）：第 i 点的曲率用弧长相距 ±baseline/2
// 的两个点与它自己三点求外接圆，而不是用相邻点。
//   <= 0 → 退化为相邻三点（旧行为，曲率会随采样密度变化，仅建议单测用）
//   >  0 → 曲率只反映几何形状，与 path_spacing 无关
// 必须用正值：按相邻点估时，点距 0.05m 下 κ ≈ 565 × 横向偏差，栅格阶梯的微小抖动
// 会被放大成 30+/m 的假曲率，速度规划里的 w = v·κ 与 dk/dt 两条约束随即把车压到爬行。
Path toPath(const std::vector<Point2D>& pts, double curvature_baseline = 0.0);

}  // namespace geom
}  // namespace unk
