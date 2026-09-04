#pragma once
// 路径平滑：把栅格 A* + LOS 拉直后的「尖角折线」变成曲率有界的光滑路径。
//
// 为什么必须做：LOS 拉直后的拐点曲率约为 sqrt(2)/resolution（res=0.05 时约 28 /m），
// 速度规划里 w = v*kappa <= w_max 与 dk/dt <= dk_max 两条约束会把过弯速度压到爬行
// （实测横墙绕行时降到 v_max 的 17%）。平滑的目的不是好看，是把过弯速度提回来。
//
// 两步法，每步都做碰撞复验，任一步失败就回退到上一步结果 —— 因此输出保证无碰撞：
//   1) filletCorners：拐点插入与两边相切的圆弧，把曲率上界压到 1/r
//   2) laplacian    ：整体松弛，把圆弧进出口的曲率阶跃摊开，降低 dk/ds
// 只做第 1 步的话，曲率从 0 跳到 1/r 仍是阶跃，dk/dt 约束照样会限速。
#include <vector>

#include "unk_nav/types.h"

namespace unk {
namespace smooth {

struct Options {
  double fillet_radius = 0.0;     // 倒角圆弧半径 m；<=0 关闭倒角
  int laplacian_iters = 0;        // 拉普拉斯松弛迭代次数；<=0 关闭
  double laplacian_lambda = 0.2;  // 松弛系数 (0,1)，越大越平滑但越容易撞
  double spacing = 0.05;          // 圆弧采样间距 m
  int shrink_retry = 3;           // 圆弧碰撞时半径折半重试次数
  double min_turn_angle = 0.05;   // 小于此偏转角(rad)视为直线，不倒角
};

// 由目标过弯速度反解倒角半径，同时满足两条运动约束：
//   角速度     w = v/r <= w_max      →  r >= v/w_max
//   横向加速度 a = v^2/r <= a_lat_max →  r >= v^2/a_lat_max
// 这是动力学参数，不随 sensor_range 缩放（换雷达不该改变过弯半径）。
double filletRadiusForCornerSpeed(double v_corner, double w_max, double a_lat_max);

// 拐点圆弧倒角。圆弧采样点碰撞时半径折半重试，仍碰撞则保留尖角。
std::vector<Point2D> filletCorners(const GridMap& g, const std::vector<Point2D>& pts,
                                   const Options& opt);

// 拉普拉斯松弛 + 碰撞复验，端点固定不动。
std::vector<Point2D> laplacian(const GridMap& g, const std::vector<Point2D>& pts,
                               const Options& opt);

// 一站式：倒角 → 松弛。任何一步导致碰撞都会回退，输出必定无碰撞。
std::vector<Point2D> run(const GridMap& g, const std::vector<Point2D>& pts,
                         const Options& opt);

// 折线最大曲率（三点外接圆估计），用于验证平滑效果
double maxCurvature(const std::vector<Point2D>& pts);

// 折线最大曲率变化率 |dk|/ds，对应 dk/dt = (dk/ds) * v
double maxCurvatureRate(const std::vector<Point2D>& pts);

}  // namespace smooth
}  // namespace unk
