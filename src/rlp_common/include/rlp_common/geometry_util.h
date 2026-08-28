#pragma once
// 折线几何工具：重采样、裁剪、横向偏移、点-折线距离
#include <vector>

#include "rlp_common/types.h"

namespace rlp {

double polylineLength(const std::vector<Point2D>& pts);

// 按弧长等距重采样
std::vector<Point2D> resampleByArc(const std::vector<Point2D>& pts, double spacing);

// 保留弧长 [0, s_max] 的部分
std::vector<Point2D> clipByArc(const std::vector<Point2D>& pts, double s_max);

// 沿左法线方向横向偏移（offset > 0 向曲线左侧）；假设点列为前进方向有序
std::vector<Point2D> offsetPolyline(const std::vector<Point2D>& pts, double offset);

// 点到折线的最短距离
double pointToPolylineDistance(const Point2D& q, const std::vector<Point2D>& poly);

// 点列 → Path（填充弧长 s）
Path toPath(const std::vector<Point2D>& pts);

}  // namespace rlp
