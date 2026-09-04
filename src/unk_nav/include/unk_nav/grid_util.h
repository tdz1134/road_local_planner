#pragma once
// 栅格工具：障碍膨胀、车体足迹清空、视线检查、路径拉直、螺旋找可行格。
// 所有函数都以 GridMap 为输入，坐标系一律是车体系（base_link）。
#include <utility>
#include <vector>

#include "unk_nav/types.h"

namespace unk {
namespace grid {

// 圆盘膨胀核：返回半径 radius 覆盖的栅格偏移量集合（含 (0,0)）
std::vector<std::pair<int, int>> diskKernel(double radius, double resolution);

// 障碍膨胀：把 occupied 向外扩张 radius。
// inflate_unknown=false（默认）时 unknown 保持不变，以维持「乐观探索」语义；
// 越界格不参与写入。返回新栅格，不修改入参。
GridMap inflate(const GridMap& in, double radius, bool inflate_unknown = false);

// 车体足迹强制清空：以 (cx,cy) 为圆心、radius 为半径内的格置为 free。
// 必须在 inflate 之后调用，否则膨胀层会把车自己判成障碍 → A* 起点即死锁。
void clearFootprint(GridMap* g, double cx, double cy, double radius);

// 线段是否可通行：按 resolution 步进采样。
// occupied 或越界 → false；unknown → 放行（乐观）。
bool segmentFree(const GridMap& g, double x1, double y1, double x2, double y2);

// 整条折线是否可通行：所有顶点 + 所有线段都可通行才返回 true。
// 平滑类操作每次修改点列后都必须用它复验，否则会把路径推进障碍里。
bool polylineFree(const GridMap& g, const std::vector<Point2D>& pts);

// LOS 拉直（string pulling）：贪心地去掉能被直线跨越的中间点，消除栅格阶梯。
// 单次向后跳跃上限 kMaxShortcutSkip，避免 O(n^2) 退化。
std::vector<Point2D> shortcut(const GridMap& g, const std::vector<Point2D>& pts);

// 目标格不可行时，在其周围按环螺旋搜索最近的可行格。
// 成功返回 true 并改写 *gx/*gy；max_radius 为搜索环数上限。
bool findFeasibleCellNear(const GridMap& g, int* gx, int* gy, int max_radius);

}  // namespace grid
}  // namespace unk
