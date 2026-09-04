#pragma once
// 子目标生成：把窗口外的远处终点投影成局部栅格内的一个可搜索目标点。
//
// 这是「没有全局地图」这一约束的直接产物：A* 只能搜到窗口内，所以每周期都要
// 重新决定「这一步朝哪儿走」。
//
// 只做沿终点方向的单点投影，不做扇形候选展开。后果是明确的：凹槽开口背向终点时，
// 投影点会落进凹槽里，A* 把它吸附到凹槽内壁 → 局部极小。扇形展开（在多个方位角上
// 各投一个候选、取可达且代价最低的）是这个缺口的补救手段之一，已排除在当前范围外
// （见 README「能力边界」）。
#include <vector>

#include "unk_nav/types.h"

namespace unk {
namespace subgoal {

struct Result {
  Point2D point;                 // 子目标，车体系
  bool valid = false;            // 是否成功生成
  double goal_dist = 0.0;        // 终点在车体系的距离 m
  double goal_bearing = 0.0;     // 终点在车体系的方位角 rad（0 = 正前方）
  double reach = 0.0;            // 实际投影距离 m
  bool clipped_by_window = false;  // 投影距离是否被窗口边界截短
};

// 沿终点方向投影：
//   reach = min(lookahead, goal_dist, 窗口边界在该方向上的可达距离)
// 用射线-包围盒（slab）求窗口边界，保证子目标始终落在终点方向上、只是被拉近，
// 不会像「按坐标轴夹取」那样把方向掰偏。
Result project(const GridMap& grid, const Point2D& goal_base, const NavParams& p);

}  // namespace subgoal
}  // namespace unk
