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

// 截断式障碍距离带：多源 BFS 从所有占据格出发，只填充「到最近障碍 <= max_dist_cells」
// 的格，带外格视为足够远。距离按 8-连通步数存 uint8（Chebyshev 距离，近似欧氏）。
//
// 跨周期复用缓冲区 + 代际 stamp（stamp != gen 即视为带外），于是：
//   · 空世界只有 O(全图) 一次扫描找种子（读 int8，1M 格 ≈ 0.2~0.5 ms），
//     带内几乎无扩展 → 极便宜；
//   · 障碍密集时成本上界是带面积，被 max_dist_cells 截断，不随精确欧氏膨胀；
//   · 栅格尺寸与 max_dist_cells 不变时零堆分配 —— 保持现有每周期零分配约定。
//
// 供 A* 障碍软代价 exp(-d/σ) 使用；将来 speed_planner 也可复用同一份。
struct DistanceField {
  int max_dist_cells = 0;             // 截断半径（格）；<=0 时 build() 直接返回，视为禁用
  std::vector<uint8_t> dist;          // 带内格到最近障碍的 BFS 步数（需配 stamp 判定有效性）
  std::vector<int> stamp;             // 代际标记；stamp[idx] != gen 视为「远」
  int gen = 0;                        // 当前代际
  std::vector<int> queue;             // BFS 环形队列，跨周期复用零分配

  // 容量或截断半径变化时重分配并复位代际；两者都不变则直接返回（复用）。
  void ensure(size_t n, int max_cells);
  // 用 g 的占据格作为源重建距离带；每次调用 gen 自增（等价于把整幅 stamp 清零）。
  // 溢出保护：gen 达到 INT_MAX 时把 stamp 整体清零一次（按 10Hz 跑 6.8 年才会触发）。
  void build(const GridMap& g);
  // 用一组车体系世界坐标点（如上一帧路径）作为源重建距离带。语义同 build，
  // 但种子来自给定折线而非栅格里的占据格；落在窗口外的点自动忽略。
  void buildFromPoints(const GridMap& g, const std::vector<Point2D>& pts);
  // 按下标查询：命中带内返回 BFS 步数（0=占据格），带外返回 max_dist_cells + 1。
  int atIdx(size_t idx) const {
    return stamp[idx] == gen ? static_cast<int>(dist[idx]) : max_dist_cells + 1;
  }
};

}  // namespace grid
}  // namespace unk
