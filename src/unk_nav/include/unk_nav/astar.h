#pragma once
// 局部栅格 A*。
//
// ── 使用约束（务必遵守）──────────────────────────────────────────
// 这是**窗口内**的搜索，goal 参数必须是已经落在局部栅格范围内的「子目标」，
// 绝不允许把远处的真实终点直接传进来。真实终点 → 子目标的投影由 subgoal 模块
// 负责（见 types.h 顶部的核心约束 2、3）。传入越界的 goal 会直接返回空路径。
//
// 与全局 A* 的本质区别：本搜索的结果只是当前滚动周期内的一段局部路径，
// 下一周期会在新的栅格上重新搜索，不存在「一次规划到终点」的语义。
#include "unk_nav/types.h"
#include "unk_nav/grid_util.h"

namespace unk {
namespace astar {

struct Options {
  double unknown_cost = 1.4;      // unknown 格代价倍率（>1 偏向已知区，仍可穿越）
  int max_iter = 200000;          // 迭代上限（安全护栏，防止极端栅格下失控）
  double w = 1.0;                 // 启发式权重 f = g + w*h。1.0=严格最优（默认，靠 h 打破 f 相等的对称）；
                                  // >1（如 1.05）=加权 A*，扩展更少但代价次优 ≤ w×
  bool allow_unknown = true;      // false 时 unknown 视为不可通行
  bool forbid_corner_cutting = true;  // 对角移动要求两侧正交格均可行（防穿墙角）
  int goal_snap_radius = 10;      // 目标格不可行时的螺旋搜索环数上限

  // 障碍软代价（批次2）：进入单格代价 = step × (1 + soft_k·exp(-d/soft_sigma))，
  // d 为该格到最近障碍的距离（米，来自截断 DistanceField）。soft_k<=0 关闭（默认，
  // 不构建距离带 → 零额外开销）。代价因子恒 ≥1 → 单格最小代价 ≥step，
  // 欧氏启发仍可采纳 → 不破坏最优性与批次1 的 tie-break。
  double soft_k = 0.0;
  double soft_sigma = 0.35;       // 距离衰减尺度 m（仅在 soft_k>0 时使用）

  // 一致性软代价（批次3）：进入单格代价再加 consistency_k·(1-exp(-d_prev/consistency_sigma))，
  // d_prev 为该格到「上一帧路径」的距离（米）。落在上帧路径上 → 加 0，偏离越远越接近
  // consistency_k（饱和上限）。作用：轴对称镜像两条路 f/g/h 全相等时，tie-break 无法区分，
  // 而「沿上帧走」因这项而严格更便宜 → 打破对称、消除每帧左右翻烧饼的抖动。
  // consistency_k<=0 或 prev_path 为空/无效 → 关闭（零开销）。只加不减 → 单格代价仍 ≥step
  // → 欧氏启发仍可采纳，最优性（对新代价函数）与批次1 tie-break 均不受影响。
  double consistency_k = 0.0;
  double consistency_sigma = 0.4;   // 黏性走廊半宽 m（仅在 consistency_k>0 时使用）
  // 上一帧路径（**当前 base_link 系**，调用方负责把上帧结果换算到本帧车体系）；可空。
  const std::vector<Point2D>* prev_path = nullptr;

  // ---- 路径平滑（见 path_smooth.h，全部为 0 时关闭）----
  double smooth_fillet_radius = 0.0;    // 拐点圆弧倒角半径 m
  int smooth_laplacian_iters = 0;       // 拉普拉斯松弛迭代次数
  double smooth_laplacian_lambda = 0.2; // 松弛系数
  int smooth_shrink_retry = 3;          // 圆弧碰撞时半径折半重试次数

  // 曲率测量基线 m（见 NavParams::curvature_baseline）；<=0 退化为按相邻点估
  double curvature_baseline = 0.0;
};

// 可复用的搜索工作区。
// 用「代际标记」代替每次把 g_cost 整体填成 inf：stamp[i] != gen 即视为未访问。
// 于是每周期成本只与**实际扩展的格数**相关，与栅格总格数无关，且栅格尺寸不变时
// 完全不重新分配 —— 每周期零堆分配，这对实时系统的确定性很重要。
// 实车窗口 51m@0.05m ≈ 100 万格时，不做这件事就是每周期 12MB 的分配 + memset。
struct Workspace {
  std::vector<double> g_cost;
  std::vector<int> parent;
  std::vector<int> stamp;
  int gen = 0;
  int last_iter = 0;   // 诊断：上次 plan() 的扩展迭代数（验证 tie-break 减少扩展用）
  std::vector<Point2D> cells;      // 回溯出的格中心点列
  std::vector<Point2D> straight;   // LOS 拉直结果
  std::vector<Point2D> smoothed;   // 平滑结果
  grid::DistanceField dist;        // 障碍软代价用的距离带（仅 soft_k>0 时构建/复用）
  grid::DistanceField prev;        // 一致性软代价用的「到上帧路径」距离带（仅 consistency_k>0 时）

  void ensure(size_t n);  // 容量不足时重新分配并复位代际
};

// 在局部栅格上从 start 搜到 goal（均为车体系世界坐标）。
// 失败返回空 Path，失败原因包括：栅格为空 / 起终点越界 / 起点被困 /
// 目标附近无可行格 / 目标不可达 / 迭代超限。
// 成功返回的路径已做 LOS 拉直 + 平滑 + 按段加密采样，首点为 start、末点为 goal。
// ws 传 nullptr 时内部用局部工作区（便于单测），生产调用应传入复用的 Workspace。
Path plan(const GridMap& grid, const Point2D& start, const Point2D& goal,
          double spacing, const Options& opt = Options(), Workspace* ws = nullptr);

}  // namespace astar
}  // namespace unk
