#include "unk_nav/astar.h"

#include <algorithm>
#include <climits>
#include <cmath>
#include <queue>
#include <vector>

#include "unk_nav/geom_util.h"
#include "unk_nav/grid_util.h"
#include "unk_nav/path_smooth.h"

namespace unk {
namespace astar {

// ── 内部辅助（仅本文件使用）──────────────────────────────────────
namespace {

constexpr double kInf = 1e18;

struct Cell {
  int x, y;
  double g;  // 起点到当前格的已知代价（单位：格，含 unknown 倍率）
  double h;  // 启发式（到子目标欧氏距离，格单位）—— f 相等时用它 tie-break
  double f;  // g + w*h
};

struct CellCmp {
  // min-heap：先按 f 升序；f 相等时按 h 升序（更靠近子目标者优先出队）。
  // 这条 tie-break 打破直线路径上的 f plateau，扩展数骤降；因为只调整 f 完全
  // 相等的节点出队顺序，最优代价不变 —— 严格最优性保留。
  // f、h 都相等时（轴对称镜像的典型情形）再按固定空间键 (y,x) 定序：
  // 让“三相等”节点的出队顺序与 push 顺序、浮点噪声无关 → 消除非确定性翻转。
  bool operator()(const Cell& a, const Cell& b) const {
    if (a.f != b.f) return a.f > b.f;
    if (a.h != b.h) return a.h > b.h;
    if (a.y != b.y) return a.y > b.y;
    return a.x > b.x;
  }
};

inline size_t indexOf(int x, int y, int w) {
  return static_cast<size_t>(y) * static_cast<size_t>(w) + static_cast<size_t>(x);
}

// 欧几里得启发式。单格最小代价倍率为 1.0（free），故对 8-连通栅格可采纳。
inline double heuristic(int x1, int y1, int x2, int y2) {
  return std::hypot(static_cast<double>(x1 - x2), static_cast<double>(y1 - y2));
}

// 8 方向偏移与步长（对角 √2）
const int kDx[8] = {1, 1, 1, 0, 0, -1, -1, -1};
const int kDy[8] = {1, 0, -1, 1, -1, 1, 0, -1};
const double kStep[8] = {kSqrt2, 1.0, kSqrt2, 1.0, 1.0, kSqrt2, 1.0, kSqrt2};

}  // namespace

// ── 公共接口（在 astar.h 中声明）─────────────────────────────────

void Workspace::ensure(size_t n) {
  if (g_cost.size() == n) return;
  g_cost.assign(n, 0.0);
  parent.assign(n, -1);
  stamp.assign(n, 0);
  gen = 0;
}

Path plan(const GridMap& grid, const Point2D& start, const Point2D& goal,
          double spacing, const Options& opt, Workspace* ws_in) {
  if (grid.empty()) return {};

  int sx = 0, sy = 0, gx = 0, gy = 0;
  // 起点必须在窗口内（车体位置由上游保证）；越界说明栅格与位姿不匹配
  if (!grid.worldToGrid(start.x, start.y, &sx, &sy)) return {};
  // 子目标必须在窗口内：投影与夹取是 subgoal 模块的责任，此处仅做防御
  if (!grid.worldToGrid(goal.x, goal.y, &gx, &gy)) return {};
  // 起点被困（膨胀后车体所在格不可行）：交给上层脱困，不在此处强行拉出
  if (!grid.feasibleCell(sx, sy)) return {};
  // 子目标落在障碍上：螺旋吸附到最近可行格（终点投影正好打在一堵墙上时会发生）
  if (!grid.feasibleCell(gx, gy)) {
    if (!grid::findFeasibleCellNear(grid, &gx, &gy, std::max(0, opt.goal_snap_radius))) {
      return {};
    }
  }

  const int W = grid.width;
  const size_t n = static_cast<size_t>(W) * static_cast<size_t>(grid.height);
  const size_t si = indexOf(sx, sy, W);
  const size_t gi = indexOf(gx, gy, W);

  Point2D goal_w;
  grid.gridToWorld(gx, gy, &goal_w.x, &goal_w.y);

  // 起终点同格：无需搜索
  if (si == gi) {
    if (std::hypot(goal_w.x - start.x, goal_w.y - start.y) < 1e-3) return {};
    return geom::toPath({start, goal_w});
  }

  // ---- 工作区：外部复用则零堆分配，否则用局部对象（便于单测）----
  Workspace local;
  Workspace& w = (ws_in != nullptr) ? *ws_in : local;
  w.ensure(n);
  w.last_iter = 0;

  // ---- 障碍软代价：仅 soft_k>0 时才构建截断距离带（关闭时零开销，行为同旧版）----
  const bool use_soft = opt.soft_k > 0.0 && opt.soft_sigma > 1e-6;
  double inv_sigma = 0.0;
  if (use_soft) {
    // 截断半径 = 3σ：exp(-3)≈0.05，再远的格软代价增量 <5%，视为足够远。
    const int max_cells =
        std::max(1, static_cast<int>(std::ceil(3.0 * opt.soft_sigma / grid.resolution)));
    inv_sigma = 1.0 / opt.soft_sigma;
    w.dist.ensure(n, max_cells);
    w.dist.build(grid);
  }
  const double res = grid.resolution;

  // ---- 一致性软代价：仅 consistency_k>0 且传入有效上帧路径时才构建 ----
  // 种子是上帧路径（已换算到当前 base 系），同样用截断距离带，机制与障碍带同构。
  const bool use_consist = opt.consistency_k > 0.0 && opt.consistency_sigma > 1e-6 &&
                           opt.prev_path != nullptr && !opt.prev_path->empty();
  double inv_sigma_p = 0.0;
  if (use_consist) {
    const int max_cells_p =
        std::max(1, static_cast<int>(std::ceil(3.0 * opt.consistency_sigma / grid.resolution)));
    inv_sigma_p = 1.0 / opt.consistency_sigma;
    w.prev.ensure(n, max_cells_p);
    w.prev.buildFromPoints(grid, *opt.prev_path);
  }
  // 代际自增即等价于「把所有 g_cost 重置为 inf」，但成本是 O(1) 而非 O(n)。
  // 溢出保护：按 10Hz 连续跑也要 6.8 年才会到 INT_MAX。
  if (w.gen >= INT_MAX) {
    std::fill(w.stamp.begin(), w.stamp.end(), 0);
    w.gen = 0;
  }
  const int gen = ++w.gen;
  auto getG = [&w, gen](size_t i) { return w.stamp[i] == gen ? w.g_cost[i] : kInf; };
  auto setG = [&w, gen](size_t i, double v) {
    w.stamp[i] = gen;
    w.g_cost[i] = v;
  };

  std::priority_queue<Cell, std::vector<Cell>, CellCmp> open;
  setG(si, 0.0);
  w.parent[si] = -1;
  const double h0 = heuristic(sx, sy, gx, gy);
  open.push(Cell{sx, sy, 0.0, h0, opt.w * h0});  // f = g + w*h（w=1 时即经典 A*）

  int iter = 0;
  bool found = false;
  while (!open.empty() && iter < opt.max_iter) {
    ++iter;
    const Cell cur = open.top();
    open.pop();

    const size_t ci = indexOf(cur.x, cur.y, W);
    if (ci == gi) {
      found = true;
      break;
    }
    if (cur.g > getG(ci) + 1e-9) continue;  // 堆中的过期条目

    for (int d = 0; d < 8; ++d) {
      const int nx = cur.x + kDx[d];
      const int ny = cur.y + kDy[d];
      if (!grid.inBounds(nx, ny)) continue;  // 越界绝不可通行

      const int8_t v = grid.valueAtCell(nx, ny);
      if (v >= kOccupyThreshold) continue;                // 占据
      if (v == kUnknown && !opt.allow_unknown) continue;  // 禁止穿越未知

      // 禁止穿角：对角移动要求两个正交邻格均可行，否则会擦过墙角
      if (opt.forbid_corner_cutting && kDx[d] != 0 && kDy[d] != 0) {
        if (!grid.feasibleCell(cur.x + kDx[d], cur.y) ||
            !grid.feasibleCell(cur.x, cur.y + kDy[d])) {
          continue;
        }
      }

      const size_t ni = indexOf(nx, ny, W);
      // unknown 乐观放行但代价略高 → 已知区优先，仍敢于探索未知
      double mul = (v == kUnknown) ? std::max(1.0, opt.unknown_cost) : 1.0;
      // 障碍软代价：离障碍越近进入代价越高 → 路径流向通道中央。因子恒 ≥1，
      // 与 unknown 倍率相乘后单格代价仍 ≥step → 欧氏启发仍可采纳。
      if (use_soft) {
        const double dm = static_cast<double>(w.dist.atIdx(ni)) * res;
        mul *= 1.0 + opt.soft_k * std::exp(-dm * inv_sigma);
      }
      // 一致性代价：偏离上帧路径越远越贵（饱和到 consistency_k）。只加不减，
      // 单格代价仍 ≥step → 欧氏启发仍可采纳。打破轴对称镜像的 f/g/h 全相等僵局。
      double step_cost = kStep[d] * mul;
      if (use_consist) {
        const double dp = static_cast<double>(w.prev.atIdx(ni)) * res;
        step_cost += opt.consistency_k * (1.0 - std::exp(-dp * inv_sigma_p));
      }
      const double ng = cur.g + step_cost;
      if (ng < getG(ni) - 1e-9) {
        setG(ni, ng);
        w.parent[ni] = static_cast<int>(ci);
        const double nh = heuristic(nx, ny, gx, gy);
        open.push(Cell{nx, ny, ng, nh, ng + opt.w * nh});
      }
    }
  }
  w.last_iter = iter;  // 记录本次扩展数（诊断 / tie-break 回归测试用）
  if (!found) return {};

  // ---- 回溯：栅格索引 → 格中心世界坐标 ----
  w.cells.clear();
  for (size_t ci = gi;;) {
    const int cx = static_cast<int>(ci % static_cast<size_t>(W));
    const int cy = static_cast<int>(ci / static_cast<size_t>(W));
    Point2D p;
    grid.gridToWorld(cx, cy, &p.x, &p.y);
    w.cells.push_back(p);
    if (ci == si) break;
    if (w.parent[ci] < 0) return {};  // 防御：父链断裂
    ci = static_cast<size_t>(w.parent[ci]);
  }
  std::reverse(w.cells.begin(), w.cells.end());
  w.cells.front() = start;  // 首点用精确车位而非格中心
  w.cells.back() = goal_w;  // 末点用（可能已吸附的）子目标格中心

  const double sp = spacing > 0.0 ? spacing : grid.resolution;

  // ---- 后处理：LOS 拉直 → 平滑 → 按段加密采样 → 组装 Path ----
  // 平滑放在拉直之后、加密之前：此时拐点还是显式顶点，倒角才能识别得出。
  w.straight = grid::shortcut(grid, w.cells);

  smooth::Options so;
  so.fillet_radius = opt.smooth_fillet_radius;
  so.laplacian_iters = opt.smooth_laplacian_iters;
  so.laplacian_lambda = opt.smooth_laplacian_lambda;
  so.spacing = sp;
  so.shrink_retry = opt.smooth_shrink_retry;
  w.smoothed = smooth::run(grid, w.straight, so);

  // 必须用 resampleKeepCorners 而非 resampleByArc：shortcut/平滑出来的每一段都已
  // 验证无碰撞，但按全局弧长打点会让相邻采样点跨过拐点，其连线切掉拐角内侧，
  // 从而削进障碍（U 形墙臂端处必现）。保留拐点即可保证加密后仍然无碰撞。
  const auto resampled = geom::resampleKeepCorners(w.smoothed, sp);
  if (resampled.size() < 2) return {};
  return geom::toPath(resampled, opt.curvature_baseline);
}

}  // namespace astar
}  // namespace unk
