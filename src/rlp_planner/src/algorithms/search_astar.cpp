#include "rlp_planner/algorithms/search_astar.h"

#include <algorithm>
#include <cmath>
#include <queue>
#include <vector>

#include "rlp_common/geometry_util.h"
#include "rlp_planner/candidate_gen.h"

namespace rlp {
namespace planner {

// ── 内部辅助（仅本文件使用）──────────────────────────────────────
namespace {

struct Cell {
  int x, y;
  double g;  // 起点到当前格的已知代价
  double f;  // g + 启发式
};

struct CellCmp {
  bool operator()(const Cell& a, const Cell& b) const { return a.f > b.f; }
};

inline int idx(int x, int y, int w) { return y * w + x; }

// 欧几里得启发式（对 8-连通栅格是可采纳的）
inline double heuristic(int x1, int y1, int x2, int y2) {
  return std::hypot(x1 - x2, y1 - y2);
}

// 在目标格周围螺旋搜索最近的自由格（目标格被占据时的回退）
bool findFreeCellNear(const GridMap& map, int& gx, int& gy, int max_radius) {
  const int W = map.width, H = map.height;
  for (int r = 1; r <= max_radius; ++r) {
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        if (std::abs(dx) != r && std::abs(dy) != r) continue;  // 只查外圈
        const int nx = gx + dx, ny = gy + dy;
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        if (map.data[static_cast<size_t>(ny) * W + nx] < 65) {
          gx = nx;
          gy = ny;
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace

// ── 公共接口 ─────────────────────────────────────────────────────

SearchAStarAlg::SearchAStarAlg(const PlannerParams& p) : p_(p) {}

Path SearchAStarAlg::directPlan(const PlanningContext& ctx) const {
  const GridMap& map = ctx.input->map;
  if (map.empty() || !ctx.input->goal_valid) return {};

  const double res = map.resolution;
  const int W = map.width, H = map.height;
  if (W <= 0 || H <= 0 || res <= 0.0) return {};

  // ---- 起点：车辆位置 (0,0) → 栅格坐标 ----
  const int sx = static_cast<int>(std::floor((0.0 - map.origin_x) / res));
  const int sy = static_cast<int>(std::floor((0.0 - map.origin_y) / res));
  if (sx < 0 || sy < 0 || sx >= W || sy >= H) return {};

  // ---- 目标点：终点方向上 min(lookahead, goal_dist) 处 ----
  const double la = candidate_gen::lookaheadLength(p_, ctx.input->current_speed);
  const double goal_dist = std::hypot(ctx.input->goal.x, ctx.input->goal.y);
  if (goal_dist < 1e-3) return {};

  const double scale = std::min(la, goal_dist) / goal_dist;
  const double tx = ctx.input->goal.x * scale;
  const double ty = ctx.input->goal.y * scale;

  int gx = static_cast<int>(std::floor((tx - map.origin_x) / res));
  int gy = static_cast<int>(std::floor((ty - map.origin_y) / res));
  gx = std::max(0, std::min(W - 1, gx));
  gy = std::max(0, std::min(H - 1, gy));

  // 目标格被占据 → 螺旋搜索最近自由格
  if (map.data[static_cast<size_t>(gy) * W + gx] >= 65) {
    if (!findFreeCellNear(map, gx, gy, 5)) return {};
  }

  // ---- A* 搜索（8-连通）----
  const int n = W * H;
  std::vector<double> g_cost(n, 1e18);
  std::vector<int> parent(n, -1);
  std::priority_queue<Cell, std::vector<Cell>, CellCmp> open;

  const int si = idx(sx, sy, W);
  const int gi = idx(gx, gy, W);
  g_cost[si] = 0.0;
  open.push({sx, sy, 0.0, heuristic(sx, sy, gx, gy)});

  // 8 方向偏移与对应步长（对角线 √2）
  static const int dx8[] = {-1, 0, 1, -1, 1, -1, 0, 1};
  static const int dy8[] = {-1, -1, -1, 0, 0, 1, 1, 1};
  static const double step8[] = {1.414, 1.0, 1.414, 1.0, 1.0, 1.414, 1.0, 1.414};

  int iter = 0;
  const int max_iter = p_.astar_max_iter;
  bool found = false;

  while (!open.empty() && iter < max_iter) {
    ++iter;
    const Cell cur = open.top();
    open.pop();

    const int ci = idx(cur.x, cur.y, W);
    if (ci == gi) {
      found = true;
      break;
    }
    if (cur.g > g_cost[ci] + 1e-9) continue;  // 过期条目

    for (int d = 0; d < 8; ++d) {
      const int nx = cur.x + dx8[d];
      const int ny = cur.y + dy8[d];
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;

      const int ni = idx(nx, ny, W);
      // 占据检查
      if (map.data[static_cast<size_t>(ny) * W + nx] >= 65) continue;

      // 走廊软约束：走廊外的栅格增加额外代价
      double extra = 0.0;
      if (ctx.corridor != nullptr && ctx.corridor->valid()) {
        const double wx = map.origin_x + (nx + 0.5) * res;
        const double wy = map.origin_y + (ny + 0.5) * res;
        const double dist = pointToPolylineDistance({wx, wy}, ctx.corridor->centerline);
        if (dist > ctx.corridor->half_width) {
          extra = p_.w_offroad * (dist - ctx.corridor->half_width);
        }
      }

      const double ng = cur.g + step8[d] + extra;
      if (ng < g_cost[ni] - 1e-9) {
        g_cost[ni] = ng;
        parent[ni] = ci;
        open.push({nx, ny, ng, ng + heuristic(nx, ny, gx, gy)});
      }
    }
  }

  if (!found) return {};

  // ---- 路径回溯（栅格坐标 → 世界坐标）----
  std::vector<Point2D> world_pts;
  for (int ci = gi; ci != -1; ci = parent[ci]) {
    const int cx = ci % W;
    const int cy = ci / W;
    world_pts.push_back({map.origin_x + (cx + 0.5) * res,
                         map.origin_y + (cy + 0.5) * res});
  }
  std::reverse(world_pts.begin(), world_pts.end());

  // ---- 后处理：等距重采样 + 拼接车辆原点 ----
  if (world_pts.size() < 2) return {};
  const auto resampled = resampleByArc(world_pts, p_.path_spacing);
  if (resampled.size() < 2) return {};

  // 拼接原点 (0,0)，跳过离原点过近的点
  std::vector<Point2D> full;
  full.reserve(resampled.size() + 1);
  full.push_back({0.0, 0.0});
  for (const auto& p : resampled) {
    if (std::hypot(p.x, p.y) > 0.2) full.push_back(p);
  }

  return toPath(full);
}

}  // namespace planner
}  // namespace rlp
