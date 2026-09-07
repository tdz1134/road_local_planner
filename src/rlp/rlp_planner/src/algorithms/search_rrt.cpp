#include "rlp_planner/algorithms/search_rrt.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <vector>

#include "rlp_common/geometry_util.h"
#include "rlp_planner/candidate_gen.h"

namespace rlp {
namespace planner {

// ── 内部辅助（仅本文件使用）──────────────────────────────────────
namespace {

struct Node {
  double x, y;
  int parent;  // -1 = 根节点
};

// 检查线段 (x1,y1)→(x2,y2) 是否与占据栅格碰撞
// 沿线段按 resolution 步长采样，任一点占据则碰撞
bool segmentFree(const GridMap& map, double x1, double y1, double x2, double y2) {
  const double dx = x2 - x1, dy = y2 - y1;
  const double len = std::hypot(dx, dy);
  if (len < 1e-6) return !map.occupiedAt(x1, y1);
  const int steps = std::max(1, static_cast<int>(std::ceil(len / map.resolution)));
  for (int i = 0; i <= steps; ++i) {
    const double t = static_cast<double>(i) / steps;
    if (map.occupiedAt(x1 + dx * t, y1 + dy * t)) return false;
  }
  return true;
}

// 在树中找离 (x,y) 最近的节点
int nearestNode(const std::vector<Node>& tree, double x, double y) {
  int best = 0;
  double best_d = 1e18;
  for (int i = 0; i < static_cast<int>(tree.size()); ++i) {
    const double d = std::hypot(tree[i].x - x, tree[i].y - y);
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

}  // namespace

// ── 公共接口 ─────────────────────────────────────────────────────

SearchRRTAlg::SearchRRTAlg(const PlannerParams& p) : p_(p) {}

Path SearchRRTAlg::directPlan(const PlanningContext& ctx) const {
  const GridMap& map = ctx.input->map;
  if (map.empty() || !ctx.input->goal_valid) return {};

  // ---- 目标点：终点方向上 min(lookahead, goal_dist) 处 ----
  const double la = candidate_gen::lookaheadLength(p_, ctx.input->current_speed);
  const double goal_dist = std::hypot(ctx.input->goal.x, ctx.input->goal.y);
  if (goal_dist < 1e-3) return {};

  const double scale = std::min(la, goal_dist) / goal_dist;
  const double gx = ctx.input->goal.x * scale;
  const double gy = ctx.input->goal.y * scale;

  // 起点必须在栅格内且自由
  if (map.occupiedAt(0.0, 0.0)) return {};

  // ---- RRT 搜索 ----
  std::vector<Node> tree;
  tree.push_back({0.0, 0.0, -1});  // 根节点 = 车辆位置

  std::mt19937 rng(42);  // 固定种子，保证测试可复现
  std::uniform_real_distribution<double> unif(0.0, 1.0);

  // 栅格范围（世界坐标）
  const double x_min = map.origin_x;
  const double x_max = map.origin_x + map.width * map.resolution;
  const double y_min = map.origin_y;
  const double y_max = map.origin_y + map.height * map.resolution;

  const double step = p_.rrt_step_size;
  const double goal_threshold = step * 1.5;  // 距目标小于此值视为到达
  int goal_node = -1;

  for (int iter = 0; iter < p_.rrt_max_iter; ++iter) {
    // ---- 采样 ----
    double sx, sy;
    const double r = unif(rng);

    if (r < p_.rrt_goal_bias) {
      // 朝终点采样
      sx = gx;
      sy = gy;
    } else if (ctx.corridor != nullptr && ctx.corridor->valid() && r < p_.rrt_goal_bias + 0.4) {
      // 走廊内采样：沿中线随机位置 + 随机横向偏移
      const auto& cl = ctx.corridor->centerline;
      if (cl.size() >= 2) {
        const int ci = static_cast<int>(unif(rng) * cl.size()) % cl.size();
        const double lat = (unif(rng) * 2.0 - 1.0) * ctx.corridor->half_width * 0.8;
        // 简化：直接用中线点 + 横向偏移（不精确计算法线，v1 够用）
        sx = cl[ci].x;
        sy = cl[ci].y + lat;
      } else {
        sx = x_min + unif(rng) * (x_max - x_min);
        sy = y_min + unif(rng) * (y_max - y_min);
      }
    } else {
      // 纯随机采样
      sx = x_min + unif(rng) * (x_max - x_min);
      sy = y_min + unif(rng) * (y_max - y_min);
    }

    // ---- 找最近节点 ----
    const int near = nearestNode(tree, sx, sy);
    const double dx = sx - tree[near].x;
    const double dy = sy - tree[near].y;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-6) continue;

    // ---- 扩展：从最近节点朝采样点走 step 距离 ----
    const double t = std::min(1.0, step / dist);
    const double nx = tree[near].x + dx * t;
    const double ny = tree[near].y + dy * t;

    // 新点必须在栅格内且自由
    if (nx < x_min || nx > x_max || ny < y_min || ny > y_max) continue;
    if (map.occupiedAt(nx, ny)) continue;

    // 线段碰撞检查
    if (!segmentFree(map, tree[near].x, tree[near].y, nx, ny)) continue;

    // ---- 加入树 ----
    tree.push_back({nx, ny, near});

    // 检查是否到达目标
    if (std::hypot(nx - gx, ny - gy) < goal_threshold) {
      goal_node = static_cast<int>(tree.size()) - 1;
      break;
    }
  }

  if (goal_node < 0) return {};

  // ---- 路径回溯 ----
  std::vector<Point2D> world_pts;
  for (int ci = goal_node; ci != -1; ci = tree[ci].parent) {
    world_pts.push_back({tree[ci].x, tree[ci].y});
  }
  std::reverse(world_pts.begin(), world_pts.end());

  // ---- 后处理：等距重采样 + 拼接车辆原点 ----
  if (world_pts.size() < 2) return {};
  const auto resampled = resampleByArc(world_pts, p_.path_spacing);
  if (resampled.size() < 2) return {};

  std::vector<Point2D> full;
  full.reserve(resampled.size() + 1);
  full.push_back({0.0, 0.0});
  for (const auto& pt : resampled) {
    if (std::hypot(pt.x, pt.y) > 0.2) full.push_back(pt);
  }

  return toPath(full);
}

}  // namespace planner
}  // namespace rlp
