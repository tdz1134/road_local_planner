#include "unk_nav/grid_util.h"

#include <algorithm>
#include <climits>
#include <cmath>

namespace unk {
namespace grid {

// ── 内部辅助（仅本文件使用）──────────────────────────────────────
namespace {

// 单次 LOS 拉直的向后跳跃上限。栅格 A* 的原始路径点很多，不限制会让
// shortcut 退化成 O(n^2) 次线段检查；64 格 @0.05m = 3.2m 已远超实际需要。
constexpr int kMaxShortcutSkip = 64;

inline size_t indexOf(const GridMap& g, int gx, int gy) {
  return static_cast<size_t>(gy) * static_cast<size_t>(g.width) +
         static_cast<size_t>(gx);
}

}  // namespace

// ── 公共接口（在 grid_util.h 中声明）─────────────────────────────

std::vector<std::pair<int, int>> diskKernel(double radius, double resolution) {
  std::vector<std::pair<int, int>> k;
  if (radius <= 0.0 || resolution <= 0.0) {
    k.push_back({0, 0});
    return k;
  }
  const double rr = radius / resolution;
  const int r = static_cast<int>(std::ceil(rr));
  const double r2 = rr * rr;
  k.reserve(static_cast<size_t>(kPi * r2) + 1);
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      if (static_cast<double>(dx * dx + dy * dy) <= r2 + 1e-9) {
        k.push_back({dx, dy});
      }
    }
  }
  return k;
}

GridMap inflate(const GridMap& in, double radius, bool inflate_unknown) {
  GridMap out = in;
  if (in.empty() || radius <= 0.0) return out;

  const auto kern = diskKernel(radius, in.resolution);
  const int W = in.width, H = in.height;

  // 先收集源占据格再统一写入：避免边写边读造成膨胀距离无限连锁扩散
  std::vector<int> src;
  for (int gy = 0; gy < H; ++gy) {
    for (int gx = 0; gx < W; ++gx) {
      if (in.data[indexOf(in, gx, gy)] >= kOccupyThreshold) {
        src.push_back(gy * W + gx);
      }
    }
  }

  for (const int cell : src) {
    const int cx = cell % W, cy = cell / W;
    for (const auto& d : kern) {
      const int nx = cx + d.first, ny = cy + d.second;
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      int8_t& v = out.data[indexOf(out, nx, ny)];
      if (v >= kOccupyThreshold) continue;              // 已是障碍
      if (v == kUnknown && !inflate_unknown) continue;  // 保持乐观探索语义
      v = kOccupied;
    }
  }
  return out;
}

void clearFootprint(GridMap* g, double cx, double cy, double radius,
                    const GridMap* raw) {
  if (g == nullptr || g->empty() || radius <= 0.0) return;
  int gx0 = 0, gy0 = 0;
  if (!g->worldToGrid(cx, cy, &gx0, &gy0)) return;  // 车体不在窗口内，异常输入
  // raw 必须是与 *g 同规格的栅格（即 inflate 的入参），否则无从分辨「膨胀出来的」
  // 和「本来就是」障碍 —— 这时退回无条件清空，不静默错判。
  const bool use_raw =
      raw != nullptr && !raw->empty() && raw->width == g->width &&
      raw->height == g->height &&
      std::fabs(raw->resolution - g->resolution) < 1e-9 &&
      std::fabs(raw->origin_x - g->origin_x) < 1e-9 &&
      std::fabs(raw->origin_y - g->origin_y) < 1e-9;
  const auto kern = diskKernel(radius, g->resolution);
  for (const auto& d : kern) {
    const int nx = gx0 + d.first, ny = gy0 + d.second;
    if (!g->inBounds(nx, ny)) continue;
    // 保留 lidar 真打到的障碍：足迹洞只该抹掉膨胀带，不该抹掉墙本身
    if (use_raw && (*raw).data[indexOf(*raw, nx, ny)] >= kOccupyThreshold) continue;
    g->data[indexOf(*g, nx, ny)] = kFree;
  }
}

bool segmentFree(const GridMap& g, double x1, double y1, double x2, double y2) {
  if (g.empty()) return false;
  const double dx = x2 - x1, dy = y2 - y1;
  const double len = std::hypot(dx, dy);
  if (len < 1e-9) return !g.blockedAt(x1, y1);
  const double step = std::max(g.resolution, 1e-6);
  const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    if (g.blockedAt(x1 + dx * t, y1 + dy * t)) return false;
  }
  return true;
}

bool polylineFree(const GridMap& g, const std::vector<Point2D>& pts) {
  if (g.empty() || pts.empty()) return false;
  for (const auto& p : pts) {
    if (g.blockedAt(p.x, p.y)) return false;
  }
  for (size_t i = 1; i < pts.size(); ++i) {
    if (!segmentFree(g, pts[i - 1].x, pts[i - 1].y, pts[i].x, pts[i].y)) return false;
  }
  return true;
}

std::vector<Point2D> shortcut(const GridMap& g, const std::vector<Point2D>& pts) {
  if (pts.size() <= 2) return pts;
  std::vector<Point2D> out;
  out.reserve(pts.size());
  out.push_back(pts.front());

  size_t anchor = 0;
  while (anchor + 1 < pts.size()) {
    // 从 anchor 往后找视线无阻挡的最远点；限制搜索窗口避免 O(n^2)
    const size_t hi = std::min(pts.size() - 1, anchor + static_cast<size_t>(kMaxShortcutSkip));
    size_t far = anchor + 1;
    for (size_t j = hi; j > anchor + 1; --j) {
      if (segmentFree(g, pts[anchor].x, pts[anchor].y, pts[j].x, pts[j].y)) {
        far = j;
        break;
      }
    }
    out.push_back(pts[far]);
    anchor = far;
  }
  return out;
}

bool findFeasibleCellNear(const GridMap& g, int* gx, int* gy, int max_radius) {
  if (gx == nullptr || gy == nullptr || g.empty()) return false;
  const int ox = *gx, oy = *gy;
  const int W = g.width, H = g.height;
  if (g.feasibleCell(ox, oy)) return true;  // 本来就可行

  for (int r = 1; r <= max_radius; ++r) {
    // 只扫半径 r 的外圈，命中即返回 → 保证找到的是最近可行格
    for (int dx = -r; dx <= r; ++dx) {
      for (int dy = -r; dy <= r; ++dy) {
        if (std::abs(dx) != r && std::abs(dy) != r) continue;
        const int nx = ox + dx, ny = oy + dy;
        if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
        if (g.feasibleCell(nx, ny)) {
          *gx = nx;
          *gy = ny;
          return true;
        }
      }
    }
  }
  return false;
}

// ── DistanceField：截断式障碍距离带 ─────────────────────────────

namespace {

// 代际自增 + 溢出保护，返回新代际。自增即等价于「把整幅 stamp 清零」，成本 O(1)。
int advanceGen(DistanceField& f) {
  if (f.gen >= INT_MAX) {
    std::fill(f.stamp.begin(), f.stamp.end(), 0);
    f.gen = 0;
  }
  return ++f.gen;
}

// 从 f.queue[0..tail) 的已入队种子向外 8-连通 BFS，只扩展到截断半径。
// 每格至多入队一次（stamp 守卫）→ tail <= 总格数，不越 queue 容量。
void bfsExpand(DistanceField& f, int W, int H, int my_gen, int tail) {
  static const int kDx[8] = {1, 1, 1, 0, 0, -1, -1, -1};
  static const int kDy[8] = {1, 0, -1, 1, -1, 1, 0, -1};
  int head = 0;
  while (head < tail) {
    const int idx = f.queue[head++];
    const int cx = idx % W, cy = idx / W;
    const int cd = f.dist[idx];
    if (cd >= f.max_dist_cells) continue;  // 到带边界即停，不再外扩
    for (int d = 0; d < 8; ++d) {
      const int nx = cx + kDx[d], ny = cy + kDy[d];
      if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
      const size_t ni = static_cast<size_t>(ny) * static_cast<size_t>(W) +
                        static_cast<size_t>(nx);
      if (f.stamp[ni] == my_gen) continue;  // 已赋值（更近或等距）
      f.stamp[ni] = my_gen;
      f.dist[ni] = static_cast<uint8_t>(cd + 1);
      f.queue[tail++] = static_cast<int>(ni);
    }
  }
}

}  // namespace

void DistanceField::ensure(size_t n, int max_cells) {
  if (dist.size() == n && max_dist_cells == max_cells) return;
  dist.assign(n, 0);
  stamp.assign(n, 0);
  queue.assign(n, 0);
  gen = 0;
  max_dist_cells = max_cells;
}

void DistanceField::build(const GridMap& g) {
  if (g.empty() || max_dist_cells <= 0) return;  // 禁用或空图：不构建
  const int W = g.width, H = g.height;
  const int my_gen = advanceGen(*this);

  int tail = 0;
  // 多种子：所有占据格入队，dist=0，stamp 置本代际。
  // 这一步是全图扫描（读 int8），是唯一与总格数线性相关的开销。
  for (int gy = 0; gy < H; ++gy) {
    const size_t row = static_cast<size_t>(gy) * static_cast<size_t>(W);
    for (int gx = 0; gx < W; ++gx) {
      const size_t idx = row + static_cast<size_t>(gx);
      if (g.data[idx] >= kOccupyThreshold) {
        stamp[idx] = my_gen;
        dist[idx] = 0;
        queue[tail++] = static_cast<int>(idx);
      }
    }
  }
  bfsExpand(*this, W, H, my_gen, tail);
}

void DistanceField::buildFromPoints(const GridMap& g,
                                    const std::vector<Point2D>& pts) {
  if (g.empty() || max_dist_cells <= 0 || pts.empty()) return;
  const int W = g.width, H = g.height;
  const int my_gen = advanceGen(*this);

  int tail = 0;
  // 种子：折线每个点所在的格（同格去重）。窗口外的点忽略。
  for (const auto& p : pts) {
    int gx = 0, gy = 0;
    if (!g.worldToGrid(p.x, p.y, &gx, &gy)) continue;
    const size_t idx = static_cast<size_t>(gy) * static_cast<size_t>(W) +
                       static_cast<size_t>(gx);
    if (stamp[idx] == my_gen) continue;  // 同格已入队
    stamp[idx] = my_gen;
    dist[idx] = 0;
    queue[tail++] = static_cast<int>(idx);
  }
  if (tail == 0) return;  // 全部落在窗口外
  bfsExpand(*this, W, H, my_gen, tail);
}

}  // namespace grid
}  // namespace unk
