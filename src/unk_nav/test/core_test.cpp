// 离线单测：裸 main + 自定义断言，不依赖 gtest / ROS（yaml-cpp 仅 params_io 组用到）。
// 构建后直接运行：./unk_nav_test
//
// 覆盖范围（批次 2~4 + 配置加载）：
//   geom_util —— 角度归一化、坐标变换、弧长、重采样、裁剪、点线距、曲率
//   grid_util —— 膨胀核、障碍膨胀、足迹清空、视线检查、LOS 拉直、螺旋找格
//   astar     —— 直路 / 带缺口墙 / 实心墙 / U 形墙（凹障碍）/ 全 unknown /
//                越界子目标 / 起点被困 / 迭代护栏 / 禁止穿角 / 膨胀闭口
//   params_io —— YAML 加载：正常/缺省保留/未知 key/类型错/文件不存在
//   controller —— 纯跟踪前视点选取（含车已前移时）/ 弦收缩 / 限幅 / 指令斜率限幅
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

#include "unk_nav/astar.h"
#include "unk_nav/behavior_fsm.h"
#include "unk_nav/controller.h"
#include "unk_nav/geom_util.h"
#include "unk_nav/grid_util.h"
#include "unk_nav/nav_core.h"
#include "unk_nav/params_io.h"
#include "unk_nav/path_smooth.h"
#include "unk_nav/road_follow.h"
#include "unk_nav/speed_planner.h"
#include "unk_nav/subgoal.h"
#include "unk_nav/types.h"

// ── 测试脚手架 ───────────────────────────────────────────────────
namespace {

int g_pass = 0;
int g_fail = 0;

void group(const char* name) { std::printf("\n== %s ==\n", name); }

std::string f2s(double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.4f", v);
  return std::string(buf);
}

void check(bool cond, const std::string& what) {
  if (cond) {
    ++g_pass;
    std::printf("  [ok]   %s\n", what.c_str());
  } else {
    ++g_fail;
    std::printf("  [FAIL] %s\n", what.c_str());
  }
}

void checkNear(double got, double want, double tol, const std::string& what) {
  check(std::fabs(got - want) <= tol,
        what + "  (got " + f2s(got) + ", want " + f2s(want) + " +/-" + f2s(tol) + ")");
}

// ---- 合成栅格工具 ----

// 生成 size_m × size_m 的正方形窗口，车体在原点，全部填 fill
unk::GridMap makeGrid(double size_m, double res, int8_t fill) {
  unk::GridMap g;
  g.resolution = res;
  g.width = static_cast<int>(std::round(size_m / res));
  g.height = g.width;
  g.origin_x = -0.5 * g.width * res;
  g.origin_y = -0.5 * g.height * res;
  g.data.assign(static_cast<size_t>(g.width) * static_cast<size_t>(g.height), fill);
  return g;
}

void setCell(unk::GridMap* g, int gx, int gy, int8_t v) {
  g->data[static_cast<size_t>(gy) * static_cast<size_t>(g->width) +
          static_cast<size_t>(gx)] = v;
}

// 在世界坐标矩形 [x0,x1] × [y0,y1] 内按格中心填值
void fillRect(unk::GridMap* g, double x0, double y0, double x1, double y1, int8_t v) {
  for (int gy = 0; gy < g->height; ++gy) {
    for (int gx = 0; gx < g->width; ++gx) {
      double wx = 0.0, wy = 0.0;
      g->gridToWorld(gx, gy, &wx, &wy);
      if (wx >= x0 && wx <= x1 && wy >= y0 && wy <= y1) setCell(g, gx, gy, v);
    }
  }
}

// 沿路径稠密检查是否穿过障碍（比只查顶点严格）
bool pathCollides(const unk::GridMap& g, const unk::Path& p) {
  for (size_t i = 1; i < p.size(); ++i) {
    if (!unk::grid::segmentFree(g, p[i - 1].p.x, p[i - 1].p.y, p[i].p.x, p[i].p.y)) {
      return true;
    }
  }
  return false;
}

double pathMaxY(const unk::Path& p) {
  double m = 0.0;
  for (const auto& pp : p) m = std::max(m, std::fabs(pp.p.y));
  return m;
}

// 采样间距是否合法：resampleKeepCorners 按段等分，间距 <= spacing 且不过密
bool spacingOk(const unk::Path& p, double spacing) {
  if (p.size() < 2) return false;
  for (size_t i = 1; i < p.size(); ++i) {
    const double d = p[i].s - p[i - 1].s;
    if (d > spacing + 1e-9 || d < spacing * 0.5) return false;
  }
  return true;
}

const double kSpacing = 0.05;

// ── geom_util ────────────────────────────────────────────────────

void testGeom() {
  group("geom_util");
  using unk::geom::normalizeAngle;

  checkNear(normalizeAngle(3.0 * unk::kPi), unk::kPi, 1e-9, "normalizeAngle(3pi) -> pi");
  checkNear(normalizeAngle(-3.0 * unk::kPi), unk::kPi, 1e-9, "normalizeAngle(-3pi) -> pi");
  checkNear(normalizeAngle(0.5), 0.5, 1e-9, "normalizeAngle(0.5) 不变");

  // 坐标变换：车在 (1,1) 朝 +y，全局 (1,2) 应在车正前方 1m
  const unk::Pose2D veh{1.0, 1.0, unk::kPi * 0.5};
  const unk::Point2D b = unk::geom::globalToBase({1.0, 2.0}, veh);
  checkNear(b.x, 1.0, 1e-9, "globalToBase 前向分量");
  checkNear(b.y, 0.0, 1e-9, "globalToBase 横向分量");

  // 往返互逆（含任意航向与平移）
  const unk::Pose2D v2{2.0, -1.0, 0.7};
  const unk::Point2D q{-3.5, 4.25};
  const unk::Point2D rt = unk::geom::baseToGlobal(unk::geom::globalToBase(q, v2), v2);
  checkNear(rt.x, q.x, 1e-9, "globalToBase/baseToGlobal 往返互逆 x");
  checkNear(rt.y, q.y, 1e-9, "globalToBase/baseToGlobal 往返互逆 y");

  // 弧长：3-4-5 折线
  const std::vector<unk::Point2D> poly{{0.0, 0.0}, {3.0, 0.0}, {3.0, 4.0}};
  checkNear(unk::geom::polylineLength(poly), 7.0, 1e-9, "polylineLength(3-4-5)");

  // 等距重采样：10m 直线按 1m 采 → 11 点，长度守恒
  const std::vector<unk::Point2D> line{{0.0, 0.0}, {10.0, 0.0}};
  const auto rs = unk::geom::resampleByArc(line, 1.0);
  check(rs.size() == 11, "resampleByArc 点数=11  (got " + std::to_string(rs.size()) + ")");
  checkNear(unk::geom::polylineLength(rs), 10.0, 1e-9, "resampleByArc 长度守恒");
  checkNear(rs.back().x, 10.0, 1e-9, "resampleByArc 末点落在终点");

  // 裁剪
  const auto cl = unk::geom::clipByArc(line, 3.0);
  checkNear(cl.back().x, 3.0, 1e-9, "clipByArc 截断到 3m");

  // resampleKeepCorners：必须保留拐点（否则加密采样的弦会切掉拐角内侧）
  const std::vector<unk::Point2D> corner{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  const auto kc = unk::geom::resampleKeepCorners(corner, 0.3);
  bool corner_kept = false;
  double kc_max_step = 0.0;
  for (size_t i = 0; i < kc.size(); ++i) {
    if (std::fabs(kc[i].x - 1.0) < 1e-9 && std::fabs(kc[i].y) < 1e-9) corner_kept = true;
    if (i > 0) {
      kc_max_step = std::max(kc_max_step, std::hypot(kc[i].x - kc[i - 1].x,
                                                     kc[i].y - kc[i - 1].y));
    }
  }
  check(corner_kept, "resampleKeepCorners 保留拐点 (1,0)");
  check(kc_max_step <= 0.3 + 1e-9, "resampleKeepCorners 间距不超过 spacing (max=" +
                                       f2s(kc_max_step) + ")");
  checkNear(kc.back().x, 1.0, 1e-9, "resampleKeepCorners 末点=折线末点 x");
  checkNear(kc.back().y, 1.0, 1e-9, "resampleKeepCorners 末点=折线末点 y");
  // 对照：resampleByArc 按全局弧长打点，拐点会被跳过
  const auto arc = unk::geom::resampleByArc(corner, 0.3);
  bool arc_has_corner = false;
  for (const auto& q : arc) {
    if (std::fabs(q.x - 1.0) < 1e-9 && std::fabs(q.y) < 1e-9) arc_has_corner = true;
  }
  check(!arc_has_corner, "对照组 resampleByArc 确实丢掉拐点（说明上面那条断言有意义）");

  // 点到线段：垂足在段内 / 段外夹取
  checkNear(unk::geom::pointToSegmentDistance({5.0, 3.0}, {0.0, 0.0}, {10.0, 0.0}),
            3.0, 1e-9, "pointToSegmentDistance 垂足在段内");
  checkNear(unk::geom::pointToSegmentDistance({-5.0, 0.0}, {0.0, 0.0}, {10.0, 0.0}),
            5.0, 1e-9, "pointToSegmentDistance 段外夹取到端点");

  // 点到折线（注意 (3,1) 落在第二段上，距离为 0，故取段外点）
  checkNear(unk::geom::pointToPolylineDistance({4.0, 1.0}, poly), 1.0, 1e-9,
            "pointToPolylineDistance");

  // 曲率：共线为 0，左转为正
  checkNear(unk::geom::threePointCurvature({0, 0}, {1, 0}, {2, 0}), 0.0, 1e-9,
            "threePointCurvature 共线=0");
  check(unk::geom::threePointCurvature({0, 0}, {1, 0}, {1, 1}) > 0.0,
        "threePointCurvature 左转为正");
  check(unk::geom::threePointCurvature({0, 0}, {1, 0}, {1, -1}) < 0.0,
        "threePointCurvature 右转为负");

  // toPath：弧长单调且末点等于总长
  const unk::Path p = unk::geom::toPath(poly);
  check(p.size() == 3, "toPath 点数不变");
  checkNear(p.back().s, 7.0, 1e-9, "toPath 末点弧长=总长");
  check(p[0].s <= p[1].s && p[1].s <= p[2].s, "toPath 弧长单调");
}

// ── grid_util ────────────────────────────────────────────────────

void testGrid() {
  group("grid_util");

  // 膨胀核
  const auto kern = unk::grid::diskKernel(0.25, 0.1);
  bool has_0_0 = false, has_2_0 = false, has_2_2 = false, has_3_0 = false;
  for (const auto& d : kern) {
    if (d.first == 0 && d.second == 0) has_0_0 = true;
    if (d.first == 2 && d.second == 0) has_2_0 = true;
    if (d.first == 2 && d.second == 2) has_2_2 = true;
    if (d.first == 3 && d.second == 0) has_3_0 = true;
  }
  check(has_0_0, "diskKernel 含 (0,0)");
  check(has_2_0, "diskKernel 含 (2,0)：4 <= 6.25");
  check(!has_2_2, "diskKernel 不含 (2,2)：8 > 6.25");
  check(!has_3_0, "diskKernel 不含 (3,0)：9 > 6.25");

  // 膨胀：单点障碍向外扩张，unknown 默认不受影响
  unk::GridMap g = makeGrid(2.0, 0.1, unk::kFree);
  setCell(&g, 10, 10, unk::kOccupied);
  const unk::GridMap inf = unk::grid::inflate(g, 0.25);
  check(inf.valueAtCell(12, 10) >= unk::kOccupyThreshold, "膨胀 2 格处为障碍");
  check(inf.valueAtCell(13, 10) == unk::kFree, "膨胀 3 格处仍为 free");
  check(inf.valueAtCell(10, 10) >= unk::kOccupyThreshold, "源障碍保留");

  unk::GridMap gu = makeGrid(2.0, 0.1, unk::kUnknown);
  setCell(&gu, 10, 10, unk::kOccupied);
  const unk::GridMap inf_u = unk::grid::inflate(gu, 0.25, false);
  check(inf_u.valueAtCell(11, 10) == unk::kUnknown, "默认不把 unknown 膨胀成障碍");
  const unk::GridMap inf_uk = unk::grid::inflate(gu, 0.25, true);
  check(inf_uk.valueAtCell(11, 10) >= unk::kOccupyThreshold,
        "inflate_unknown=true 时 unknown 被膨胀");

  // 足迹清空：膨胀后车脚下必须恢复 free，否则 A* 起点即死锁
  unk::GridMap gf = unk::grid::inflate(g, 0.25);
  unk::grid::clearFootprint(&gf, 0.0, 0.0, 0.12);
  check(gf.valueAtCell(10, 10) == unk::kFree, "clearFootprint 清掉车体所在格");
  check(gf.feasibleAt(0.0, 0.0), "clearFootprint 后车体位置可行");

  // 足迹洞只该抹膨胀带，不该抹 lidar 真打到的墙。否则车贴墙到 footprint 半径以内时
  // 近场被抹平，膨胀带与弦碰撞检测一起失效（= 撞墙前最后一段盲区）。
  unk::GridMap tr = makeGrid(2.0, 0.1, unk::kFree);
  setCell(&tr, 11, 10, unk::kOccupied);  // 真墙格：离车心 0.15m，落在足迹盘(0.12)内
  unk::GridMap t_old = unk::grid::inflate(tr, 0.25);
  unk::grid::clearFootprint(&t_old, 0.0, 0.0, 0.12);
  check(t_old.valueAtCell(11, 10) == unk::kFree,
        "不传 raw：脚下真墙被足迹洞抹平（旧行为，仅供不传 raw 的调用点）");
  unk::GridMap t_new = unk::grid::inflate(tr, 0.25);
  unk::grid::clearFootprint(&t_new, 0.0, 0.0, 0.12, &tr);
  check(t_new.valueAtCell(11, 10) >= unk::kOccupyThreshold,
        "传 raw：足迹盘内的 lidar 真障碍保持 occupied");
  check(t_new.valueAtCell(10, 10) == unk::kFree && t_new.feasibleAt(0.0, 0.0),
        "传 raw：车中心格（raw 为 free、仅被膨胀污染）仍清成 free → 起点不死锁");
  check(t_new.valueAtCell(10, 8) >= unk::kOccupyThreshold, "传 raw：盘外膨胀带不受影响");
  unk::GridMap t_bad = unk::grid::inflate(tr, 0.25);
  const unk::GridMap bad_raw = makeGrid(1.0, 0.1, unk::kFree);  // 规格与 t_bad 不符
  unk::grid::clearFootprint(&t_bad, 0.0, 0.0, 0.12, &bad_raw);
  check(t_bad.valueAtCell(11, 10) == unk::kFree, "raw 规格不符：退回无条件清空");

  // 视线检查
  unk::GridMap w = makeGrid(4.0, 0.05, unk::kFree);
  fillRect(&w, -0.05, -2.0, 0.05, 2.0, unk::kOccupied);  // x=0 处横墙
  check(!unk::grid::segmentFree(w, -1.0, 0.0, 1.0, 0.0), "segmentFree 穿墙=false");
  check(unk::grid::segmentFree(w, -1.0, 0.0, -1.0, 1.5), "segmentFree 平行墙=true");
  check(!unk::grid::segmentFree(w, -1.0, 0.0, 5.0, 0.0), "segmentFree 越出窗口=false");

  unk::GridMap uk = makeGrid(4.0, 0.05, unk::kUnknown);
  check(unk::grid::segmentFree(uk, -1.0, 0.0, 1.0, 0.0), "segmentFree 穿越 unknown=true（乐观）");

  // LOS 拉直：自由栅格里的 L 形折线应被拉成 2 点
  unk::GridMap fr = makeGrid(4.0, 0.05, unk::kFree);
  const std::vector<unk::Point2D> lp{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};
  const auto sc = unk::grid::shortcut(fr, lp);
  check(sc.size() == 2, "shortcut 自由区 L 形拉直为 2 点  (got " +
                            std::to_string(sc.size()) + ")");

  // 螺旋找可行格
  unk::GridMap blk = makeGrid(2.0, 0.1, unk::kFree);
  fillRect(&blk, -0.3, -0.3, 0.3, 0.3, unk::kOccupied);  // 0.6m 见方实心块
  int gx = 0, gy = 0;
  check(blk.worldToGrid(0.0, 0.0, &gx, &gy), "worldToGrid 成功");
  check(!blk.feasibleCell(gx, gy), "块中心不可行");
  check(unk::grid::findFeasibleCellNear(blk, &gx, &gy, 20), "findFeasibleCellNear 找到可行格");
  check(blk.feasibleCell(gx, gy), "吸附后的格确实可行");

  unk::GridMap full = makeGrid(1.0, 0.1, unk::kOccupied);
  int fx = 5, fy = 5;
  check(!unk::grid::findFeasibleCellNear(full, &fx, &fy, 20), "全障碍时找不到可行格");
}

// ── astar ────────────────────────────────────────────────────────

void testAstar() {
  group("astar");
  const unk::Point2D origin{0.0, 0.0};

  // 1) 空栅格直路
  unk::GridMap fr = makeGrid(8.0, 0.05, unk::kFree);
  const unk::Point2D sub{3.0, 0.0};
  const unk::Path p1 = unk::astar::plan(fr, origin, sub, kSpacing);
  check(!p1.empty(), "空栅格：搜到路径");
  if (!p1.empty()) {
    checkNear(p1.front().p.x, 0.0, 1e-9, "空栅格：首点=车位 x");
    checkNear(p1.front().p.y, 0.0, 1e-9, "空栅格：首点=车位 y");
    checkNear(p1.back().p.x, sub.x, 0.06, "空栅格：末点≈子目标 x");
    checkNear(p1.back().p.y, sub.y, 0.06, "空栅格：末点≈子目标 y");
    checkNear(p1.back().s, 3.0, 0.1, "空栅格：路径长≈直线距离");
    check(spacingOk(p1, kSpacing), "空栅格：采样间距合法（<= spacing）");
    check(!pathCollides(fr, p1), "空栅格：路径无碰撞");
  }

  // 2) 横墙留缺口：从缺口穿过
  unk::GridMap gap = makeGrid(8.0, 0.05, unk::kFree);
  fillRect(&gap, -0.05, -4.0, 0.05, -0.6, unk::kOccupied);
  fillRect(&gap, -0.05, 0.6, 0.05, 4.0, unk::kOccupied);
  const unk::Path p2 = unk::astar::plan(gap, origin, {3.0, 0.0}, kSpacing);
  check(!p2.empty(), "带缺口墙：搜到路径");
  if (!p2.empty()) {
    check(!pathCollides(gap, p2), "带缺口墙：路径无碰撞");
    checkNear(p2.back().p.x, 3.0, 0.06, "带缺口墙：到达子目标");
  }

  // 3) 实心横墙贯穿整个窗口：不可达，必须返回空（交给上层脱困）
  unk::GridMap wall = makeGrid(8.0, 0.05, unk::kFree);
  fillRect(&wall, -0.05, -4.0, 0.05, 4.0, unk::kOccupied);
  check(unk::astar::plan(wall, origin, {3.0, 0.0}, kSpacing).empty(),
        "实心墙贯穿窗口：不可达返回空");

  // 4) U 形墙（凹障碍）：主要矛盾场景，必须绕出而不是顶在凹槽里
  unk::GridMap ushape = makeGrid(8.0, 0.05, unk::kFree);
  fillRect(&ushape, 0.95, -1.0, 1.05, 1.0, unk::kOccupied);   // 背墙
  fillRect(&ushape, 1.0, 0.95, 2.0, 1.05, unk::kOccupied);    // 上臂
  fillRect(&ushape, 1.0, -1.05, 2.0, -0.95, unk::kOccupied);  // 下臂（开口朝 +x）
  const unk::Path p4 = unk::astar::plan(ushape, origin, {2.8, 0.0}, kSpacing);
  check(!p4.empty(), "U 形墙：搜到路径");
  if (!p4.empty()) {
    check(!pathCollides(ushape, p4), "U 形墙：路径无碰撞");
    // 臂端最外格中心在 y=1.025，绕过时至少要走到下一格 1.075
    check(pathMaxY(p4) > 1.03, "U 形墙：确实绕过臂端（max|y|=" +
                                   f2s(pathMaxY(p4)) + " > 1.03）");
    checkNear(p4.back().p.x, 2.8, 0.06, "U 形墙：到达子目标");
  }

  // 5) 全 unknown：乐观直穿
  unk::GridMap uk = makeGrid(8.0, 0.05, unk::kUnknown);
  const unk::Path p5 = unk::astar::plan(uk, origin, {3.0, 0.0}, kSpacing);
  check(!p5.empty(), "全 unknown：乐观放行搜到路径");
  if (!p5.empty()) checkNear(p5.back().s, 3.0, 0.1, "全 unknown：路径为直线");

  unk::astar::Options no_unk;
  no_unk.allow_unknown = false;
  check(unk::astar::plan(uk, origin, {3.0, 0.0}, kSpacing, no_unk).empty(),
        "全 unknown + allow_unknown=false：返回空");

  // 6) 子目标越界（真实终点未投影就直接传进来的误用）：必须拒绝
  check(unk::astar::plan(fr, origin, {100.0, 0.0}, kSpacing).empty(),
        "子目标越界：返回空（防御误用真实终点）");
  check(unk::astar::plan(fr, origin, {-50.0, 3.0}, kSpacing).empty(),
        "子目标越界（负向）：返回空");

  // 7) 起点被困：返回空，由上层脱困
  unk::GridMap trapped = makeGrid(8.0, 0.05, unk::kFree);
  fillRect(&trapped, -0.2, -0.2, 0.2, 0.2, unk::kOccupied);
  check(unk::astar::plan(trapped, origin, {3.0, 0.0}, kSpacing).empty(),
        "起点被困：返回空");

  // 8) 迭代护栏：不许死循环
  unk::astar::Options tiny;
  tiny.max_iter = 1;
  check(unk::astar::plan(fr, origin, {3.0, 0.0}, kSpacing, tiny).empty(),
        "max_iter=1：迭代护栏生效返回空");

  // 9) 禁止穿角：对角障碍墙把窗口分成两半，只有穿角才能通过
  unk::GridMap diag = makeGrid(2.0, 0.1, unk::kFree);
  for (int i = 0; i < diag.width; ++i) setCell(&diag, i, i, unk::kOccupied);
  const unk::Point2D a{-0.5, 0.5}, b{0.5, -0.5};
  unk::astar::Options cut;
  cut.forbid_corner_cutting = false;
  check(!unk::astar::plan(diag, a, b, kSpacing, cut).empty(),
        "对角墙 + 允许穿角：能挤过去");
  check(unk::astar::plan(diag, a, b, kSpacing).empty(),
        "对角墙 + 禁止穿角（默认）：不挤墙角，返回空");

  // 10) 膨胀与规划链路：缺口宽窄决定通行性
  unk::GridMap narrow = makeGrid(8.0, 0.05, unk::kFree);
  fillRect(&narrow, -0.05, -4.0, 0.05, -0.35, unk::kOccupied);
  fillRect(&narrow, -0.05, 0.35, 0.05, 4.0, unk::kOccupied);  // 缺口 0.7m
  const unk::GridMap wide_inf = unk::grid::inflate(narrow, 0.16);
  check(!unk::astar::plan(wide_inf, origin, {3.0, 0.0}, kSpacing).empty(),
        "0.7m 缺口 + 0.16m 膨胀：仍可通行");

  unk::GridMap tight = makeGrid(8.0, 0.05, unk::kFree);
  fillRect(&tight, -0.05, -4.0, 0.05, -0.15, unk::kOccupied);
  fillRect(&tight, -0.05, 0.15, 0.05, 4.0, unk::kOccupied);  // 缺口 0.3m
  const unk::GridMap tight_inf = unk::grid::inflate(tight, 0.16);
  check(unk::astar::plan(tight_inf, origin, {3.0, 0.0}, kSpacing).empty(),
        "0.3m 缺口 + 0.16m 膨胀：被膨胀封死，返回空");

  // 11) 膨胀吃掉车体 → 足迹清空后恢复可规划（关键陷阱回归测试）
  //     几何：车正前方 0.5m 处一条短墙，膨胀半径 0.55m 恰好把车埋进障碍层，
  //     但墙顶膨胀到 y=0.075，清空 0.12m 足迹后车可从 y>=0.175 绕过去。
  unk::GridMap selfblock = makeGrid(8.0, 0.05, unk::kFree);
  fillRect(&selfblock, -0.3, -0.55, 0.3, -0.45, unk::kOccupied);
  unk::GridMap sb_inf = unk::grid::inflate(selfblock, 0.55);
  check(!sb_inf.feasibleAt(0.0, 0.0), "膨胀把车埋住：车位不可行");
  check(unk::astar::plan(sb_inf, origin, {3.0, 0.0}, kSpacing).empty(),
        "膨胀把车埋住：A* 起点不可行返回空");
  unk::grid::clearFootprint(&sb_inf, 0.0, 0.0, 0.12);
  check(sb_inf.feasibleAt(0.0, 0.0), "clearFootprint 后车位恢复可行");
  const unk::Path p11 = unk::astar::plan(sb_inf, origin, {3.0, 0.0}, kSpacing);
  check(!p11.empty(), "clearFootprint 后恢复可规划");
  if (!p11.empty()) check(!pathCollides(sb_inf, p11), "clearFootprint 后路径无碰撞");

  // 12) tie-break by h（f 相等时 h 小者优先出队）：最优性不变 + 扩展数受控 + 加权可选
  //     空旷大窗口走对角线：整条对角线 f 恒等于最优值（plateau），最能暴露对称翻转。
  //     默认 w=1.0 靠 h tie-break 沿对角贪心，扩展数远小于全图；路径长仍 = 对角最优。
  {
    unk::GridMap big = makeGrid(10.0, 0.05, unk::kFree);  // 200×200 = 4 万格
    const unk::Point2D s{-4.5, -4.5}, t{4.5, 4.5};        // 一角到对角（纯斜线）
    unk::astar::Workspace ws;
    const unk::Path pd =
        unk::astar::plan(big, s, t, kSpacing, unk::astar::Options(), &ws);
    check(!pd.empty(), "tie-break：对角长路搜到路径");
    if (!pd.empty()) {
      check(!pathCollides(big, pd), "tie-break：路径无碰撞");
      // 最优长 = 对角欧氏距离 hypot(9,9)=12.728（含格心量化/末点吸附小偏差）。
      // tie-break 只改 f 相等节点的出队顺序，不改最优代价 → 长度不变。
      checkNear(pd.back().s, std::hypot(9.0, 9.0), 0.3,
                "tie-break：路径长≈对角最优（最优性保留）");
    }
    // 扩展数受控：全图 4 万格，tie-break 后只扩展对角带附近。上界取 1/4 图作
    // 宽松护栏（防回归：tie-break 若失效会扩展成大菱形，数量级上万）。实际值会
    // 打印在下方消息里，稳定后可把这个上界收得更紧。
    check(ws.last_iter > 0 && ws.last_iter < big.width * big.height / 4,
          "tie-break：扩展数受控（last_iter=" + std::to_string(ws.last_iter) +
              " < 10000，全图 40000）");

    // 加权 w=1.05：仍返回可行（无碰撞）路径，代价次优但更省扩展
    unk::astar::Options wo;
    wo.w = 1.05;
    unk::astar::Workspace ws2;
    const unk::Path pw = unk::astar::plan(big, s, t, kSpacing, wo, &ws2);
    check(!pw.empty(), "加权 w=1.05：搜到路径");
    if (!pw.empty()) check(!pathCollides(big, pw), "加权 w=1.05：路径无碰撞");
  }
}

// ── subgoal ──────────────────────────────────────────────────────

void testSubgoal() {
  group("subgoal");
  unk::NavParams p;  // sensor_range=12 → lookahead=4.2, subgoalMin=1.2

  const unk::GridMap g = makeGrid(20.0, 0.05, unk::kFree);  // ±10m 窗口

  // 远处终点 → 投影到前瞻距离，方向保持
  const auto r1 = unk::subgoal::project(g, {30.0, 40.0}, p);
  check(r1.valid, "远处终点：投影成功");
  checkNear(r1.reach, p.lookahead(), 1e-9, "远处终点：reach = lookahead(4.2)");
  checkNear(std::atan2(r1.point.y, r1.point.x), std::atan2(40.0, 30.0), 1e-9,
            "远处终点：投影方向与终点方向一致");
  check(!r1.clipped_by_window, "远处终点：未被窗口截断");

  // 终点比前瞻近 → reach = 终点距离
  const auto r2 = unk::subgoal::project(g, {2.0, 0.0}, p);
  check(r2.valid, "近处终点：投影成功");
  checkNear(r2.reach, 2.0, 1e-9, "近处终点：reach = goal_dist");
  checkNear(r2.point.x, 2.0, 1e-9, "近处终点：投影点即终点");

  // 死区回归：终点比 subgoalMin*0.5 还近时仍必须给出子目标，
  // 否则 [goal_tolerance, 0.6) 区间既不判到达又不出路径 → 误触发脱困直到 ABORT
  const auto r3 = unk::subgoal::project(g, {0.30, 0.0}, p);
  check(r3.valid, "末段收敛：0.30m < subgoalMin*0.5 仍给出子目标（死区回归）");
  checkNear(r3.reach, 0.30, 1e-9, "末段收敛：reach = 终点距离");

  // 窗口截断：小窗口下 reach 被窗口边界拉近，但方向不变
  const unk::GridMap small = makeGrid(2.0, 0.05, unk::kFree);  // ±1m
  const auto r4 = unk::subgoal::project(small, {10.0, 0.0}, p);
  check(r4.valid, "小窗口：仍给出子目标");
  check(r4.clipped_by_window, "小窗口：标记被窗口截断");
  check(r4.reach < 1.0, "小窗口：reach 被压到窗口内 (got " + f2s(r4.reach) + ")");

  // 窗口过小到无法容纳子目标 → 判无效
  const unk::GridMap tiny = makeGrid(1.0, 0.05, unk::kFree);  // ±0.5m
  check(!unk::subgoal::project(tiny, {10.0, 0.0}, p).valid, "超小窗口：投影失败返回无效");

  // 落点截断：投影落点正好砸在墙上（lookahead=4.2，墙 x=4.0~4.6）
  // → 应沿射线回退到墙前最后一个可行点，而不是交给 A* 螺旋吸附
  unk::GridMap wb = makeGrid(20.0, 0.05, unk::kFree);
  fillRect(&wb, 4.0, -5.0, 4.6, 5.0, unk::kOccupied);
  const auto r5 = unk::subgoal::project(wb, {30.0, 0.0}, p);
  check(r5.valid, "落点砸墙：截断后仍出子目标");
  check(r5.truncated_by_obstacle, "落点砸墙：截断标志置位");
  check(r5.point.x < 4.0, "落点砸墙：回退到墙前 (got " + f2s(r5.point.x) + ")");
  checkNear(r5.point.y, 0.0, 1e-9, "落点砸墙：截断不掰方向");
  check(!unk::subgoal::project(wb, {3.0, 0.0}, p).truncated_by_obstacle,
        "落点本来可行：不触发截断");

  // 整条射线无可行落点（车被占区包住）→ 无效，交给上层脱困
  unk::GridMap box = makeGrid(20.0, 0.05, unk::kFree);
  fillRect(&box, -0.5, -0.5, 5.5, 0.5, unk::kOccupied);  // 盖住整条前瞻射线
  check(!unk::subgoal::project(box, {30.0, 0.0}, p).valid, "射线无可行落点：投影无效");

  // 退化输入
  check(!unk::subgoal::project(g, {0.0, 0.0}, p).valid, "终点与车重合：返回无效");
  check(!unk::subgoal::project(unk::GridMap(), {3.0, 0.0}, p).valid, "空栅格：返回无效");
}

// ── road_follow（沿路前瞻，无定位）──────────────────────

void testRoadFollow() {
  group("road_follow");
  unk::NavParams p;
  p.follow_road = true;
  p.sensor_range = 12.0;  // roadLookahead = 0.35*12 = 4.2
  const double L = p.roadLookahead();
  const double W = 20.0, half = W / 2;

  // 直走廊：两侧 y=±3 是墙，中间 free。正前方自由距离最大 + 对齐最好 → 直行
  unk::GridMap corr = makeGrid(W, 0.05, unk::kFree);
  fillRect(&corr, -half, 3.0, half, half, unk::kOccupied);    // 北墙
  fillRect(&corr, -half, -half, half, -3.0, unk::kOccupied);  // 南墙
  {
    const auto rr = unk::road::lookAhead(corr, p);
    check(rr.valid, "直走廊：找到前向");
    checkNear(rr.bearing, 0.0, 0.06, "直走廊：方向≈正前方");
    checkNear(rr.reach, L, 0.15, "直走廊：reach≈前瞻（走廊够长）");
    check(corr.feasibleAt(rr.point.x, rr.point.y), "直走廊：子目标落点可行");
  }

  // 路中圆柱：正前方被挡，扇形偏向旁边空隙（用户关心的核心场景）
  unk::GridMap cyl = corr;
  fillRect(&cyl, 2.2, -0.35, 2.8, 0.35, unk::kOccupied);  // x≈2.5 处的路中障碍
  {
    const auto rr = unk::road::lookAhead(cyl, p);
    check(rr.valid, "路中障碍：仍找到前向");
    check(std::fabs(rr.bearing) > 0.05,
          "路中障碍：方向偏转绕过（|θ|=" + f2s(std::fabs(rr.bearing)) + "）");
    check(rr.reach > 2.8, "路中障碍：reach 越过障碍（=" + f2s(rr.reach) + "）");
    check(cyl.feasibleAt(rr.point.x, rr.point.y), "路中障碍：子目标落点可行");
  }

  // 弯道：正前方近处被封（东墙 x>2），北侧开阔 → 方向偏向北（左）
  unk::GridMap bend = makeGrid(W, 0.05, unk::kFree);
  fillRect(&bend, -half, -half, half, -3.0, unk::kOccupied);  // 南墙
  fillRect(&bend, -half, 3.0, -1.0, half, unk::kOccupied);    // 北墙仅 x<-1（x>-1 向北开口）
  fillRect(&bend, 2.0, -3.0, half, 3.0, unk::kOccupied);      // 东端封堵（逼迫北转）
  {
    const auto rr = unk::road::lookAhead(bend, p);
    check(rr.valid, "弯道：找到前向");
    check(rr.bearing > 0.5 && rr.bearing < 1.4,
          "弯道：方向偏向北侧开口（θ=" + f2s(rr.bearing) + " rad）");
    check(bend.feasibleAt(rr.point.x, rr.point.y), "弯道：子目标落点可行");
  }

  // 正前方紧贴全封（一步之外即障碍）→ 无可行前向 → 无效
  unk::GridMap blocked = makeGrid(W, 0.05, unk::kFree);
  fillRect(&blocked, 0.0, -half, half, half, unk::kOccupied);  // x>=0 全占据
  check(!unk::road::lookAhead(blocked, p).valid, "正前方全封：前向无效");

  // 空栅格 → 无效
  check(!unk::road::lookAhead(unk::GridMap(), p).valid, "空栅格：返回无效");
}

// ── speed_planner ────────────────────────────────────────────────

void testSpeed() {
  group("speed_planner");
  unk::NavParams p;
  const unk::GridMap g = makeGrid(8.0, 0.05, unk::kFree);

  // 制动包络与反解互为逆运算
  const double v0 = 3.0;
  const double d = unk::speed::brakeDistance(v0, p);
  checkNear(d, v0 * v0 / (2 * p.a_decel_max) + v0 * p.t_reaction + p.safety_margin, 1e-9,
            "brakeDistance 公式");
  checkNear(unk::speed::speedForDistance(d - p.safety_margin, p), v0, 1e-6,
            "speedForDistance 是 brakeDistance 的反函数");

  // 空路径
  const auto e = unk::speed::limit(unk::Path(), g, 0.0, p);
  check(e.emergency_stop, "空路径：急停");
  check(std::string(e.limit_by) == "empty_path", "空路径：limit_by=empty_path");

  // 无障碍直路 → 跑满 v_max
  const unk::Path straight = unk::geom::toPath({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}});
  const auto s1 = unk::speed::limit(straight, g, 0.0, p);
  check(!s1.emergency_stop, "无障碍直路：不急停");
  checkNear(s1.v, p.v_max, 1e-9, "无障碍直路：v = v_max");
  check(std::string(s1.limit_by) == "v_max", "无障碍直路：limit_by=v_max");

  // 前方障碍在制动包络内 → 急停
  unk::GridMap blocked = makeGrid(8.0, 0.05, unk::kFree);
  fillRect(&blocked, 0.4, -1.0, 0.6, 1.0, unk::kOccupied);
  const auto s2 = unk::speed::limit(straight, blocked, 2.0, p);
  check(s2.emergency_stop, "障碍在制动包络内：急停");
  check(std::string(s2.limit_by) == "brake_envelope", "障碍在制动包络内：limit_by=brake_envelope");
  // 墙占据格中心落在 x ∈ [0.425, 0.575]，稠密采样应命中最前缘
  check(s2.obstacle_dist > 0.35 && s2.obstacle_dist < 0.65,
        "障碍距离沿线稠密采样 (got " + f2s(s2.obstacle_dist) + ")");

  // 顶点稀疏的路径也必须查到薄障碍（只查顶点会整段跨过去 → d_obs 误判为 inf）
  check(!std::isinf(s2.obstacle_dist), "薄障碍未被稀疏顶点漏掉");

  // 曲率超过 kappa_max → 几何不可行，急停而非降速
  unk::NavParams pk = p;
  pk.kappa_max = 1.0;
  const unk::Path curvy = unk::geom::toPath({{0, 0}, {0.5, 0}, {1.0, 0}, {1.0, 0.5}, {1.0, 1.0}});
  check(curvy[2].k > 1.0, "构造的路径曲率确实超限 (k=" + f2s(curvy[2].k) + ")");
  const auto s3 = unk::speed::limit(curvy, g, 0.0, pk);
  check(s3.emergency_stop, "曲率超 kappa_max：急停");
  check(std::string(s3.limit_by) == "kappa_exceeded", "曲率超限：limit_by=kappa_exceeded");

  // 默认 kappa_max=0（差速底盘）：同一条路径不得被硬判死，只能降速通过。
  // 栅格 A* 的直角拐点曲率约 sqrt(2)/resolution，硬门限会把正常拐弯全部误杀。
  const auto s3b = unk::speed::limit(curvy, g, 0.0, p);
  check(!s3b.emergency_stop, "默认关闭曲率硬门限：急转弯降速通过而非急停");
  check(s3b.v > 0.0, "默认关闭曲率硬门限：仍有非零推荐速度 (v=" + f2s(s3b.v) + ")");

  // 三条运动约束分别生效（放大 v_max 才能让分项成为瓶颈）
  unk::NavParams pl = p;
  pl.v_max = 5.0;
  pl.a_lat_max = 1.0;
  pl.w_max = 100.0;
  pl.dk_max = 1e9;
  const auto s4 = unk::speed::limit(curvy, g, 0.0, pl);
  check(std::string(s4.limit_by) == "lateral_accel", "横向加速度成为瓶颈");
  checkNear(s4.v, std::sqrt(pl.a_lat_max / s4.kappa_max), 1e-9, "v = sqrt(a_lat/kappa)");

  unk::NavParams pw = pl;
  pw.a_lat_max = 100.0;
  pw.w_max = 1.0;
  const auto s5 = unk::speed::limit(curvy, g, 0.0, pw);
  check(std::string(s5.limit_by) == "yaw_rate", "角速度上限成为瓶颈");
  checkNear(s5.v, pw.w_max / s5.kappa_max, 1e-9, "v = w_max/kappa");

  unk::NavParams pd = pl;
  pd.a_lat_max = 100.0;
  pd.dk_max = 10.0;
  const auto s6 = unk::speed::limit(curvy, g, 0.0, pd);
  check(std::string(s6.limit_by) == "curvature_rate", "曲率变化率成为瓶颈");
  checkNear(s6.v, pd.dk_max / s6.dk_ds_max, 1e-9, "v = dk_max/(dkappa/ds)");
}

// ── behavior_fsm ─────────────────────────────────────────────────

void testFsm() {
  group("behavior_fsm");
  unk::NavParams p;
  p.stuck_time = 3.0;
  unk::fsm::BehaviorFsm f(p);

  auto ctx = [](double now, bool plan_ok, double goal_dist) {
    unk::fsm::Context c;
    c.now = now;
    c.goal_valid = true;
    c.goal_dist = goal_dist;
    c.plan_ok = plan_ok;
    c.emergency_stop = !plan_ok;
    return c;
  };

  unk::fsm::Context nog;
  nog.goal_valid = false;
  check(f.update(nog) == unk::NavState::IDLE, "无终点 → IDLE");

  f.reset();
  check(f.update(ctx(0.0, true, 10.0)) == unk::NavState::GO, "正常推进 → GO");

  // 到达并锁存
  check(f.update(ctx(1.0, true, 0.10)) == unk::NavState::ARRIVED, "距终点 < 容差 → ARRIVED");
  check(f.update(ctx(2.0, true, 10.0)) == unk::NavState::ARRIVED, "ARRIVED 锁存（终点又变远也不退出）");
  f.newGoal();
  check(f.update(ctx(3.0, true, 10.0)) == unk::NavState::GO, "newGoal 后退出 ARRIVED 锁存");

  // 规划成功但没能朝终点推进 → RECOVERY（stuck_dist 默认 0.05）
  f.reset();
  check(f.update(ctx(0.0, true, 10.0)) == unk::NavState::GO, "无进展：首周期只锚定不判");
  check(f.update(ctx(3.1, true, 9.90)) == unk::NavState::GO, "实质推进 → 棘轮刷新计时");
  check(f.update(ctx(6.0, true, 9.88)) == unk::NavState::GO, "推进不足 stuck_dist 不算进展，但未满窗口");
  check(f.update(ctx(6.2, true, 9.87)) == unk::NavState::RECOVERY,
        "距上次实质推进 3.1s → RECOVERY");

  // 振荡：规划一直成功、车一直在动、距终点在原地波动 —— 位移量判定的盲区。
  // 旧实现在这个场景下永远沉默并持续上报 GO 满速（实测深凹槽振荡 280s 不报）。
  f.reset();
  check(f.update(ctx(0.0, true, 14.00)) == unk::NavState::GO, "振荡：锚定");
  check(f.update(ctx(1.0, true, 14.08)) == unk::NavState::GO, "振荡：距终点变大");
  check(f.update(ctx(2.0, true, 13.98)) == unk::NavState::GO, "振荡：微小改善不算进展，未满窗口");
  check(f.update(ctx(3.0, true, 14.05)) == unk::NavState::RECOVERY,
        "振荡 3s 零推进 → RECOVERY（位移量判定会漏掉这个）");

  // 连续规划失败 → RECOVERY → 重试耗尽 → ABORT
  f.reset();
  check(f.update(ctx(0.0, false, 10.0)) == unk::NavState::GO, "刚被挡：未满 stuck_time 仍报 GO");
  check(f.update(ctx(2.9, false, 10.0)) == unk::NavState::GO, "2.9s 仍未满判定时长");
  check(f.update(ctx(3.0, false, 10.0)) == unk::NavState::RECOVERY, "满 3s → RECOVERY");
  check(f.retryCount() == 1, "重试计数 +1");
  for (int i = 0; i < p.recovery_max_retry + 2; ++i) {
    f.update(ctx(10.0 + i * 10.0, false, 10.0));
    f.update(ctx(13.0 + i * 10.0, false, 10.0));
  }
  check(f.state() == unk::NavState::ABORT, "重试耗尽 → ABORT");
  check(f.update(ctx(999.0, true, 10.0)) == unk::NavState::ABORT, "ABORT 锁存");
  f.reset();
  check(f.state() == unk::NavState::IDLE && f.retryCount() == 0, "reset 清空状态与计数");
}

// ── nav_core（端到端）────────────────────────────────────────────

void testNavCore() {
  group("nav_core 端到端");
  unk::NavParams p;
  const double kWindow = 1.7 * p.sensor_range;

  auto makeInput = [&](const unk::GridMap& grid, const unk::Pose2D& veh,
                       const unk::Point2D& goal, double t, double spd) {
    unk::NavInput in;
    in.now = t;
    in.local_grid = grid;
    in.vehicle_pose = veh;
    in.goal = goal;
    in.goal_valid = true;
    in.current_speed = spd;
    return in;
  };

  unk::NavCore nav(p);

  // 无终点 → IDLE + 停车
  unk::NavInput nog;
  nog.local_grid = makeGrid(kWindow, 0.05, unk::kFree);
  nog.goal_valid = false;
  const auto r0 = nav.plan(nog);
  check(r0.state == unk::NavState::IDLE, "无终点 → IDLE");
  check(r0.emergency_stop && r0.recommended_speed == 0.0, "无终点 → 停车");
  check(r0.path.empty(), "无终点 → 不输出路径");

  // 空世界 + 远处终点 → 直线路径（demo 的核心场景）
  nav.reset();
  const unk::GridMap fr = makeGrid(kWindow, 0.05, unk::kFree);
  const unk::Pose2D veh{0.0, 0.0, 0.0};
  const auto r1 = nav.plan(makeInput(fr, veh, {30.0, 0.0}, 0.0, 0.0));
  check(r1.state == unk::NavState::GO, "空世界远处终点 → GO");
  check(!r1.path.empty(), "空世界远处终点 → 出路径");
  check(r1.recommended_speed > 0.0, "空世界远处终点 → 非零推荐速度");
  check(r1.subgoal_reachable, "空世界远处终点 → 子目标可达");
  checkNear(r1.subgoal.x, p.lookahead(), 0.1, "子目标落在 lookahead 处");
  if (!r1.path.empty()) {
    check(!pathCollides(nav.workGrid(), r1.path), "空世界：路径在工作栅格上无碰撞");
    checkNear(r1.path.back().p.y, 0.0, 0.1, "空世界：路径为直线（y 不漂）");
    checkNear(r1.path.back().s, p.lookahead(), 0.1, "空世界：路径长≈lookahead");
  }

  // 全 unknown（完全未知环境）→ 乐观放行仍能出路径
  nav.reset();
  const unk::GridMap uk = makeGrid(kWindow, 0.05, unk::kUnknown);
  const auto r2 = nav.plan(makeInput(uk, veh, {30.0, 0.0}, 0.0, 0.0));
  check(!r2.path.empty(), "全 unknown：乐观放行仍出路径");
  check(r2.recommended_speed > 0.0, "全 unknown：非零推荐速度");

  // 终点已在容差内 → ARRIVED + 停车 + 不输出路径
  nav.reset();
  const auto r3 = nav.plan(makeInput(fr, veh, {0.10, 0.0}, 0.0, 0.0));
  check(r3.state == unk::NavState::ARRIVED, "终点在容差内 → ARRIVED");
  check(r3.recommended_speed == 0.0, "ARRIVED → 零速");
  check(r3.path.empty(), "ARRIVED → 不输出路径");

  // 实心墙挡在正前方：v0 无绕行能力，必须停车而不是硬冲
  nav.reset();
  unk::GridMap wall = makeGrid(kWindow, 0.05, unk::kFree);
  fillRect(&wall, -0.05, -kWindow, 0.05, kWindow, unk::kOccupied);
  const auto r4 = nav.plan(makeInput(wall, veh, {30.0, 0.0}, 0.0, 0.0));
  check(r4.path.empty(), "实心墙：不出路径");
  check(r4.emergency_stop && r4.recommended_speed == 0.0, "实心墙：急停零速");
  check(!r4.subgoal_reachable, "实心墙：子目标不可达");

  // 连续被挡 → 触发脱困计数
  for (double t = 0.1; t <= p.stuck_time + 0.2; t += 0.1) {
    nav.plan(makeInput(wall, veh, {30.0, 0.0}, t, 0.0));
  }
  check(nav.fsm().retryCount() >= 1, "连续被挡超过 stuck_time：脱困计数已累加");

  // 换终点应清掉脱困计数
  nav.plan(makeInput(fr, veh, {-20.0, 8.0}, 100.0, 0.0));
  check(nav.fsm().retryCount() == 0, "换终点后脱困计数清零");
}

// ── nav_core 沿路模式（无定位端到端）────────────────────

void testNavCoreRoad() {
  group("nav_core 沿路模式");
  unk::NavParams p;
  p.follow_road = true;
  p.sensor_range = 12.0;
  p.stuck_time = 3.0;
  const double W = 1.7 * p.sensor_range;  // 20.4
  const double half = W / 2;

  auto makeRoadInput = [&](const unk::GridMap& grid, double t, double spd) {
    unk::NavInput in;
    in.now = t;
    in.local_grid = grid;
    in.goal_valid = false;  // 沿路模式无全局终点、无 pose
    in.current_speed = spd;
    in.speed_valid = true;
    return in;
  };

  // 直走廊：无终点、无 pose → 仍 GO + 出路径 + 非零速度（无定位沿路的核心断言）
  unk::NavCore nav(p);
  unk::GridMap corr = makeGrid(W, 0.05, unk::kFree);
  fillRect(&corr, -half, 3.0, half, half, unk::kOccupied);
  fillRect(&corr, -half, -half, half, -3.0, unk::kOccupied);
  const auto r1 = nav.plan(makeRoadInput(corr, 0.0, 0.2));
  check(r1.state == unk::NavState::GO, "沿路直走廊：GO");
  check(!r1.path.empty(), "沿路直走廊：出路径");
  check(r1.recommended_speed > 0.0, "沿路直走廊：非零推荐速度");
  if (!r1.path.empty()) {
    check(!pathCollides(nav.workGrid(), r1.path), "沿路直走廊：路径无碰撞");
    check(r1.path.back().p.x > 1.0, "沿路直走廊：路径朝前推进");
    check(std::fabs(r1.path.back().p.y) < 3.0, "沿路直走廊：路径留在走廊内");
  }

  // 路中圆柱：绕行且无碰撞
  nav.reset();
  unk::GridMap cyl = corr;
  fillRect(&cyl, 2.2, -0.35, 2.8, 0.35, unk::kOccupied);
  const auto r2 = nav.plan(makeRoadInput(cyl, 0.0, 0.2));
  check(!r2.path.empty(), "沿路遇路中障碍：出路径");
  if (!r2.path.empty())
    check(!pathCollides(nav.workGrid(), r2.path), "沿路遇路中障碍：路径无碰撞（绕行）");

  // 走廊被横墙封死：无法前进（speed=0）→ 前进位移棘轮 → RECOVERY → ABORT（不谎报 GO）
  nav.reset();
  unk::GridMap dead = makeGrid(W, 0.05, unk::kFree);
  fillRect(&dead, -half, -half, half, -3.0, unk::kOccupied);
  fillRect(&dead, -half, 3.0, half, half, unk::kOccupied);
  fillRect(&dead, 1.5, -3.0, 1.7, 3.0, unk::kOccupied);  // 横贯走廊的墙
  unk::NavState last = unk::NavState::IDLE;
  for (double t = 0.0; t <= 30.0; t += 0.1) last = nav.plan(makeRoadInput(dead, t, 0.0)).state;
  check(last == unk::NavState::ABORT || last == unk::NavState::RECOVERY,
        "沿路走廊封死：最终 RECOVERY/ABORT（不谎报 GO）");
}

// ── 曲率基线 + path_smooth ───────────────────────────────────────

double pathMaxK(const unk::Path& p) {
  double m = 0.0;
  for (const auto& pp : p) m = std::max(m, std::fabs(pp.k));
  return m;
}

void testCurvatureBaseline() {
  group("曲率测量基线");

  // 带 ±0.01m 抖动的直线：按相邻点估会把抖动放大成假曲率，按基线估则接近 0。
  // 这是「曲率必须与采样密度解耦」的核心性质。
  std::vector<unk::Point2D> zig;
  for (int i = 0; i <= 40; ++i) {
    zig.push_back({i * 0.05, (i % 2 == 0) ? 0.01 : -0.01});
  }
  const double k_adj = pathMaxK(unk::geom::toPath(zig, 0.0));
  const double k_base = pathMaxK(unk::geom::toPath(zig, 0.30));
  check(k_adj > 5.0, "相邻点估：±0.01m 抖动被放大成假曲率 (k=" + f2s(k_adj) + ")");
  check(k_base < k_adj * 0.2, "基线估：同一抖动几乎不产生曲率 (k=" + f2s(k_base) + ")");

  // 真圆弧 R=2.0：两种采样密度下基线估的曲率都应 ≈ 1/R，且彼此一致
  auto makeArc = [](double R, double spacing) {
    std::vector<unk::Point2D> pts;
    const double angle = 1.0;  // 1 弧度
    const int n = std::max(2, static_cast<int>(std::ceil(R * angle / spacing)));
    for (int i = 0; i <= n; ++i) {
      const double a = angle * static_cast<double>(i) / static_cast<double>(n);
      pts.push_back({R * std::cos(a), R * std::sin(a)});
    }
    return pts;
  };
  const double k_coarse = pathMaxK(unk::geom::toPath(makeArc(2.0, 0.05), 0.30));
  const double k_fine = pathMaxK(unk::geom::toPath(makeArc(2.0, 0.02), 0.30));
  checkNear(k_coarse, 0.5, 0.02, "R=2 圆弧、spacing=0.05：kappa≈1/R");
  checkNear(k_fine, 0.5, 0.02, "R=2 圆弧、spacing=0.02：kappa≈1/R");
  check(std::fabs(k_coarse - k_fine) < 0.02,
        "曲率与采样密度解耦（相差 " + f2s(std::fabs(k_coarse - k_fine)) + "）");
}

void testSmooth() {
  group("path_smooth");

  // 倒角半径反解：同时满足 w = v/r <= w_max 与 v^2/r <= a_lat_max
  checkNear(unk::smooth::filletRadiusForCornerSpeed(0.22, 2.84, 1.0), 0.22 / 2.84, 1e-9,
            "低速差速车：角速度约束起决定作用");
  checkNear(unk::smooth::filletRadiusForCornerSpeed(8.3, 0.8, 2.5), 8.3 * 8.3 / 2.5, 1e-9,
            "30km/h 实车：横向加速度约束起决定作用");
  checkNear(unk::smooth::filletRadiusForCornerSpeed(0.0, 2.84, 1.0), 0.0, 1e-9,
            "过弯速度 0 → 半径 0（关闭倒角）");

  // 直角折线倒角：曲率下降、端点不动、无碰撞
  const unk::GridMap g = makeGrid(6.0, 0.05, unk::kFree);
  const std::vector<unk::Point2D> corner{{0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}};

  unk::smooth::Options so;
  so.fillet_radius = 0.3;
  so.spacing = 0.05;
  const auto fil = unk::smooth::filletCorners(g, corner, so);
  check(fil.size() > corner.size(), "倒角插入了圆弧采样点");
  checkNear(fil.front().x, 0.0, 1e-9, "倒角不挪起点 x（起点=车位）");
  checkNear(fil.back().y, 1.0, 1e-9, "倒角不挪终点 y（终点=子目标）");
  check(unk::grid::polylineFree(g, fil), "倒角结果无碰撞");

  // 加密采样后比较曲率：倒角必须显著降低最大曲率（这才是提速的原因）
  const double k_raw = pathMaxK(unk::geom::toPath(
      unk::geom::resampleKeepCorners(corner, 0.05), 0.30));
  const double k_fil = pathMaxK(
      unk::geom::toPath(unk::geom::resampleKeepCorners(fil, 0.05), 0.30));
  check(k_raw > 5.0, "未倒角的直角拐点曲率很大 (k=" + f2s(k_raw) + ")");
  check(k_fil < k_raw * 0.5, "倒角后曲率显著下降 (" + f2s(k_raw) + " → " + f2s(k_fil) + ")");
  checkNear(k_fil, 1.0 / 0.3, 0.15, "倒角后曲率≈1/半径");

  // 半径越大曲率越小（单调性）
  unk::smooth::Options so2 = so;
  so2.fillet_radius = 0.6;
  const auto fil2 = unk::smooth::filletCorners(g, corner, so2);
  const double k_fil2 = pathMaxK(
      unk::geom::toPath(unk::geom::resampleKeepCorners(fil2, 0.05), 0.30));
  check(k_fil2 < k_fil, "半径加大 → 曲率进一步下降");

  // 拐角内侧贴障碍：倒角必须收缩或放弃，但绝不许撞。
  // 障碍块要严格落在拐角内侧 —— 不能碰到 y=0（格中心 0.025）与 x=1（格中心 1.025）
  // 两条原线段所在的格子，否则输入折线本身就是撞的，测不出倒角的回退行为。
  unk::GridMap tight = makeGrid(6.0, 0.05, unk::kFree);
  fillRect(&tight, 0.85, 0.06, 0.95, 0.14, unk::kOccupied);
  check(unk::grid::polylineFree(tight, corner), "前提：原折线在 tight 栅格上无碰撞");
  const auto fil_t = unk::smooth::filletCorners(tight, corner, so);
  check(unk::grid::polylineFree(tight, fil_t), "贴障拐角：倒角收缩/放弃后仍无碰撞");

  // 拉普拉斯松弛：端点固定
  unk::smooth::Options lo;
  lo.laplacian_iters = 5;
  lo.laplacian_lambda = 0.3;
  const auto lap = unk::smooth::laplacian(g, fil, lo);
  check(lap.size() == fil.size(), "拉普拉斯不改变点数");
  checkNear(lap.front().x, fil.front().x, 1e-9, "拉普拉斯固定起点");
  checkNear(lap.back().y, fil.back().y, 1e-9, "拉普拉斯固定终点");
  check(unk::grid::polylineFree(g, lap), "拉普拉斯结果无碰撞");

  // 松弛撞障时必须回退：把折线整体贴到墙边，松弛会把它推进墙里
  unk::GridMap wallg = makeGrid(6.0, 0.05, unk::kFree);
  fillRect(&wallg, -3.0, 0.3, 3.0, 3.0, unk::kOccupied);  // y>=0.3 全是墙
  const std::vector<unk::Point2D> hug{{-1.0, 0.0}, {0.0, 0.2}, {1.0, 0.0}};
  const auto lap2 = unk::smooth::laplacian(wallg, hug, lo);
  check(unk::grid::polylineFree(wallg, lap2), "松弛撞障时回退，输出仍无碰撞");

  // run() 一站式：输出保证无碰撞
  const auto done = unk::smooth::run(tight, corner, so);
  check(unk::grid::polylineFree(tight, done), "run() 输出保证无碰撞");

  // 关闭时原样返回
  unk::smooth::Options off;
  const auto same = unk::smooth::run(g, corner, off);
  check(same.size() == corner.size(), "平滑全关时原样返回");
}

}  // namespace

// ── params_io ──────────────────────────────────────────────────────────────────

void testParamsIo() {
  group("params_io");
  const char* path = "/tmp/unk_nav_params_test.yaml";
  std::string err;

  // 正常加载：写出的字段覆盖，未写的保持默认
  {
    std::ofstream f(path);
    f << "# 注释行\n\n"
      << "v_max: 1.5\nw_max: 2.5\nsmooth_laplacian_iters: 5\n"
      << "inflate_unknown: true\npursuit_lookahead: 0.9\n";
  }
  unk::NavParams p;
  check(unk::loadNavParams(path, &p, &err), "正常加载");
  checkNear(p.v_max, 1.5, 1e-12, "double 读入");
  check(p.smooth_laplacian_iters == 5, "int 读入");
  check(p.inflate_unknown == true, "bool 读入");
  checkNear(p.pursuit_lookahead, 0.9, 1e-12, "控制器参数同表读入");
  checkNear(p.sensor_range, unk::NavParams().sensor_range, 1e-12, "未写字段保持默认");

  // 未知 key → 拒绝（拼错的参数名绝不能静默失效）
  {
    std::ofstream f(path);
    f << "v_maxx: 1.0\n";
  }
  unk::NavParams q;
  check(!unk::loadNavParams(path, &q, &err), "未知 key：拒绝加载");
  check(err.find("unknown") != std::string::npos, "未知 key：报错含原因");

  // 类型不可转换 → 拒绝
  {
    std::ofstream f(path);
    f << "astar_max_iter: abc\n";
  }
  check(!unk::loadNavParams(path, &q, &err), "类型错误：拒绝加载");

  // 文件不存在 → 拒绝
  check(!unk::loadNavParams("/tmp/definitely_missing_unk_nav_cfg.yaml", &q, &err),
        "文件不存在：拒绝加载");
  std::remove(path);
}

// ── controller ──────────────────────────────────────────

void testController() {
  group("controller");

  // L 形路径：先沿 +x 走 2m 再折向 +y 走 2m（弧长 s = 0 / 2 / 4）
  const std::vector<unk::Point2D> poly = {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}};
  const unk::Path path = unk::geom::toPath(poly);
  const unk::GridMap open = makeGrid(10.0, 0.1, unk::kFree);
  unk::PurePursuitController ctl(0.8, 1.5);

  // 1) 车在规划原点：前视点落在直段上 → 不该有角速度
  const auto c0 = ctl.compute(path, 1.0, open);
  checkNear(c0.v, 1.0, 1e-12, "线速度 = 推荐速度");
  checkNear(c0.w, 0.0, 1e-9, "车在原点：直段前视点 → w=0");
  // 2) 显式传零位姿必须与省略该实参完全一致（向后兼容硬要求）
  const auto c0b = ctl.compute(path, 1.0, open, unk::Pose2D());
  checkNear(c0b.w, c0.w, 1e-15, "car 默认值与显式零位姿等价");

  // 3) 车已走到拐角前 (1.9,0)：前视点必须跨过拐角落到 +y 段 → 提前起转
  const auto c1 = ctl.compute(path, 1.0, open, unk::Pose2D{1.9, 0.0, 0.0});
  check(c1.w > 0.5, "车前移后前视点跨过拐角 → 提前起转 (got " + f2s(c1.w) + ")");
  check(std::fabs(c0.w) < 1e-9 && c1.w > 0.5,
        "同一缓存路径：不修正车位置时每 tick 都是同一条指令");

  // 4) 车头偏左且位置偏左 → 必须往右拉
  const auto c2 = ctl.compute(path, 1.0, open, unk::Pose2D{1.0, 0.3, 0.35});
  check(c2.w < 0.0, "车偏左+航向偏左 → 右拉 (got " + f2s(c2.w) + ")");

  // 5) 角速度限幅取构造入参 w_max
  const auto c3 = ctl.compute(path, 3.0, open, unk::Pose2D{1.99, 0.0, 0.0});
  checkNear(c3.w, 1.5, 1e-12, "角速度被 w_max 限幅");

  // 6) 弦撞墙 → 前视点从远处收缩到拐角前，角速度随之变小
  unk::GridMap wall = makeGrid(10.0, 0.1, unk::kFree);
  fillRect(&wall, 1.9, 1.0, 2.1, 1.1, unk::kOccupied);  // 竖着挡在拐角内侧
  const auto c4 = ctl.compute(path, 1.0, wall, unk::Pose2D{1.9, 0.0, 0.0});
  check(c4.w < 0.1 && c4.w < c1.w, "弦撞墙 → 前视点收缩 (got " + f2s(c4.w) + ")");

  // 7) 退化输入
  check(ctl.compute(unk::Path(), 1.0, open).w == 0.0, "空路径 → 零指令");
  check(ctl.compute(path, 0.005, open).v == 0.0, "速度<=0.01 → 零指令");

  // 8) 指令斜率限幅
  unk::TwistSlewLimiter slew(3.0, 6.0);
  const unk::TwistCmd want{1.5, 1.5};
  const auto s1 = slew.limit(want, 0.02);
  checkNear(s1.v, 0.06, 1e-12, "首 tick 线速度受 cmd_a_max 限制 (3.0×0.02)");
  checkNear(s1.w, 0.12, 1e-12, "首 tick 角速度受 cmd_w_dot_max 限制 (6.0×0.02)");
  auto sN = s1;
  for (int i = 0; i < 200; ++i) sN = slew.limit(want, 0.02);
  checkNear(sN.v, 1.5, 1e-12, "持续斜坡后到达目标线速度");
  checkNear(sN.w, 1.5, 1e-12, "持续斜坡后到达目标角速度");
  slew.reset();
  checkNear(slew.limit(want, 0.02).v, 0.06, 1e-12, "reset 后重新从 0 斜坡（急停再起步）");
  checkNear(slew.limit(want, 0.0).v, 1.5, 1e-12, "dt<=0 原样透传（未启用）");
}

int main() {
  std::printf("unk_nav core_test —— 离线单测（无 ROS）\n");
  testGeom();
  testGrid();
  testAstar();
  testCurvatureBaseline();
  testSmooth();
  testSubgoal();
  testRoadFollow();
  testSpeed();
  testController();
  testFsm();
  testNavCore();
  testNavCoreRoad();
  testParamsIo();
  std::printf("\n----------------------------------------\n");
  std::printf("通过 %d 项，失败 %d 项\n", g_pass, g_fail);
  if (g_fail > 0) {
    std::printf("结果：FAIL\n");
    return 1;
  }
  std::printf("结果：ALL PASS\n");
  return 0;
}
