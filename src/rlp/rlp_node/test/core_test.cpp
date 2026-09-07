// 核心层离线单测（不依赖 ROS）：验证四种边界可用性状态、方法路由、
// 模式滞回、障碍急停。直接驱动 planner::PlannerCore。
// 编译：catkin_make 后运行 build/rlp_node/core_test（或 ctest）
#include <iostream>
#include <string>

#include "rlp_planner/planner_core.h"

using rlp::PlanMode;
using rlp::PlannerParams;
using rlp::Point2D;
using rlp::planner::PlannerCore;
using rlp::planner::PlannerInput;
using rlp::road::BoundarySet;
using rlp::road::BoundaryState;

// ── 测试辅助（仅本文件使用）──────────────────────────────────────
namespace {

BoundarySet straightRoad(bool left, bool right) {
  BoundarySet b;
  if (left) for (int i = 0; i < 20; ++i) b.left.push_back({1.0 + 2.0 * i, 2.5});
  if (right) for (int i = 0; i < 20; ++i) b.right.push_back({1.0 + 2.0 * i, -2.5});
  return b;
}

PlannerInput makeInput(const BoundarySet& b, double now, double quality,
                       bool goal_valid = false) {
  PlannerInput in;
  in.now = now;
  in.boundaries = b;
  in.localization_quality = quality;
  in.current_speed = 5.0;
  in.goal = {80.0, 0.0};
  in.goal_valid = goal_valid;
  return in;
}

int failures = 0;
void check(bool cond, const std::string& name) {
  std::cout << (cond ? "[ OK ] " : "[FAIL] ") << name << "\n";
  if (!cond) ++failures;
}

}  // namespace

// ── 测试入口 ─────────────────────────────────────────────────────

int main() {
  PlannerParams p;

  // 1) 双侧有效 + 定位差 → follow 方法，输出可行路径
  {
    PlannerCore c(p);
    auto r = c.plan(makeInput(straightRoad(true, true), 1.0, 0.2));
    check(r.mode == PlanMode::FOLLOW && r.method == "follow" &&
              std::string(r.algorithm) == "offset" && !r.emergency_stop &&
              r.path.size() > 2,
          "both sides, poor loc -> follow planner (offset alg) feasible");
  }

  // 2) 单侧缺失：先用双侧建立路宽记忆，再只给左边界 → LEFT_ONLY 走廊仍有效
  {
    PlannerCore c(p);
    c.plan(makeInput(straightRoad(true, true), 1.0, 0.2));
    // 右边界超时(>boundary_timeout)后才判 LEFT_ONLY
    auto r = c.plan(makeInput(straightRoad(true, false), 1.6, 0.2));
    check(r.boundary_state == BoundaryState::LEFT_ONLY && !r.emergency_stop,
          "left only -> corridor from width memory");
  }

  // 3) 双侧短时缺失 → 沿用上帧走廊(MISSING_SHORT)，置信度下降但不停车
  {
    PlannerCore c(p);
    c.plan(makeInput(straightRoad(true, true), 1.0, 0.2));
    // 双侧均超时缺失(>0.5s)但在记忆期(<3s)内 -> MISSING_SHORT
    auto r = c.plan(makeInput(straightRoad(false, false), 1.6, 0.2));
    check(r.boundary_state == BoundaryState::MISSING_SHORT && !r.emergency_stop &&
              r.corridor_confidence < 1.0,
          "both missing short -> memory corridor, reduced confidence");
  }

  // 4) 双侧超时缺失 + 定位差 → 无可用方法，急停保护
  {
    PlannerCore c(p);
    c.plan(makeInput(straightRoad(true, true), 1.0, 0.2));
    auto r = c.plan(makeInput(straightRoad(false, false), 10.0, 0.2));
    check(r.boundary_state == BoundaryState::MISSING_TIMEOUT && r.emergency_stop &&
              r.method == "none",
          "timeout + poor loc -> emergency stop (no method)");
  }

  // 5) 双侧超时缺失 + 定位好 + 终点可用 → free 方法（纯终点方向推进）
  {
    PlannerCore c(p);
    c.plan(makeInput(straightRoad(true, true), 1.0, 0.9, true));
    auto r = c.plan(makeInput(straightRoad(false, false), 10.0, 0.9, true));
    check(r.method == "free" && !r.emergency_stop,
          "timeout + good loc + goal -> free planner");
  }

  // 6) 定位滞回：quality 高且终点有效才切 SEARCH；quality 掉回 FOLLOW
  {
    PlannerCore c(p);
    c.plan(makeInput(straightRoad(true, true), 1.0, 0.9, true));  // 首帧进入 SEARCH
    auto r = c.plan(makeInput(straightRoad(true, true), 5.0, 0.9, true));
    check(r.mode == PlanMode::SEARCH && r.method == "search",
          "high quality + valid goal -> search planner");
    auto r2 = c.plan(makeInput(straightRoad(true, true), 20.0, 0.1, true));
    check(r2.mode == PlanMode::FOLLOW && r2.method == "follow",
          "quality drop -> back to follow planner");
  }

  // 7) 障碍挡住整条走廊 → 无可行候选急停
  {
    PlannerCore c(p);
    PlannerInput in = makeInput(straightRoad(true, true), 1.0, 0.2);
    in.map.resolution = 0.5;
    in.map.width = 100;
    in.map.height = 100;
    in.map.origin_x = 0.0;
    in.map.origin_y = -25.0;
    in.map.data.assign(100 * 100, 0);
    // 在前方 x≈5m 处画一堵横贯走廊的占据墙
    const int wx = static_cast<int>((5.0 - 0.0) / 0.5);
    for (int y = 40; y <= 60; ++y) in.map.data[y * 100 + wx] = 100;
    auto r = c.plan(in);
    check(r.emergency_stop, "full-width obstacle wall -> emergency stop");
  }

  // 8) A* 算法：定位好 + 终点可用，search_alg=astar → 直接搜索出路径（跳过候选评价）
  {
    PlannerParams pa = p;
    pa.search_alg = "astar";
    PlannerCore c(pa);
    PlannerInput in = makeInput(straightRoad(true, true), 1.0, 0.9, true);
    in.map.resolution = 0.5;
    in.map.width = 120;
    in.map.height = 100;
    in.map.origin_x = 0.0;
    in.map.origin_y = -25.0;
    in.map.data.assign(120 * 100, 0);  // 全自由栅格（unknown/空均可通行）
    auto r = c.plan(in);
    check(r.mode == PlanMode::SEARCH && r.method == "search" &&
              std::string(r.algorithm) == "astar" && !r.emergency_stop &&
              r.path.size() > 2,
          "astar: good loc + goal -> direct A* path");
  }

  // 9) A* 绕障：走廊中段被占据块挡死中线，A* 应搜出绕行路径（采样族会被碰撞淘汰）
  {
    PlannerParams pa = p;
    pa.search_alg = "astar";
    PlannerCore c(pa);
    PlannerInput in = makeInput(straightRoad(true, true), 1.0, 0.9, true);
    in.map.resolution = 0.5;
    in.map.width = 120;
    in.map.height = 100;
    in.map.origin_x = 0.0;
    in.map.origin_y = -25.0;
    in.map.data.assign(120 * 100, 0);
    // x∈[5,7], y∈[-2.5,2.5] 占据块：挡死走廊中线，但留上下绕行空间（半宽 2.5）
    for (int gx = 10; gx <= 14; ++gx)
      for (int gy = 45; gy <= 55; ++gy) in.map.data[gy * 120 + gx] = 100;
    auto r = c.plan(in);
    bool detours = false;
    for (const auto& pp : r.path) {
      if (std::abs(pp.p.y) > 2.6) { detours = true; break; }
    }
    check(!r.emergency_stop && r.path.size() > 2 && detours,
          "astar: path detours around mid-corridor obstacle");
  }

  // 10) RRT 算法：定位好 + 终点可用，search_alg=rrt → RRT 搜索出路径（跳过候选评价）
  {
    PlannerParams pa = p;
    pa.search_alg = "rrt";
    PlannerCore c(pa);
    PlannerInput in = makeInput(straightRoad(true, true), 1.0, 0.9, true);
    in.map.resolution = 0.5;
    in.map.width = 120;
    in.map.height = 100;
    in.map.origin_x = 0.0;
    in.map.origin_y = -25.0;
    in.map.data.assign(120 * 100, 0);  // 全自由栅格（unknown/空均可通行）
    auto r = c.plan(in);
    check(r.mode == PlanMode::SEARCH && r.method == "search" &&
              std::string(r.algorithm) == "rrt" && !r.emergency_stop &&
              r.path.size() > 2,
          "rrt: good loc + goal -> direct RRT path");
  }

  // 11) FreePlanner + A*：无走廊 + 定位好 + 终点可用，free_alg=astar → 纯栅格 A* 搜索
  {
    PlannerParams pa = p;
    pa.free_alg = "astar";
    PlannerCore c(pa);
    // 先用双侧建立走廊，再双侧超时进入 MISSING_TIMEOUT → free 方法
    c.plan(makeInput(straightRoad(true, true), 1.0, 0.9, true));
    PlannerInput in = makeInput(straightRoad(false, false), 10.0, 0.9, true);
    in.map.resolution = 0.5;
    in.map.width = 120;
    in.map.height = 100;
    in.map.origin_x = 0.0;
    in.map.origin_y = -25.0;
    in.map.data.assign(120 * 100, 0);
    auto r = c.plan(in);
    check(r.method == "free" && std::string(r.algorithm) == "astar" &&
              !r.emergency_stop && r.path.size() > 2,
          "free + astar: no corridor, good loc -> pure grid A* path");
  }

  std::cout << (failures == 0 ? "\nALL PASSED\n" : "\nSOME FAILED\n");
  return failures == 0 ? 0 : 1;
}
