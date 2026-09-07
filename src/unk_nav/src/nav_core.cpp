#include "unk_nav/nav_core.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "unk_nav/astar.h"
#include "unk_nav/geom_util.h"
#include "unk_nav/grid_util.h"
#include "unk_nav/path_smooth.h"
#include "unk_nav/speed_planner.h"
#include "unk_nav/subgoal.h"

namespace unk {

// ── 内部辅助（仅本文件使用）──────────────────────────────────────
namespace {

astar::Options makeAstarOptions(const NavParams& p, double resolution) {
  astar::Options o;
  o.unknown_cost = p.unknown_cost;
  o.max_iter = p.astar_max_iter;
  o.w = p.astar_w;
  o.soft_k = p.obstacle_cost_k;
  o.soft_sigma = p.obstacle_cost_sigma;
  // 吸附半径以米给出，这里才换算成格数 → 换分辨率时物理行为不变
  o.goal_snap_radius =
      resolution > 0.0 ? std::max(1, static_cast<int>(std::ceil(p.goal_snap_dist / resolution)))
                       : 10;
  // 倒角半径由目标过弯速度反解，同时满足 w = v/r <= w_max 与 v^2/r <= a_lat_max
  o.smooth_fillet_radius =
      smooth::filletRadiusForCornerSpeed(p.smooth_corner_speed, p.w_max, p.a_lat_max);
  o.smooth_laplacian_iters = p.smooth_laplacian_iters;
  o.smooth_laplacian_lambda = p.smooth_laplacian_lambda;
  o.smooth_shrink_retry = p.smooth_shrink_retry;
  o.curvature_baseline = p.curvature_baseline;
  return o;
}

}  // namespace

// ── 公共接口（在 nav_core.h 中声明）──────────────────────────────

NavCore::NavCore(const NavParams& p) : p_(p), fsm_(p) {}

void NavCore::reset() {
  fsm_.reset();
  work_grid_ = GridMap();
  have_goal_ = false;
  last_goal_ = Point2D();
  last_ = NavResult();
  last_path_odom_.clear();
  prev_path_base_.clear();
  have_prev_path_ = false;
}

void NavCore::setParams(const NavParams& p) {
  p_ = p;
  fsm_.setParams(p);
}

void NavCore::detectGoalChange(const NavInput& in) {
  if (!in.goal_valid) {
    have_goal_ = false;
    return;
  }
  // 终点移动超过一个容差视为换了目标 → 清掉 ARRIVED/ABORT 锁存与脱困计数
  const bool moved =
      !have_goal_ || std::hypot(in.goal.x - last_goal_.x, in.goal.y - last_goal_.y) >
                         std::max(p_.goal_tolerance, 0.05);
  if (moved) {
    fsm_.newGoal();
    // 换了目标：上帧路径通往旧目标，不能再当一致性吸引子，否则会把车拽向旧路。
    have_prev_path_ = false;
    last_path_odom_.clear();
  }
  last_goal_ = in.goal;
  have_goal_ = true;
}

NavResult NavCore::plan(const NavInput& in) {
  detectGoalChange(in);

  // ---- 1) 终点 → 车体系 ----
  Point2D goal_base;
  double goal_dist = 0.0;
  double goal_bearing = 0.0;
  if (in.goal_valid) {
    goal_base = geom::globalToBase(in.goal, in.vehicle_pose);
    goal_dist = std::hypot(goal_base.x, goal_base.y);
    goal_bearing = std::atan2(goal_base.y, goal_base.x);
  }

  // ---- 2) 栅格预处理：膨胀 + 车体足迹清洞 ----
  // 顺序不可颠倒：先膨胀再清洞，否则膨胀层会把车自己埋进障碍，A* 起点即死锁。
  if (!in.local_grid.empty()) {
    work_grid_ = grid::inflate(in.local_grid, p_.inflation_radius, p_.inflate_unknown);
    grid::clearFootprint(&work_grid_, 0.0, 0.0, p_.footprint_clear_radius);
  } else {
    work_grid_ = GridMap();
  }

  // ---- 3) 子目标投影（远处终点 → 窗口内可搜索目标）----
  subgoal::Result sg;
  if (in.goal_valid && !work_grid_.empty()) {
    sg = subgoal::project(work_grid_, goal_base, p_);
  }

  // ---- 4) 局部 A* ----
  Path path;
  if (sg.valid) {
    astar::Options ao = makeAstarOptions(p_, work_grid_.resolution);
    // 一致性软代价：把上帧路径（odom 系）重投影到当前 base 系作为吸引子。
    // 车已移动，必须重投影，否则一致性带会滞后错位。
    if (p_.consistency_k > 0.0 && have_prev_path_ && !last_path_odom_.empty()) {
      prev_path_base_.clear();
      prev_path_base_.reserve(last_path_odom_.size());
      for (const auto& pw : last_path_odom_) {
        prev_path_base_.push_back(geom::globalToBase(pw, in.vehicle_pose));
      }
      ao.prev_path = &prev_path_base_;
    }
    path = astar::plan(work_grid_, Point2D{0.0, 0.0}, sg.point, p_.path_spacing, ao, &ws_);
  }

  // ---- 5) 速度规划 ----
  const speed::Result sp = speed::limit(path, work_grid_, in.current_speed, p_);

  // ---- 6) 行为状态机 ----
  fsm::Context ctx;
  ctx.now = in.now;
  ctx.goal_valid = in.goal_valid;
  ctx.goal_dist = goal_dist;
  ctx.goal_bearing = goal_bearing;
  ctx.plan_ok = !path.empty();
  ctx.emergency_stop = sp.emergency_stop;
  ctx.current_speed = in.current_speed;
  const NavState st = fsm_.update(ctx);

  // ---- 7) 组装输出 ----
  NavResult r;
  r.state = st;
  r.subgoal = sg.point;
  r.subgoal_reachable = !path.empty();

  std::ostringstream oss;
  oss << navStateName(st) << "/" << fsm_.detail();
  if (!path.empty()) oss << " limit_by=" << sp.limit_by;
  // 子目标被膨胀区截断过：终点方向有墙，本周期只走到带子边缘。
  // 在 /unk_nav/state 里可见，方便区分「正常前进」和「贴带缓行」。
  if (sg.truncated_by_obstacle) oss << " subgoal_trunc";

  // 终态与无效输入：一律停车，且不输出路径（下游不该去跟踪一条通往已结束任务的路）
  if (st == NavState::IDLE || st == NavState::ARRIVED || st == NavState::ABORT) {
    if (st == NavState::IDLE && !in.goal_valid) oss << " (no goal)";
    r.emergency_stop = true;
    r.recommended_speed = 0.0;
    r.reason = oss.str();
    last_ = r;
    return r;
  }

  if (path.empty()) {
    r.emergency_stop = true;
    r.recommended_speed = 0.0;
    if (!sg.valid) {
      oss << " (no subgoal";
      if (work_grid_.empty()) oss << ": empty grid";
      oss << ")";
    } else {
      oss << " (subgoal unreachable)";
    }
    r.reason = oss.str();
    last_ = r;
    return r;
  }

  r.path = path;
  r.emergency_stop = sp.emergency_stop;
  r.recommended_speed = sp.emergency_stop ? 0.0 : sp.v;
  if (sp.emergency_stop) oss << " (estop:" << sp.limit_by << ")";
  r.reason = oss.str();
  // 记住本帧路径（转 odom 系）供下一帧做一致性吸引子。仅成功出路径时更新；
  // 偶发失败帧保留更早的路径，不因一帧丢记忆。
  last_path_odom_.clear();
  last_path_odom_.reserve(path.size());
  for (const auto& pp : path) {
    last_path_odom_.push_back(geom::baseToGlobal(pp.p, in.vehicle_pose));
  }
  have_prev_path_ = true;
  last_ = r;
  return r;
}

}  // namespace unk
