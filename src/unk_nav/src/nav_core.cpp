#include "unk_nav/nav_core.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

#include "unk_nav/curve_fit.h"
#include "unk_nav/geom_util.h"
#include "unk_nav/grid_util.h"
#include "unk_nav/road_follow.h"
#include "unk_nav/speed_planner.h"
#include "unk_nav/subgoal.h"

namespace unk {

// ── 公共接口（在 nav_core.h 中声明）──────────────────────────────

NavCore::NavCore(const NavParams& p) : p_(p), fsm_(p) {}

void NavCore::reset() {
  fsm_.reset();
  work_grid_ = GridMap();
  have_goal_ = false;
  last_goal_ = Point2D();
  last_ = NavResult();
  have_last_bearing_ = false;
  kappa_ema_ = 0.0;
  chain_filter_valid_ = false;
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
    // 换终点不能沿用旧方向记忆
    have_last_bearing_ = false;
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
  // 顺序不可颠倒：先膨胀再清洞，否则膨胀层会把车自己埋进障碍，车脚格被误判为
  // 不可行（射线自由距离与限速复查都会被糊弄）。
  // 传 raw：足迹洞只抹掉「膨胀出来的」格，lidar 真打到的墙保持 occupied。
  // 否则车贴墙到 footprint_clear_radius 以内时近场被抹平，规划器以为脚下是空地
  // （膨胀带与弦碰撞检测全部失效）。车中心格在 raw 里必为 free/unknown，仍被清成
  // free，所以上面的死锁保护不受影响。
  if (!in.local_grid.empty()) {
    work_grid_ = grid::inflate(in.local_grid, p_.inflation_radius, p_.inflate_unknown);
    grid::clearFootprint(&work_grid_, 0.0, 0.0, p_.footprint_clear_radius,
                         &in.local_grid);
  } else {
    work_grid_ = GridMap();
  }

  // ---- 3) 子目标：沿路模式从道路走廊几何取，终点模式从全局终点投影取 ----
  // 两种来源产出同一个「窗口内车体系子目标」，下游直线路径/限速完全共用。
  subgoal::Result sg;
  // 链式跳点暂存（rr 作用域在下面的 if 内，需提到函数级才能组装样条控制点 / 写进 NavResult）
  Point2D chain_hops[16];
  int chain_hop_count = 0;
  if (p_.follow_road) {
    // 无定位：不读 in.goal / in.vehicle_pose，前进方向以车头（base 系 +x）为基准。
    if (!work_grid_.empty()) {
      // 链式前瞻：第 1 跳与单跳完全一致，额外量出落点切向与前方曲率；
      // 曲线拟合关闭时不做链（省算力，行为与旧版逐字节一致）。
      road::Result rr = p_.curve_fit_enable ? road::lookAheadChain(work_grid_, p_)
                                            : road::lookAhead(work_grid_, p_);
      sg.valid = rr.valid;
      sg.point = rr.point;
      sg.reach = rr.reach;
      sg.bearing = rr.bearing;
      sg.truncated_by_obstacle = rr.truncated_by_obstacle;
      sg.candidates = std::move(rr.candidates);  // 扇形候选（调试可视化）
      if (rr.valid) {
        goal_dist = std::hypot(rr.point.x, rr.point.y);  // 仅调试/可视化用
        goal_bearing = rr.bearing;
        // 链式跳点透传给 NavResult（可视化：原点→P1→P2→P3 接力折线）
        chain_hop_count = std::min(rr.hop_count, 16);
        for (int i = 0; i < chain_hop_count; ++i) chain_hops[i] = rr.hops[i];
        // 曲率 κ 跨帧 EMA（纯诊断 / 状态文本用）；链截断（<2 跳）作废。
        if (rr.hop_count >= 2) {
          const double alpha = std::min(std::max(p_.chain_ema_alpha, 1e-3), 1.0);
          if (chain_filter_valid_) {
            kappa_ema_ += alpha * (rr.kappa_est - kappa_ema_);
          } else {
            kappa_ema_ = rr.kappa_est;
            chain_filter_valid_ = true;
          }
        } else {
          chain_filter_valid_ = false;  // 链截断 → 滤波作废，本帧走直线
        }
      } else {
        chain_filter_valid_ = false;
      }
    }
  } else if (in.goal_valid && !work_grid_.empty()) {
    sg = subgoal::project(work_grid_, goal_base, p_,
                          have_last_bearing_ ? &last_subgoal_bearing_ : nullptr);
    // 终点模式链式前瞻：用 goal_bearing 偏向子目标方向，得到跳点供样条拟合。
    // goal_align_w=0 时打分无偏向（沿路默认），>0 时射线越朝子目标打分越高。
    if (sg.valid && p_.curve_fit_enable) {
      road::Result rr = road::lookAheadChain(work_grid_, p_, 0.0, goal_bearing);
      if (rr.valid) {
        chain_hop_count = std::min(rr.hop_count, 16);
        for (int i = 0; i < chain_hop_count; ++i) chain_hops[i] = rr.hops[i];
        if (rr.hop_count >= 2) {
          const double alpha = std::min(std::max(p_.chain_ema_alpha, 1e-3), 1.0);
          if (chain_filter_valid_) {
            kappa_ema_ += alpha * (rr.kappa_est - kappa_ema_);
          } else {
            kappa_ema_ = rr.kappa_est;
            chain_filter_valid_ = true;
          }
        } else {
          chain_filter_valid_ = false;
        }
      } else {
        chain_filter_valid_ = false;
      }
    }
  }

  // ---- 4) 路径生成：条件满足时 Catmull-Rom 样条，否则直线（车→子目标）----
  // 两种模式共用样条：沿路模式跳点来自走廊几何；终点模式跳点带 goal_align_w 偏向子目标。
  // 直线是构造性保底（落点在自由射线上）；曲线鼓包可能扫进膨胀区，
  // 拟合内部已做碰撞复查，失败自动回退直线。
  Path path;
  bool curve_used = false;
  if (sg.valid) {
    if (p_.curve_fit_enable && chain_filter_valid_) {
      // Catmull-Rom 样条：曲线经过 车位O → 各跳跳点 P1→P2(→P3)，切向由相邻跳点差分自动定。
      std::vector<Point2D> ctrl;
      ctrl.reserve(static_cast<size_t>(chain_hop_count) + 1);
      ctrl.push_back(Point2D{0.0, 0.0});
      for (int i = 0; i < chain_hop_count; ++i) ctrl.push_back(chain_hops[i]);
      curve_used = curve::fitSpline(work_grid_, ctrl, 0.0, p_.path_spacing,
                                    p_.curvature_baseline, &path);
    }
    if (!curve_used) {
      const double dx = sg.point.x, dy = sg.point.y;
      const double len = std::hypot(dx, dy);
      const int n = std::max(1, static_cast<int>(std::ceil(len / p_.path_spacing)));
      for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        PathPoint nd;
        nd.p = Point2D{dx * t, dy * t};
        nd.s = len * t;
        nd.k = 0.0;  // 直线曲率 0
        path.push_back(nd);
      }
    }
  }

  // ---- 5) 速度规划 ----
  // 停车视距：仅终点模式接入。子目标被膨胀带截断 → 停在膨胀带边缘前；
  // 子目标即终点（goal_limited）→ 停在终点，消除 v_max 冲过终点的过冲。
  // road 模式本轮不接入：rr.truncated_by_obstacle 的语义包含“走廊弯曲”，
  // 直接当停车视距会在弯道上无谓减速。
  double stop_horizon = std::numeric_limits<double>::infinity();
  if (!p_.follow_road && sg.valid) {
    if (sg.truncated_by_obstacle) {
      stop_horizon = std::min(stop_horizon, sg.ray_free_dist);
    }
    if (sg.goal_limited) {
      stop_horizon = std::min(stop_horizon, sg.reach);
    }
  }
  const speed::Result sp = speed::limit(path, work_grid_, in.current_speed, p_, stop_horizon);

  // ---- 6) 行为状态机 ----
  fsm::Context ctx;
  ctx.now = in.now;
  // 沿路模式任务恒在（只要栅格有效就总有"路"要跟），不依赖全局终点；终点模式沿用 in.goal_valid。
  ctx.goal_valid = p_.follow_road ? !work_grid_.empty() : in.goal_valid;
  ctx.goal_dist = goal_dist;
  ctx.goal_bearing = goal_bearing;
  ctx.plan_ok = !path.empty();
  ctx.emergency_stop = sp.emergency_stop;
  ctx.current_speed = in.current_speed;
  ctx.speed_valid = in.speed_valid;
  const NavState st = fsm_.update(ctx);

  // ---- 7) 组装输出 ----
  NavResult r;
  r.state = st;
  r.subgoal = sg.point;
  r.subgoal_reachable = !path.empty();
  r.fan_candidates = std::move(sg.candidates);  // 扇形候选（调试可视化）
  r.goal_base = goal_base;
  r.goal_base_valid = in.goal_valid;
  r.kappa_est = chain_filter_valid_ ? kappa_ema_ : 0.0;
  // 链式前瞻可视化透传（两种模式均可产生跳点；hop_count=0 时 nav_node 不画链）
  r.chain_hop_count = chain_hop_count;
  for (int i = 0; i < chain_hop_count; ++i) r.chain_hops[i] = chain_hops[i];
  r.curve_used = curve_used;

  std::ostringstream oss;
  oss << navStateName(st) << "/" << fsm_.detail();
  if (!path.empty()) oss << " limit_by=" << sp.limit_by;
  // 子目标被膨胀区截断过：终点方向有墙，本周期只走到带子边缘。
  // 在 /unk_nav/state 里可见，方便区分「正常前进」和「贴带缓行」。
  if (sg.truncated_by_obstacle) oss << " subgoal_trunc";
  if (sg.fan_used) oss << " subgoal_fan";
  if (curve_used) oss << " curve";
  if (chain_filter_valid_) {
    oss << " kappa=" << static_cast<int>(kappa_ema_ * 1000.0) / 1000.0;
  }

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
      oss << " (no subgoal: " << subgoal::failReasonName(sg.fail);
      if (work_grid_.empty()) oss << " / empty grid";
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
  // 存本帧子目标方位角供下帧方向滞后用
  if (sg.valid && !p_.follow_road) {
    last_subgoal_bearing_ = sg.bearing;
    have_last_bearing_ = true;
  }
  last_ = r;
  return r;
}

}  // namespace unk
