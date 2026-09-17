#pragma once
// 门面：单周期总调度。外部只需要认识 NavCore + NavInput + NavResult 三个东西。
//
// 每周期内部流程：
//   终点转车体系 → 栅格膨胀 + 足迹清洞 → 子目标投影 → 路径生成（沿路曲线/直线）
//   → 速度规划 → 行为状态机
//
// 跨周期状态：BehaviorFsm 的记忆、子目标方向滞后、链式前瞻 EMA 滤波、上一帧结果（调试用）。
#include <string>

#include "unk_nav/behavior_fsm.h"
#include "unk_nav/types.h"

namespace unk {

namespace road { struct Result; }  // forward decl

class NavCore {
 public:
  explicit NavCore(const NavParams& p);

  // 单周期规划。线程不安全，调用方需保证串行。
  NavResult plan(const NavInput& in);

  // 清空全部跨周期状态（换任务、重新上电、仿真复位时调用）
  void reset();

  const NavParams& params() const { return p_; }
  void setParams(const NavParams& p);

  // 调试：本周期实际用于搜索的工作栅格（已膨胀 + 已清足迹）
  const GridMap& workGrid() const { return work_grid_; }
  const fsm::BehaviorFsm& fsm() const { return fsm_; }

 private:
  // 终点变化检测：换终点时通知状态机清终态
  void detectGoalChange(const NavInput& in);

  // 从 road::Result 提取链式跳点 + kappa EMA 更新（沿路/终点两种模式共用）
  void absorbChain(const road::Result& rr);

  // 直线保底路径：车→子目标，k=0
  static Path makeStraightPath(const Point2D& target, double spacing);

  NavParams p_;
  fsm::BehaviorFsm fsm_;
  GridMap work_grid_;

  bool have_goal_ = false;
  Point2D last_goal_;
  NavResult last_;

  // 子目标方向滞后（防翻烧饼）：存上帧选中的子目标方位角，传给 subgoal::project 作为打分参考。
  double last_subgoal_bearing_ = 0.0;
  bool   have_last_bearing_ = false;

  // 链式前瞻曲率 κ 的跨帧 EMA 滤波：扇形 3° 量化让 κ 逐帧抖动，一阶低通压噪。
  // 链截断（hop_count<2）或子目标无效时作废 → 本帧走直线。
  // 拟合切向不再单独滤波：曲线由 fitSpline 直接过 hops 各跳点，切向来自相邻点差分。
  double kappa_ema_ = 0.0;
  bool   chain_filter_valid_ = false;

  // 链式跳点（本帧）：absorbChain 填入，路径生成和输出组装读取。
  Point2D chain_hops_[16];
  int     chain_hop_count_ = 0;
};

}  // namespace unk
