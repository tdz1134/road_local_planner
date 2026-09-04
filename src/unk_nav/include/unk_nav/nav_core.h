#pragma once
// 门面：单周期总调度。外部只需要认识 NavCore + NavInput + NavResult 三个东西。
//
// 每周期内部流程：
//   终点转车体系 → 栅格膨胀 + 足迹清洞 → 子目标投影 → 局部 A* → 速度规划 → 行为状态机
//
// 跨周期状态只有两处：BehaviorFsm 的记忆，以及上一帧结果（调试用）。
#include <string>

#include "unk_nav/astar.h"
#include "unk_nav/behavior_fsm.h"
#include "unk_nav/types.h"

namespace unk {

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

  NavParams p_;
  fsm::BehaviorFsm fsm_;
  GridMap work_grid_;
  astar::Workspace ws_;  // A* 搜索工作区，跨周期复用 → 每周期零堆分配

  bool have_goal_ = false;
  Point2D last_goal_;
  NavResult last_;
};

}  // namespace unk
