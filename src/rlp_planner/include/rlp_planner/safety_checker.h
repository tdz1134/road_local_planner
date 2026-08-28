#pragma once
// 安全校验（所有规划方法共用）：制动包络
// （v²/2a + 反应距离 + 余量 vs 路径上首个障碍距离），
// 并给出推荐速度（障碍距离、路径曲率、v_max 三者取小）。
#include "rlp_planner/planner_base.h"

namespace rlp {
namespace planner {

struct SafetyResult {
  bool pass = false;
  double obs_distance = 0.0;     // 路径上第一个占据点到起点弧长（无障碍时为极大值）
  double brake_distance = 0.0;   // 当前车速下的制动距离需求
  double max_safe_speed = 0.0;   // 推荐上限速度
};

class SafetyChecker {
 public:
  explicit SafetyChecker(const PlannerParams& p);

  SafetyResult check(const Path& path, const GridMap& map, double current_speed) const;

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
