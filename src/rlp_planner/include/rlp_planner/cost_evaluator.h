#pragma once
// 候选路径代价评估（所有规划方法共用）：
// 碰撞(硬) + 越出走廊 + 平滑性 + 进度 + 与上一周期路径一致性。
// 模式差异体现在"进度"项：FOLLOW 用走廊弧长推进，SEARCH 用朝终点投影。
#include "rlp_planner/planner_base.h"

namespace rlp {
namespace planner {

class CostEvaluator {
 public:
  explicit CostEvaluator(const PlannerParams& p);

  // margin：随速度放大的边界安全边距（由 PlannerCore 计算传入）
  // prev_path 可为空（首帧）
  double evaluate(const Path& path, const GridMap& map, const road::Corridor& corridor,
                  PlanMode mode, const Point2D& goal, bool goal_valid,
                  const Path& prev_path, double margin) const;

 private:
  PlannerParams p_;
};

}  // namespace planner
}  // namespace rlp
