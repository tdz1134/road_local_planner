#pragma once
// 规划层公共定义：输入/输出、上下文、规划方法接口。
//
// 核心设计：不同"道路情况 × 定位情况"对应不同规划方法（PlannerBase 实现），
// 由 PlannerRouter 路由选择：
//   有走廊 + 定位差  -> FollowPlanner（走廊跟随，忽略终点）
//   有走廊 + 定位好  -> SearchPlanner（走廊族 + 终点扇形族）
//   无走廊 + 定位好  -> FreePlanner（终点方向直线族）
//   无走廊 + 定位差  -> 无可用方法 → 停车保护
#include <vector>

#include "rlp_common/types.h"
#include "rlp_road/road_types.h"

namespace rlp {
namespace planner {

// 一个规划周期的全部输入（节点层负责填充）
struct PlannerInput {
  double now = 0.0;                       // 单调递增时间（秒）
  road::BoundarySet boundaries;           // 左右边界（可能单侧/双侧缺失）
  GridMap map;                            // 局部占据栅格（车体系）
  double localization_quality = 0.0;      // 外部输入 [0,1]
  Point2D goal;                           // 全局终点（已转换到车体系）
  bool goal_valid = false;                // tf 不可用时为 false → 只能 FOLLOW
  double current_speed = 0.0;             // m/s
};

struct PlanResult {
  Path path;
  double recommended_speed = 0.0;
  PlanMode mode = PlanMode::FOLLOW;
  road::BoundaryState boundary_state = road::BoundaryState::MISSING_TIMEOUT;
  double corridor_confidence = 0.0;
  bool emergency_stop = false;
  std::string method;     // 实际使用的规划方法名（follow/search/free）
  std::string algorithm;  // 方法内部实际使用的候选生成算法名（调试用）
  std::string reason;
};

// 传给各规划方法的上下文
struct PlanningContext {
  const PlannerInput* input = nullptr;
  const road::Corridor* corridor = nullptr;  // 走廊无效时为 nullptr
  double margin = 0.0;                       // 边界安全边距（随速度放大）
  const Path* prev_path = nullptr;           // 上一周期路径（一致性）
};

// 规划方法接口：每种道路情况一个实现类，互相独立，便于单独替换/测试。
class PlannerBase {
 public:
  virtual ~PlannerBase() = default;
  virtual PlanMode mode() const = 0;
  virtual const char* name() const = 0;
  // 方法内部当前生效的候选生成算法名（仅带算法管理的方法类覆盖，调试用）
  virtual const char* algorithm() const { return ""; }
  // 基于当前道路情况生成候选路径；不适用时返回空（上层停车）
  virtual std::vector<Path> candidates(const PlanningContext& ctx) const = 0;
};

}  // namespace planner
}  // namespace rlp
