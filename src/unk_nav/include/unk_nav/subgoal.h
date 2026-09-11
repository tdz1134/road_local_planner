#pragma once
// 子目标生成：把窗口外的远处终点投影成局部栅格内的一个可搜索目标点。
//
// 这是「没有全局地图」这一约束的直接产物：A* 只能搜到窗口内，所以每周期都要
// 重新决定「这一步朝哪儿走」。
//
// ── 选取策略：中心优先 + 截断才扇形 ──────────────────────────────
// 先沿终点方向（θ=goal_bearing）单射线投影。若中心方向一路空到前瞻距离 → 直接返回，
// 零额外开销、行为与旧版单射线一致。
// 仅当中心方向被障碍截短（d_free < reach_clip）时，才展开扇形候选：
//   θᵢ = goal_bearing + i·Δ，i ∈ [-N, N]，Δ = subgoal_fan_step_deg
// 每条候选量自由距离 reach_i = min(lookahead, t_win(θᵢ))，硬门槛 reach_i ≥ subgoalMin*0.5，
// 打分 score = align_w·cos(θᵢ-θ_goal) + free_w·min(reach_i/subgoalMin, 1) + prev_w·cos(θᵢ-θ_prev)，
// 取 argmax。侧向候选不受 goal_dist 限制（它们本就不是去终点，是找绕行口）。
// subgoal_fan_half_deg=0 → 关闭扇形，退回单射线。
#include <vector>

#include "unk_nav/types.h"

namespace unk {
namespace subgoal {

// 失败原因枚举（调试 / reason 串用）
enum class FailReason {
  kNone,            // 成功
  kGridEmpty,       // 输入栅格为空
  kGoalTooClose,    // 终点几乎重合于车位（< 1mm）
  kWindowTooSmall,  // 窗口内缩后无法容纳子目标
  kVehicleOutside,  // 车体不在窗口内
  kWindowTooShort,  // 被窗口截得太短且非末段收敛
  kNoFeasible,      // 整条射线无可行落点（车被膨胀区围死）
  kNoCandidate,     // 扇形展开后全部候选被硬门槛淘汰
  kTooShort,        // 最终 reach 不足门槛
};

inline const char* failReasonName(FailReason f) {
  switch (f) {
    case FailReason::kNone:           return "none";
    case FailReason::kGridEmpty:      return "grid_empty";
    case FailReason::kGoalTooClose:   return "goal_too_close";
    case FailReason::kWindowTooSmall: return "window_too_small";
    case FailReason::kVehicleOutside: return "vehicle_outside";
    case FailReason::kWindowTooShort: return "window_too_short";
    case FailReason::kNoFeasible:     return "no_feasible";
    case FailReason::kNoCandidate:    return "no_candidate";
    case FailReason::kTooShort:       return "too_short";
  }
  return "?";
}

struct Result {
  Point2D point;                 // 子目标，车体系
  bool valid = false;            // 是否成功生成
  double goal_dist = 0.0;        // 终点在车体系的距离 m
  double goal_bearing = 0.0;     // 终点在车体系的方位角 rad（0 = 正前方）
  double reach = 0.0;            // 实际投影距离 m
  double bearing = 0.0;          // 选中方位角 rad（与 road::Result 对称）
  double ray_free_dist = 0.0;    // 沿选中射线首个不可行点的距离 m（停车视距用）
  bool clipped_by_window = false;        // 投影距离是否被窗口边界截短
  bool truncated_by_obstacle = false;    // 落点是否因撞进膨胀/障碍区而沿射线回退过
  bool goal_limited = false;     // reach 是否被终点距离限制（末段收敛）
  bool fan_used = false;         // 是否使用了扇形展开（而非中心直射）
  FailReason fail = FailReason::kNone;  // 失败原因
};

// 沿终点方向投影（中心优先 + 截断才扇形展开）。
//
// prev_bearing: 上帧选中的方位角（用于方向滞后防翻烧饼），nullptr = 无记忆。
//   默认参数保证既有 3 参调用点不破。
Result project(const GridMap& grid, const Point2D& goal_base, const NavParams& p,
               const double* prev_bearing = nullptr);

}  // namespace subgoal
}  // namespace unk
