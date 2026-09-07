#pragma once
// 导航行为状态机。
//
// 五个状态都会真实产生：IDLE / GO / RECOVERY / ARRIVED / ABORT。
//
// RECOVERY 只上报、不产生脱困运动 —— 脱困运动原语与凹形障碍绕行（Bug 式
// BOUNDARY_FOLLOW）都已明确排除在本模块范围外（用户决策，见 README「能力边界」）。
// 因此本状态机对「走不通」的处理是：连续失败 → RECOVERY 上报若干周期 → ABORT 放弃，
// 把决定权交回上层。它不会尝试自己爬出来。
//
// 所有跨周期记忆都保存在这里，nav_core 本身无状态。
#include "unk_nav/types.h"

namespace unk {
namespace fsm {

// 单周期判据，由 nav_core 组装后喂进来
struct Context {
  double now = 0.0;
  bool goal_valid = false;
  double goal_dist = 0.0;      // 终点在车体系的距离 m
  double goal_bearing = 0.0;   // 终点方位角 rad
  bool plan_ok = false;        // 本周期是否搜到了路径
  bool emergency_stop = true;  // 速度层是否要求急停
  double current_speed = 0.0;
  bool speed_valid = true;     // 车速是否来自有效测量；沿路模式前进位移棘轮依赖它，
                               // false（无里程计）时该棘轮不可用，仅靠规划失败判定兜底
};

class BehaviorFsm {
 public:
  explicit BehaviorFsm(const NavParams& p) : p_(p) {}

  // 推进一个周期，返回新状态
  NavState update(const Context& ctx);

  NavState state() const { return state_; }
  int retryCount() const { return retry_; }
  const char* detail() const { return detail_; }

  // 运行中改参数（保留状态记忆）
  void setParams(const NavParams& p) { p_ = p; }

  // ARRIVED / ABORT 是锁存终态，只有 reset() 或换终点才会退出
  void reset();
  // 换了新终点时调用：清终态与脱困计数，但保留卡死检测窗口
  void newGoal();

 private:
  // 无进展判定：在 stuck_time 内没能朝终点实质性推进（见 .cpp 说明为什么用「推进」
  // 而不是「位移」）；true 表示本周期触发了一次重试计数
  bool checkStuck(const Context& ctx);
  // 沿路模式无进展判定：无全局终点，改用「带符号前进位移」棘轮（∫current_speed·dt）
  bool checkStuckRoad(const Context& ctx);
  // 规划失败（连续被挡）判定
  bool checkPlanFail(const Context& ctx);
  // 触发一次重试：累加计数，超限转 ABORT
  NavState onStuckTriggered(const char* detail);

  NavParams p_;
  NavState state_ = NavState::IDLE;
  int retry_ = 0;
  const char* detail_ = "init";

  // 规划失败计时
  bool fail_active_ = false;
  double fail_since_ = 0.0;

  // 无进展棘轮：记录「历史最好终点距离」及其取得时刻。
  // 只有实质性地更接近终点才刷新，否则计时一直累加。
  bool progress_valid_ = false;
  double best_goal_dist_ = 0.0;
  double best_goal_time_ = 0.0;

  // 沿路模式无进展棘轮：前进位移 = ∫current_speed·dt（body 系前向速度，本体感知非定位）。
  // 物理卡住(v≈0)或前后振荡(v 变号、净值≈0)都能抓住；正常前进过阈则刷新并清 retry_。
  bool road_progress_valid_ = false;
  double road_adv_ = 0.0;         // 累计前进位移
  double road_adv_base_ = 0.0;    // 棘轮基准位移
  double road_adv_t0_ = 0.0;      // 基准取得时刻
  double road_adv_last_t_ = 0.0;  // 上次积分时刻
};

}  // namespace fsm
}  // namespace unk
