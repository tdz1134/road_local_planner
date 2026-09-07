#include "unk_nav/behavior_fsm.h"

namespace unk {
namespace fsm {

// ── 公共接口（在 behavior_fsm.h 中声明）──────────────────────────

void BehaviorFsm::reset() {
  state_ = NavState::IDLE;
  retry_ = 0;
  detail_ = "reset";
  fail_active_ = false;
  fail_since_ = 0.0;
  progress_valid_ = false;
  road_progress_valid_ = false;
  road_adv_ = road_adv_base_ = road_adv_t0_ = road_adv_last_t_ = 0.0;
}

void BehaviorFsm::newGoal() {
  // 新终点：清掉锁存终态与重试计数。
  // 无进展棘轮也必须作废 —— 新终点的距离标度完全不同，沿用旧的 best_goal_dist_
  // 会让车在下一周期立刻被判「无进展」。
  if (state_ == NavState::ARRIVED || state_ == NavState::ABORT) state_ = NavState::GO;
  retry_ = 0;
  fail_active_ = false;
  progress_valid_ = false;
  road_progress_valid_ = false;
  road_adv_ = road_adv_base_ = road_adv_t0_ = road_adv_last_t_ = 0.0;
  detail_ = "new goal";
}

NavState BehaviorFsm::onStuckTriggered(const char* detail) {
  ++retry_;
  // 两个计时器都清零：给车一个完整的 stuck_time 窗口去证明这次能走出来。
  // 不清零的话，触发后下一周期条件依然成立，会在 3 个周期内（0.3s）直接烧完
  // 重试次数转 ABORT，RECOVERY 根本没起到「等栅格更新后重试」的作用。
  fail_active_ = false;
  progress_valid_ = false;
  road_progress_valid_ = false;  // 沿路棘轮也作废，给一个完整的 stuck_time 窗口重新证明
  if (retry_ > p_.recovery_max_retry) {
    state_ = NavState::ABORT;
    detail_ = "recovery exhausted";
    return state_;
  }
  // RECOVERY 只上报、不产生脱困运动（该能力已排除在本模块范围外）。
  // 上层会因 plan_ok=false 输出零速，下一周期栅格更新后重新尝试。
  state_ = NavState::RECOVERY;
  detail_ = detail;
  return state_;
}

bool BehaviorFsm::checkPlanFail(const Context& ctx) {
  if (!ctx.plan_ok || ctx.emergency_stop) {
    if (!fail_active_) {
      fail_active_ = true;
      fail_since_ = ctx.now;
    }
    return (ctx.now - fail_since_) >= p_.stuck_time;
  }
  fail_active_ = false;
  return false;
}

// 为什么判「朝终点的推进量」而不是「位移量」：
// 位移量判定有一个致命盲区 —— 车在凹形障碍里横向来回振荡时位移一直不小，判定永远
// 不触发，于是模块在零进展的情况下持续上报 GO + 满速。实测深凹槽场景：车在槽内
// 振荡 280 秒、距终点始终 14.0±0.1m，全程状态都是 GO。对集成方而言这是最坏的失败
// 方式 —— 不是报错，是谎报健康。
// 推进量判定严格覆盖位移判定（车真不动 ⇒ 距终点必然不缩小），且能抓住振荡。
// 代价是「必要的绕行」会短暂拉大终点距离，所以 stuck_dist 阈值必须小于绕行期间每
// stuck_time 的正常推进量（默认 0.05m / 3s，远低于 v_max*stuck_time = 0.66m）。
bool BehaviorFsm::checkStuck(const Context& ctx) {
  // 只在「规划说能走」时判无进展：规划本身就失败的情况由 checkPlanFail 负责
  if (!ctx.plan_ok || ctx.emergency_stop) return false;

  if (!progress_valid_) {
    best_goal_dist_ = ctx.goal_dist;
    best_goal_time_ = ctx.now;
    progress_valid_ = true;
    return false;
  }
  // 棘轮：只有「实质性地更接近终点」才刷新最好成绩与计时。
  if (ctx.goal_dist < best_goal_dist_ - p_.stuck_dist) {
    best_goal_dist_ = ctx.goal_dist;
    best_goal_time_ = ctx.now;
    return false;
  }
  return (ctx.now - best_goal_time_) >= p_.stuck_time;
}

// 沿路模式的无进展判定。无全局终点，改用「带符号前进位移」棘轮：
// 前进位移 = ∫ current_speed·dt（body 系前向速度，本体感知，非定位）。
//   · 物理卡住（v≈0）→ 位移不增 → 抓住；
//   · 前后振荡（v 变号）→ 净位移≈0 → 抓住；
//   · 正常前进 → 位移过阈 → 刷新棘轮并清 retry_（临时障碍绕过后恢复健康）。
// speed_valid=false（无里程计）时本判定不可用 → 返回 false，仅靠 checkPlanFail 兜底
//（见 README 能力边界：此时无法检测「有可行规划但被顶住不动」，属无定位降级）。
bool BehaviorFsm::checkStuckRoad(const Context& ctx) {
  // 只在「规划说能走」时判无进展；规划本身失败由 checkPlanFail 负责
  if (!ctx.plan_ok || ctx.emergency_stop) return false;
  if (!ctx.speed_valid) return false;  // 无有效车速 → 不判（避免 ∫0·dt 恒为0 误触发 ABORT）

  const double dt = ctx.now - road_adv_last_t_;
  road_adv_last_t_ = ctx.now;
  // 首周期或时间跳变时 dt 可能异常大，夹上限防止位移积分爆掉
  if (dt > 0.0 && dt < 1.0) road_adv_ += ctx.current_speed * dt;

  if (!road_progress_valid_) {
    road_adv_base_ = road_adv_;
    road_adv_t0_ = ctx.now;
    road_progress_valid_ = true;
    return false;
  }
  // 棘轮：实质性前进（超过 stuck_dist）才刷新基准与计时，并清 retry_（临时障碍已绕过）
  if (road_adv_ - road_adv_base_ >= p_.stuck_dist) {
    road_adv_base_ = road_adv_;
    road_adv_t0_ = ctx.now;
    retry_ = 0;
    return false;
  }
  return (ctx.now - road_adv_t0_) >= p_.stuck_time;
}

NavState BehaviorFsm::update(const Context& ctx) {
  // ---- 1) 无终点 ----
  if (!ctx.goal_valid) {
    state_ = NavState::IDLE;
    detail_ = "no goal";
    fail_active_ = false;
    return state_;
  }

  // ---- 2) 终态锁存 ----
  if (state_ == NavState::ARRIVED) {
    detail_ = "arrived, latched";
    return state_;
  }
  if (state_ == NavState::ABORT) {
    detail_ = "aborted, latched";
    return state_;
  }

  // ---- 3) 到达判定 ----
  // 只判距离，不判航向：差速底盘原地转向能力充足，强制对齐航向会让车在终点附近
  // 来回转圈，对 demo 与实车都是负收益。需要定向停靠时再显式加参数与判定。
  // 沿路模式无全局终点 → 永不 ARRIVED，跳过本判定（一直跟路直到被阻 ABORT 或外部停车）。
  if (!p_.follow_road && ctx.goal_dist <= p_.goal_tolerance) {
    state_ = NavState::ARRIVED;
    detail_ = "goal reached";
    return state_;
  }

  // ---- 4) 连续规划失败 ----
  if (checkPlanFail(ctx)) {
    return onStuckTriggered("plan blocked too long");
  }

  // ---- 5) 规划成功但没能推进 ----
  // 终点模式：朝终点推进量棘轮；沿路模式：带符号前进位移棘轮。
  if (p_.follow_road) {
    if (checkStuckRoad(ctx)) return onStuckTriggered("no road progress");
  } else if (checkStuck(ctx)) {
    return onStuckTriggered("no progress");
  }

  // ---- 6) 正常推进 ----
  state_ = NavState::GO;
  detail_ = ctx.plan_ok ? "go" : "go, waiting for feasible plan";
  return state_;
}

}  // namespace fsm
}  // namespace unk
