#pragma once
// 沿路前瞻：无定位模式下，从局部栅格的**道路走廊几何**直接推出一个车体系前瞻子目标。
//
// 这是「无定位沿路行驶」的核心：终点模式靠 `globalToBase(goal, pose)` 把窗口外的全局
// 终点换算成车体系方向（见 subgoal.h），需要定位；本模块不需要任何全局位姿，前进方向
// 完全以**车头朝向**（base 系 +x）为基准，从局部栅格里"看"出道路往哪延伸。
//
// ── 道路是什么（路线 A）──────────────────────────────────────────
// 道路 = 两侧路缘/墙夹出的可通行自由走廊。激光打到两侧边界 → 栅格形成一条 free 带，
// 这条带就是路。**不需要任何显式语义层**：走廊由横向边界界定，路中间的小障碍（圆柱、
// 石块）只是带内的一撮 occupied 格，绕行是下游规划 + 障碍软代价回中的职责，
// 本模块只负责决定"这一步朝走廊的哪个方向探出去"。
//
// ── 怎么选方向 ─────────────────────────────────────────────────
// 在车头前向半球（±road_fan_half_deg）撒一把扇形射线，每条量"前方自由距离" d(θ)，
// 打分 score(θ) = road_free_w·min(d,L)/L + road_align_w·cos θ：
//   · 自由距离项 → 想往更空、能走更远的方向（弯道处内侧被外墙截短，外侧更长 → 跟弯）；
//   · 对齐项 cos θ → 偏好直行，避免无谓摆动，也保证"沿车头朝向的前向"这一无定位基准。
// 取 argmax 得 θ*，落点 = min(d(θ*), L) 处。路中有圆柱时正前方 d 被截短、旁边空隙
// d 更长，θ* 自然偏向空隙 → 绕过后正前方又变最远 → 自动回中。
//
// ── 接力前瞻（Relay Lookahead, RLA；旧称链式前瞻，即 lookAheadChain）──
// 一句话画像：**看远·走近·接力 n 跳**——扇形扫描视野 L 只用来选方向（看远防短视），
// 每跳落点只沿选中方向前进一小步（走近 = min(road_step_dist, road_step_ratio×L)），
// 把虚拟车挪到落点接力再扫、重复 chain_hops 次串成 hops 链。帧内接力成链 + 帧间滚动重规划。
// 单跳扫描只回答"从车这里看哪个方向最空"，落点方向是**弦向**，不含"路接下来往哪弯"。
// 链式版把扫描接力做 chain_hops 跳：在第 1 跳落点放"虚拟车"（朝向 = 到达方向）再扫，
// 量出道路在落点处的继续转角 θ₂'：
//   · 终点切向修正：圆弧的切向 = 弦向 + 转角的一半 → tangent_end = θ₁ + θ₂'/2；
//   · 前方曲率：κ = Σ方向变化 / Σ弧长（左正右负），供弯道限速与曲线拟合。
// 后续跳失败（落点处被封死/贴窗口边缘）→ 链截断，退化为单跳结果，不报错。
// 本质是每周期现造一条寿命 100ms 的"微型参考线"，不需要显式道路参考线/Frenet 系。
//
// 纯 C++14，零 ROS 依赖，只读 GridMap（用 feasibleAt：occupied/越界=false，free/unknown=true）。
#include <cmath>
#include <limits>
#include <vector>

#include "unk_nav/types.h"

namespace unk {
namespace road {

struct Result {
  Point2D point;                     // 前瞻子目标，车体系（base_link）
  bool valid = false;                // 是否找到可通行的前向（全被堵则 false）
  double bearing = 0.0;              // 选中的方向 θ*，rad（0 = 正前方）
  double reach = 0.0;                // 子目标实际距离 m
  bool truncated_by_obstacle = false; // 落点是否因撞进膨胀/障碍区而沿射线回退过
  std::vector<NavResult::FanCandidate> candidates;  // 扇形候选（调试可视化用，沿路每帧展开故总是非空）

  // ── 链式前瞻扩展（lookAheadChain 精化；lookAhead 给单跳退化默认值）──
  double tangent_end = 0.0;   // 落点处道路切向估计 rad（单跳：= bearing）。纯诊断量，拟合已改用 hops 样条
  double kappa_est = 0.0;     // 前方曲率估计 1/m，左正右负（单跳：0）
  Point2D hops[16];           // 各跳落点（hops[0]=point=P1、hops[1]=P2…），供 fitSpline 逐点过；容量 16
  int hop_count = 0;          // 成功跳数（1..16）
};

// 从膨胀后的工作栅格中，沿车头前向半球选出道路前瞻子目标（单跳）。
// work_grid：已膨胀 + 已清足迹的工作栅格（与 subgoal::project 的入参一致，车体系）。
// current_speed：当前车速 m/s，仅用于“看多深”随速度放大（见 NavParams::roadLookahead(v)）；0=不随速度。
// 失败（栅格空 / 前向全不可行）返回 valid=false。
Result lookAhead(const GridMap& work_grid, const NavParams& p, double current_speed = 0.0);

// 接力前瞻（Relay Lookahead, RLA；旧称链式前瞻）：第 1 跳与 lookAhead 完全一致（零回归），随后接力 chain_hops-1 跳
// 估计落点处道路切向（tangent_end）与前方曲率（kappa_est）。链截断时优雅退化。
// start_heading：第一跳扫描朝向（0=车头，沿路模式默认）；终点模式可传 sg.bearing。
// goal_bearing：子目标绝对方位角 rad；非 NAN 时打分加 goal_align_w × cos(射线−goal_bearing)。
//                沿路模式传 NAN（不加子目标项）；终点模式传 sg.bearing 偏向目标。
// current_speed：当前车速 m/s，只影响“看多深” L=roadLookahead(v)（越快看越远）；0=不随速度（零回归）。
Result lookAheadChain(const GridMap& work_grid, const NavParams& p,
                      double start_heading = 0.0,
                      double goal_bearing = std::numeric_limits<double>::quiet_NaN(),
                      double current_speed = 0.0);

}  // namespace road
}  // namespace unk
