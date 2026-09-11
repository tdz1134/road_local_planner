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
// 石块）只是带内的一撮 occupied 格，绕行是下游 A*（凸障碍强项）+ 障碍软代价回中的职责，
// 本模块只负责决定"这一步朝走廊的哪个方向探出去"。
//
// ── 怎么选方向 ─────────────────────────────────────────────────
// 在车头前向半球（±road_fan_half_deg）撒一把扇形射线，每条量"前方自由距离" d(θ)，
// 打分 score(θ) = road_free_w·min(d,L)/L + road_align_w·cos θ：
//   · 自由距离项 → 想往更空、能走更远的方向（弯道处内侧被外墙截短，外侧更长 → 跟弯）；
//   · 对齐项 cos θ → 偏好直行，避免无谓摆动，也保证"沿车头朝向的前向"这一无定位基准。
// 取 argmax 得 θ*，落点 = min(d(θ*), L) 处，并沿用 subgoal 的落点可行性截断（撞膨胀带
// 则沿射线向车侧回退到首个可行点）。路中有圆柱时正前方 d 被截短、旁边空隙 d 更长，
// θ* 自然偏向空隙 → A* 绕过后正前方又变最远 → 自动回中。
//
// 纯 C++14，零 ROS 依赖，只读 GridMap（用 feasibleAt：occupied/越界=false，free/unknown=true）。
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
};

// 从膨胀后的工作栅格中，沿车头前向半球选出道路前瞻子目标。
// work_grid：已膨胀 + 已清足迹的工作栅格（与 subgoal::project 的入参一致，车体系）。
// 失败（栅格空 / 前向全不可行）返回 valid=false。
Result lookAhead(const GridMap& work_grid, const NavParams& p);

}  // namespace road
}  // namespace unk
