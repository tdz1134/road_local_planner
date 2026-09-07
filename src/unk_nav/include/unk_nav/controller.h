#pragma once
// 纯跟踪控制器：从车体系路径中找前视点，算出 (v, w)。
//
// 输入：unk::Path（车体系，x 前 y 左）+ 推荐速度 + 膨胀后的工作栅格
// 输出：TwistCmd{v, w}（线速度 m/s，角速度 rad/s）
//
// 关键：纯跟踪实际走的是「车 → 前视点」的弦，而非路径本身。拐角处这条弦
// 会切进内侧，速度越高、前视越大切得越深，足以吃掉规划留出的膨胀安全边距
//（把车送进障碍层困死）。因此从前视点往近处收缩，取第一个「车→该点」弦
// 无碰撞的点，保证实际走的弦不穿障碍。work_grid 由 NavCore::workGrid() 提供。
//
// 纯 C++14，零 ROS 依赖。unk_nav_sim 的 nav_node 负责把 TwistCmd 转成
// geometry_msgs::Twist 发布到 /cmd_vel。
//
// 替换方式：改 compute() 实现，或新建一个类保持相同签名。

#include <cmath>
#include <algorithm>
#include "unk_nav/types.h"
#include "unk_nav/grid_util.h"

namespace unk {

// 控制器输出（与 ROS 解耦的速度指令）
struct TwistCmd {
  double v = 0.0;  // 线速度 m/s
  double w = 0.0;  // 角速度 rad/s
};

// 纯跟踪控制器（带弦碰撞检测收缩）
class PurePursuitController {
 public:
  // lookahead: 前视距离 m
  // w_max:     角速度限幅 rad/s
  PurePursuitController(double lookahead, double w_max)
      : lookahead_(lookahead), w_max_(w_max) {}

  // 从车体系路径 + 推荐速度 + 工作栅格计算 (v, w)
  // work_grid: 膨胀后的栅格（NavCore::workGrid()），用于弦碰撞检测
  // path 为空或速度 <= 0.01 时返回零速
  TwistCmd compute(const Path& path, double speed, const GridMap& work_grid) const {
    TwistCmd cmd;
    if (path.size() < 2 || speed <= 0.01) return cmd;

    // 1. 找第一个弧长 >= lookahead_ 的点索引 i0
    size_t i0 = path.size() - 1;
    for (size_t i = 0; i < path.size(); ++i) {
      if (path[i].s >= lookahead_) {
        i0 = i;
        break;
      }
    }

    // 2. 从 i0 往近处收缩，取第一个「车→该点」弦无碰撞的点。
    //    纯跟踪实际走弦而非路径，拐角处弦会切进内侧吃掉膨胀余量。
    size_t pick = 1;
    for (size_t i = i0; i >= 1; --i) {
      if (grid::segmentFree(work_grid, 0.0, 0.0, path[i].p.x, path[i].p.y)) {
        pick = i;
        break;
      }
      if (i == 1) break;  // 防 size_t 下溢
    }

    const double tx = path[pick].p.x;
    const double ty = path[pick].p.y;

    // 3. 纯跟踪公式：omega = 2 * v * sin(alpha) / L
    double L = std::sqrt(tx * tx + ty * ty);
    if (L < 0.05) L = 0.05;  // 防除零

    double alpha = std::atan2(ty, tx);
    double omega = 2.0 * speed * std::sin(alpha) / L;

    // 限幅
    omega = std::max(-w_max_, std::min(w_max_, omega));

    cmd.v = speed;
    cmd.w = omega;
    return cmd;
  }

  void setLookahead(double d) { lookahead_ = d; }
  void setWMax(double w) { w_max_ = w; }
  double lookahead() const { return lookahead_; }

 private:
  double lookahead_ = 0.5;
  double w_max_ = 2.0;
};

}  // namespace unk
