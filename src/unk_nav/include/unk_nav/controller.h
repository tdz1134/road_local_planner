#pragma once
// 纯跟踪控制器：从车体系路径中找前视点，算出 (v, w)。
//
// 输入：unk::Path（车体系，x 前 y 左）+ 推荐速度 + 膨胀后的工作栅格
//       + 车在该栅格系下的当前位姿（控制环快于规划环时必须传）
// 输出：TwistCmd{v, w}（线速度 m/s，角速度 rad/s）
//
// 关键：纯跟踪实际走的是「车 → 前视点」的弦，而非路径本身。拐角处这条弦
// 会切进内侧，速度越高、前视越大切得越深，足以吃掉规划留出的膨胀安全边距
//（把车送进障碍层困死）。因此从前视点往近处收缩，取第一个「车→该点」弦
// 无碰撞的点，保证实际走的弦不穿障碍。work_grid 由 NavCore::workGrid() 提供。
//
// 关于 car 入参：path 与 work_grid 都在「规划那一刻的车体系」里。控制环提到 50Hz
// 而规划仍是 10Hz 时，车已在这个系里走开了一段（1.5m/s × 100ms = 15cm）。若仍假定
// 车在原点选前视点、在原点做弦检查，每个 tick 都会算出与上个 tick 完全相同的
// (v,w) —— 高频退化成零阶保持的重复发送，对被控对象毫无改善。因此 compute() 先把车
// 投影到 path 上得到当前弧长，再向前取 lookahead。默认 car=Pose2D() 即「车就在规划
// 原点」，与规划/控制同频时的老行为逐字节一致。
//
// 纯 C++14，零 ROS 依赖。unk_nav_sim 的 nav_node 负责把 TwistCmd 转成
// geometry_msgs::Twist 发布到 /cmd_vel。
//
// 替换方式：改 compute() 实现，或新建一个类保持相同签名。

#include <cmath>
#include <algorithm>
#include <limits>
#include "unk_nav/types.h"
#include "unk_nav/grid_util.h"

namespace unk {

// 控制器输出（与 ROS 解耦的速度指令）
struct TwistCmd {
  double v = 0.0;  // 线速度 m/s
  double w = 0.0;  // 角速度 rad/s
};

namespace detail {
inline double clampd(double x, double lo, double hi) {
  return std::max(lo, std::min(hi, x));
}
}  // namespace detail

// 纯跟踪控制器（带弦碰撞检测收缩）
class PurePursuitController {
 public:
  // lookahead: 前视距离 m
  // w_max:     角速度限幅 rad/s
  PurePursuitController(double lookahead, double w_max)
      : lookahead_(lookahead), w_max_(w_max) {}

  // 从路径 + 推荐速度 + 工作栅格计算 (v, w)。
  // path / work_grid: 同在「规划时刻的车体系」（work_grid 即 NavCore::workGrid()）
  // car: 车在**同一栅格系**下的当前位姿。默认零位姿 = 控制与规划同频的老行为。
  TwistCmd compute(const Path& path, double speed, const GridMap& work_grid,
                   const Pose2D& car = Pose2D()) const {
    TwistCmd cmd;
    if (path.size() < 2 || speed <= 0.01) return cmd;

    // 1. 先把车投影到 path 上得到当前弧长，再沿路径向前取 lookahead。
    //    直接用「s >= lookahead」会在车已前移时又选回同一个近点。
    const double s_target = arcLengthAt(path, car) + lookahead_;
    size_t i0 = path.size() - 1;
    for (size_t i = 0; i < path.size(); ++i) {
      if (path[i].s >= s_target) {
        i0 = i;
        break;
      }
    }

    // 2. 从 i0 往近处收缩，取第一个「车→该点」弦无碰撞的点。
    //    纯跟踪实际走弦而非路径，拐角处弦会切进内侧吃掉膨胀余量。
    size_t pick = i0;
    for (size_t i = i0; i >= 1; --i) {
      if (grid::segmentFree(work_grid, car.x, car.y, path[i].p.x, path[i].p.y)) {
        pick = i;
        break;
      }
      if (i == 1) {  // 全被拦：退到最近的段末，与旧版兜底一致
        pick = 1;
        break;
      }
    }

    // 3. 目标点转到车系（前 x / 左 y）后再套纯跟踪公式。
    const double ex = path[pick].p.x - car.x;
    const double ey = path[pick].p.y - car.y;
    const double cc = std::cos(car.yaw), ss = std::sin(car.yaw);
    const double lx = cc * ex + ss * ey;
    const double ly = -ss * ex + cc * ey;

    double L = std::sqrt(lx * lx + ly * ly);
    if (L < 0.05) L = 0.05;  // 防除零

    const double alpha = std::atan2(ly, lx);
    // 限幅
    cmd.v = speed;
    cmd.w = detail::clampd(2.0 * speed * std::sin(alpha) / L, -w_max_, w_max_);
    return cmd;
  }

  void setLookahead(double d) { lookahead_ = d; }
  void setWMax(double w) { w_max_ = w; }
  double lookahead() const { return lookahead_; }

 private:
  // 车在 path 上的投影弧长：逐段取最近投影点。path 点数不多（前瞻 7m @0.05m ≈ 140 点），
  // 全扫一遍是微秒级，不必做增量搜索。
  static double arcLengthAt(const Path& path, const Pose2D& car) {
    double best_d2 = std::numeric_limits<double>::infinity();
    double best_s = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
      const double ax = path[i - 1].p.x - car.x, ay = path[i - 1].p.y - car.y;
      const double dx = path[i].p.x - path[i - 1].p.x;
      const double dy = path[i].p.y - path[i - 1].p.y;
      const double seg2 = dx * dx + dy * dy;
      const double t = seg2 > 1e-12
                           ? detail::clampd(-(ax * dx + ay * dy) / seg2, 0.0, 1.0)
                           : 0.0;
      const double px = ax + t * dx, py = ay + t * dy;
      const double d2 = px * px + py * py;
      if (d2 < best_d2) {
        best_d2 = d2;
        best_s = path[i - 1].s + t * (path[i].s - path[i - 1].s);
      }
    }
    return best_s;
  }

  double lookahead_ = 0.5;
  double w_max_ = 2.0;
};

// 指令斜率限幅：把阶跃指令整形成不超过底盘加速度的斜坡。
// 不限幅则每条指令都让执行器饱和（底盘 6 rad/s² 跑 0→0.8 要 133ms，比一个旧控制
// 周期还长），响应永远追不上命令。带上一帧指令状态，故独立成类而非塞进无状态的
// compute()。⚠ 急停必须绕过它：由调用方直接发零并 reset()，否则「能立刻停住」
// 这件事被斜坡拖慢。
class TwistSlewLimiter {
 public:
  TwistSlewLimiter(double a_max, double w_dot_max)
      : a_max_(a_max), w_dot_max_(w_dot_max) {}

  // dt: 距上次发指令的时间 s；dt <= 0 时原样透传（未启用 / 时钟异常）。
  TwistCmd limit(const TwistCmd& cmd, double dt) {
    TwistCmd out = cmd;
    if (dt > 0.0) {
      out.v = detail::clampd(cmd.v, prev_.v - a_max_ * dt, prev_.v + a_max_ * dt);
      out.w = detail::clampd(cmd.w, prev_.w - w_dot_max_ * dt, prev_.w + w_dot_max_ * dt);
    }
    prev_ = out;  // 记的是**实际发出**的值，否则请求值会偷偷跑到限幅前面
    return out;
  }

  void reset() { prev_ = TwistCmd(); }
  TwistCmd previous() const { return prev_; }

 private:
  double a_max_ = 3.0;       // 线速度指令斜率上限 m/s²
  double w_dot_max_ = 6.0;   // 角速度指令斜率上限 rad/s²
  TwistCmd prev_;
};

}  // namespace unk
