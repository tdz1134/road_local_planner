#pragma once
// 速度规划：由路径与局部栅格反推本周期的推荐速度。
//
// 同时强制差速底盘的四条限制，取最小值：
//   1. 车辆能力     v <= v_max
//   2. 障碍制动包络 v^2/(2a) + v*T_reaction + margin <= d_obs
//   3. 横向加速度   v <= sqrt(a_lat_max / kappa)
//   4. 角速度上限   v <= w_max / kappa          （因 w = v * kappa）
//   5. 曲率变化率   v <= dk_max / (dkappa/ds)
// 只限曲率而不限后两条，差速底盘在高速段会侧滑失稳，故三条运动约束必须一起卡。
//
// 另外做可选的硬可行性判定：kappa_max > 0 时，路径曲率超过它就急停（该车根本走不出
// 这条路径），而不是单纯降速 —— 降速解决不了几何不可行。差速底盘默认关闭此项
// （kappa_max = 0），因为它能原地转向，曲率不构成运动学约束，硬卡会把栅格 A* 的
// 正常直角拐点全判死；阿克曼底盘才需要打开。
#include <limits>

#include "unk_nav/types.h"

namespace unk {
namespace speed {

struct Result {
  double v = 0.0;                  // 推荐速度 m/s
  double obstacle_dist = std::numeric_limits<double>::infinity();  // 路径上首个阻挡点弧长
  double brake_dist = 0.0;         // 当前速度下的制动包络距离
  double kappa_max = 0.0;          // 路径最大曲率
  double dk_ds_max = 0.0;          // 路径最大曲率变化率（对弧长）
  bool emergency_stop = true;
  const char* limit_by = "none";   // 起限制作用的分项，调试用
};

// 当前速度下的制动包络距离
double brakeDistance(double v, const NavParams& p);

// 由可用停车距离反解最大安全速度（brakeDistance 的反函数，取正根）
double speedForDistance(double d, const NavParams& p);

// 综合限速。grid 必须是与生成 path 时同一份（已膨胀）栅格，否则障碍距离不一致。
Result limit(const Path& path, const GridMap& grid, double current_speed,
             const NavParams& p);

}  // namespace speed
}  // namespace unk
