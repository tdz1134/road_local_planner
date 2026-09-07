// 离线 demo：不需要 ROS，不需要 Gazebo，直接 ./unk_nav_demo 就能跑。
//
// 做的事：合成一个 2D 世界（默认为空 → 车跑直线），给一个远在局部窗口之外的终点，
//         闭环调用 NavCore，把车开到终点。
//
// 边界说明：这里的「世界渲染 → 局部栅格」和「纯跟踪 → 速度指令」都是**替身**，
// 实车上分别由感知模块和控制模块提供，不属于本模块交付物。NavCore 只吃
// 局部栅格 + 位姿 + 车速 + 终点，只吐路径 + 推荐速度。
//
// 用法：
//   ./unk_nav_demo                          # 空世界跑直线（雷达 12m → 窗口 20.4m）
//   ./unk_nav_demo --goal 30 5              # 指定终点
//   ./unk_nav_demo --obs 5 -3 6 3           # 加一个矩形障碍（可多次），看绕行
//   ./unk_nav_demo --range 30               # 换实车尺度雷达，窗口自动变 51m（约 100 万格）
//   ./unk_nav_demo --config ../config/nav_params.yaml   # 用与仿真/实车同一份 YAML 配置
//                                     # （不传 --config 则用代码默认值；--range 会覆盖配置里的 sensor_range）
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "unk_nav/geom_util.h"
#include "unk_nav/grid_util.h"
#include "unk_nav/nav_core.h"
#include "unk_nav/params_io.h"
#include "unk_nav/types.h"

namespace {

// ---- 替身①：世界 → 局部栅格（实车由感知模块提供）----

struct Rect {
  double x0, y0, x1, y1;
};

struct World {
  std::vector<Rect> obstacles;
  bool blocked(double gx, double gy) const {
    for (const auto& r : obstacles) {
      if (gx >= r.x0 && gx <= r.x1 && gy >= r.y0 && gy <= r.y1) return true;
    }
    return false;
  }
};

// 渲染以车为中心的 window×window 局部栅格（车体系）。
// 超出 sensor_range 的格子标为 unknown —— 这正是「环境未知」的来源。
unk::GridMap renderLocalGrid(const World& w, const unk::Pose2D& veh, double window,
                             double res, double sensor_range) {
  unk::GridMap g;
  g.resolution = res;
  g.width = g.height = static_cast<int>(std::round(window / res));
  g.origin_x = g.origin_y = -0.5 * g.width * res;
  g.data.resize(static_cast<size_t>(g.width) * static_cast<size_t>(g.height), unk::kFree);

  const double c = std::cos(veh.yaw), s = std::sin(veh.yaw);
  const double r2 = sensor_range * sensor_range;
  for (int gy = 0; gy < g.height; ++gy) {
    for (int gx = 0; gx < g.width; ++gx) {
      const double bx = g.origin_x + (gx + 0.5) * res;
      const double by = g.origin_y + (gy + 0.5) * res;
      int8_t v = unk::kFree;
      if (bx * bx + by * by > r2) {
        v = unk::kUnknown;  // 雷达够不到 → 未知，乐观放行
      } else {
        const double wx = c * bx - s * by + veh.x;  // 车体系 → 全局系
        const double wy = s * bx + c * by + veh.y;
        if (w.blocked(wx, wy)) v = unk::kOccupied;
      }
      g.data[static_cast<size_t>(gy) * static_cast<size_t>(g.width) +
             static_cast<size_t>(gx)] = v;
    }
  }
  return g;
}

// ---- 替身②：纯跟踪控制器（实车由控制模块提供）----

struct Twist {
  double v = 0.0, w = 0.0;
};

Twist purePursuit(const unk::Path& path, double v_cmd, const unk::NavParams& p,
                  const unk::GridMap& work_grid) {
  if (path.size() < 2 || v_cmd <= 1e-3) return Twist{0.0, 0.0};
  // 预瞄距离随速度变化；低速下过大会导致转向迟钝
  const double ld = std::min(1.5, std::max(0.4, 0.4 + 2.5 * v_cmd));

  // 第一个弧长达到 ld 的点
  size_t i0 = path.size() - 1;
  for (size_t i = 0; i < path.size(); ++i) {
    if (path[i].s >= ld) {
      i0 = i;
      break;
    }
  }
  // 关键：纯跟踪走的是「车 → 预瞄点」的弦，会切进拐角内侧。速度越高 ld 越大、
  // 切得越深，足以吃掉规划留出的膨胀安全边距（实测会把车送进障碍层里困死）。
  // 因此从 i0 往近处收缩，取第一个弦无碰撞的点。
  size_t pick = 1;
  for (size_t i = i0; i >= 1; --i) {
    if (unk::grid::segmentFree(work_grid, 0.0, 0.0, path[i].p.x, path[i].p.y)) {
      pick = i;
      break;
    }
    if (i == 1) break;  // 防 size_t 下溢
  }
  const unk::Point2D tgt = path[pick].p;

  const double alpha = std::atan2(tgt.y, tgt.x);
  const double l = std::max(std::hypot(tgt.x, tgt.y), 0.1);
  double v = v_cmd;
  double w = v * 2.0 * std::sin(alpha) / l;
  // 大角度先原地转：差速底盘转向能力充足，边走边转反而画大弧
  if (std::fabs(alpha) > 0.7) {
    v = 0.03;
    w = (alpha > 0.0 ? 1.0 : -1.0) * p.w_max * 0.6;
  }
  v = std::min(v, p.v_max);
  w = std::max(-p.w_max, std::min(p.w_max, w));
  return Twist{v, w};
}

// ---- 结果输出 ----

void printAsciiMap(const World& w, const std::vector<unk::Pose2D>& traj,
                   const unk::Point2D& goal) {
  double xmin = -2.0, xmax = 2.0, ymin = -2.0, ymax = 2.0;
  for (const auto& p : traj) {
    xmin = std::min(xmin, p.x); xmax = std::max(xmax, p.x);
    ymin = std::min(ymin, p.y); ymax = std::max(ymax, p.y);
  }
  for (const auto& r : w.obstacles) {
    xmin = std::min(xmin, r.x0); xmax = std::max(xmax, r.x1);
    ymin = std::min(ymin, r.y0); ymax = std::max(ymax, r.y1);
  }
  xmin = std::min(xmin, goal.x); xmax = std::max(xmax, goal.x);
  ymin = std::min(ymin, goal.y); ymax = std::max(ymax, goal.y);
  xmin -= 1.5; xmax += 1.5; ymin -= 1.5; ymax += 1.5;

  const double cell = 0.5;  // 每个字符 0.5m
  const int nx = static_cast<int>(std::ceil((xmax - xmin) / cell));
  const int ny = static_cast<int>(std::ceil((ymax - ymin) / cell));
  std::vector<std::string> rows(static_cast<size_t>(ny), std::string(static_cast<size_t>(nx), ' '));

  auto put = [&](double wx, double wy, char c) {
    const int ix = static_cast<int>((wx - xmin) / cell);
    const int iy = static_cast<int>((wy - ymin) / cell);
    if (ix < 0 || iy < 0 || ix >= nx || iy >= ny) return;
    // 终端行序自上而下，世界 y 轴向上 → 翻转
    rows[static_cast<size_t>(ny - 1 - iy)][static_cast<size_t>(ix)] = c;
  };

  for (double x = xmin; x <= xmax; x += cell * 0.5) {
    for (double y = ymin; y <= ymax; y += cell * 0.5) {
      if (w.blocked(x, y)) put(x, y, '#');
    }
  }
  for (const auto& p : traj) put(p.x, p.y, '*');
  put(goal.x, goal.y, 'G');
  if (!traj.empty()) {
    put(traj.front().x, traj.front().y, 'S');
    put(traj.back().x, traj.back().y, '@');
  }

  std::printf("\n---- 轨迹（S=起点  @=停车点  G=终点  #=障碍  *=轨迹，每字符 0.5m）----\n");
  for (const auto& r : rows) std::printf("%s\n", r.c_str());
}

// ---- 耗时统计 ----

struct Timing {
  std::vector<double> plan_ms;    // NavCore::plan 单次耗时
  std::vector<double> render_ms;  // 世界渲染耗时（替身，不属于本模块）

  static double percentile(std::vector<double> v, double q) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const size_t i = static_cast<size_t>(q * static_cast<double>(v.size() - 1));
    return v[i];
  }
  static double mean(const std::vector<double>& v) {
    if (v.empty()) return 0.0;
    double s = 0.0;
    for (const double x : v) s += x;
    return s / static_cast<double>(v.size());
  }
  static double worst(const std::vector<double>& v) {
    return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end());
  }

  void report() const {
    const double m = mean(plan_ms);
    std::printf("\n---- 规划耗时（NavCore::plan，不含感知与控制替身）----\n");
    std::printf("样本数  : %zu\n", plan_ms.size());
    std::printf("平均    : %.3f ms   → 上限 %.1f Hz\n", m, m > 1e-9 ? 1000.0 / m : 0.0);
    std::printf("p95     : %.3f ms   → %.1f Hz\n", percentile(plan_ms, 0.95),
                1000.0 / std::max(1e-9, percentile(plan_ms, 0.95)));
    std::printf("p99     : %.3f ms   → %.1f Hz\n", percentile(plan_ms, 0.99),
                1000.0 / std::max(1e-9, percentile(plan_ms, 0.99)));
    std::printf("最差    : %.3f ms   → %.1f Hz\n", worst(plan_ms),
                1000.0 / std::max(1e-9, worst(plan_ms)));
    std::printf("10Hz 预算 100ms：平均占 %.2f%%，最差占 %.2f%%  → %s\n", m * 10.0,
                worst(plan_ms) * 10.0, worst(plan_ms) < 100.0 ? "满足" : "不满足");
    std::printf("（参考：世界渲染替身平均 %.3f ms，实车由感知模块提供，不计入本模块）\n",
                mean(render_ms));
  }
};

}  // namespace

int main(int argc, char** argv) {
  // ---- 参数解析 ----
  unk::Point2D goal{30.0, 5.0};
  World world;
  double sensor_range = 12.0;
  bool range_given = false;
  std::string config;  // 空 = 用代码默认参数；非空 = 从 YAML 加载（与仿真同一份）
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--goal" && i + 2 < argc) {
      goal.x = std::atof(argv[++i]);
      goal.y = std::atof(argv[++i]);
    } else if (a == "--range" && i + 1 < argc) {
      sensor_range = std::atof(argv[++i]);  // 用于实测实车尺度（30m 雷达 → 51m 窗口）
      range_given = true;
    } else if (a == "--config" && i + 1 < argc) {
      config = argv[++i];
    } else if (a == "--obs" && i + 4 < argc) {
      Rect r;
      r.x0 = std::atof(argv[++i]);
      r.y0 = std::atof(argv[++i]);
      r.x1 = std::atof(argv[++i]);
      r.y1 = std::atof(argv[++i]);
      world.obstacles.push_back(r);
    }
  }

  // ---- 规划器参数：默认代码值，可被 --config 的 YAML 覆盖 ----
  unk::NavParams p;
  if (!config.empty()) {
    std::string err;
    if (!unk::loadNavParams(config, &p, &err)) {
      std::fprintf(stderr, "[demo] 配置加载失败 [%s]：%s\n", config.c_str(), err.c_str());
      return 1;
    }
    std::printf("[demo] 已加载配置：%s\n", config.c_str());
  }
  if (range_given) p.sensor_range = sensor_range;  // 命令行显式指定优先级最高
  const double kWindow = 1.7 * p.sensor_range;  // 无量纲化：窗口 = 1.7 × 雷达量程
  const double kRes = 0.05;

  unk::NavCore nav(p);
  unk::Pose2D veh{0.0, 0.0, std::atan2(goal.y, goal.x)};  // 起点大致朝终点
  double v = 0.0;

  const double dt = 1.0 / p.plan_freq;
  const double t_max = 300.0;
  std::vector<unk::Pose2D> traj;
  traj.push_back(veh);

  FILE* csv = std::fopen("trajectory.csv", "w");
  if (csv != nullptr) {
    std::fprintf(csv, "t,x,y,yaw,v_cmd,w_cmd,state,goal_dist,subgoal_x,subgoal_y,"
                      "path_len,v_rec,reason\n");
  }

  std::printf("unk_nav demo：终点 (%.1f, %.1f)，直线距离 %.1f m，障碍 %zu 个\n", goal.x,
              goal.y, std::hypot(goal.x, goal.y), world.obstacles.size());
  std::printf("雷达量程 %.1fm  窗口 %.1fm  栅格 %d×%d = %.2f 万格  分辨率 %.2fm\n",
              p.sensor_range, kWindow, static_cast<int>(kWindow / kRes),
              static_cast<int>(kWindow / kRes), kWindow * kWindow / (kRes * kRes) / 1e4, kRes);
  std::printf("前瞻 %.1fm  v_max %.2f m/s\n\n", p.lookahead(), p.v_max);

  int cycles = 0;
  double path_len_sum = 0.0;
  unk::NavState final_state = unk::NavState::IDLE;
  std::string final_reason;
  Timing timing;
  timing.plan_ms.reserve(4096);
  timing.render_ms.reserve(4096);
  double kmax_seen = 0.0;
  double v_min_moving = 1e9;

  for (double t = 0.0; t < t_max; t += dt, ++cycles) {
    // ---- 组装输入 ----
    const auto tr0 = std::chrono::steady_clock::now();
    unk::NavInput in;
    in.now = t;
    in.local_grid = renderLocalGrid(world, veh, kWindow, kRes, p.sensor_range);
    in.vehicle_pose = veh;
    in.goal = goal;
    in.goal_valid = true;
    in.current_speed = v;
    const auto tr1 = std::chrono::steady_clock::now();

    // ---- 规划 ----
    const unk::NavResult r = nav.plan(in);
    const auto tp1 = std::chrono::steady_clock::now();
    timing.render_ms.push_back(
        std::chrono::duration<double, std::milli>(tr1 - tr0).count());
    timing.plan_ms.push_back(std::chrono::duration<double, std::milli>(tp1 - tr1).count());
    final_state = r.state;
    final_reason = r.reason;

    // ---- 控制（替身）----
    const Twist cmd = r.emergency_stop ? Twist{0.0, 0.0}
                                       : purePursuit(r.path, r.recommended_speed, p, nav.workGrid());

    if (csv != nullptr) {
      const double gd = std::hypot(goal.x - veh.x, goal.y - veh.y);
      std::fprintf(csv, "%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%s,%.3f,%.3f,%.3f,%.3f,%.3f,%s\n",
                   t, veh.x, veh.y, veh.yaw, cmd.v, cmd.w, unk::navStateName(r.state), gd,
                   r.subgoal.x, r.subgoal.y, r.path.empty() ? 0.0 : r.path.back().s,
                   r.recommended_speed, r.reason.c_str());
    }

    // 每 10 个周期打一行进度
    if (cycles % 10 == 0) {
      std::printf("t=%5.1f  pos=(%6.2f,%6.2f)  距终点=%6.2f  v=%.3f  路径点=%3zu  %s\n",
                  t, veh.x, veh.y, std::hypot(goal.x - veh.x, goal.y - veh.y),
                  r.recommended_speed, r.path.size(), r.reason.c_str());
    }
    if (!r.path.empty()) path_len_sum += r.path.back().s;

    // 路径质量统计：最大曲率 + 非零推荐速度的最小值（过弯降速程度）
    for (const auto& pp : r.path) kmax_seen = std::max(kmax_seen, std::fabs(pp.k));
    if (r.recommended_speed > 1e-6) {
      v_min_moving = std::min(v_min_moving, r.recommended_speed);
    }

    // ---- 差速运动学积分 ----
    veh.x += cmd.v * std::cos(veh.yaw) * dt;
    veh.y += cmd.v * std::sin(veh.yaw) * dt;
    veh.yaw = unk::geom::normalizeAngle(veh.yaw + cmd.w * dt);
    v = cmd.v;
    traj.push_back(veh);

    if (r.state == unk::NavState::ARRIVED || r.state == unk::NavState::ABORT) break;
  }

  // ---- 汇总 ----
  double mileage = 0.0;
  for (size_t i = 1; i < traj.size(); ++i) {
    mileage += std::hypot(traj[i].x - traj[i - 1].x, traj[i].y - traj[i - 1].y);
  }
  const double err = std::hypot(goal.x - traj.back().x, goal.y - traj.back().y);
  const bool ok = (final_state == unk::NavState::ARRIVED);

  std::printf("\n================ 结果 ================\n");
  std::printf("状态      : %s (%s)\n", unk::navStateName(final_state), final_reason.c_str());
  std::printf("周期数    : %d  (%.1f s)\n", cycles, cycles * dt);
  std::printf("行驶里程  : %.2f m   (直线距离 %.2f m，绕行比 %.2f)\n", mileage,
              std::hypot(goal.x, goal.y), mileage / std::max(1e-6, std::hypot(goal.x, goal.y)));
  std::printf("终点误差  : %.3f m   (容差 %.2f)\n", err, p.goal_tolerance);
  std::printf("平均路径长: %.2f m\n", path_len_sum / std::max(1, cycles));
  std::printf("最大曲率  : %.3f 1/m  (转弯半径 %.2f m)\n", kmax_seen,
              kmax_seen > 1e-9 ? 1.0 / kmax_seen : 1e9);
  std::printf("过弯最低速: %.3f m/s  (v_max %.2f，越低说明弯道被限速越狠)\n",
              v_min_moving > 1e8 ? 0.0 : v_min_moving, p.v_max);
  std::printf("轨迹已写入: trajectory.csv\n");
  std::printf("结论      : %s\n", ok ? "成功到达 PASS" : "未到达 FAIL");

  timing.report();
  printAsciiMap(world, traj, goal);
  if (csv != nullptr) std::fclose(csv);
  return ok ? 0 : 1;
}
