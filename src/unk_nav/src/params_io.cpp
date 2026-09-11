#include "unk_nav/params_io.h"

#include <yaml-cpp/yaml.h>

#include <sstream>
#include <vector>

namespace unk {
namespace {

// yaml key ↔ NavParams 成员的绑定表。
// 加新参数共三处：types.h 加字段、本表加一行、config yaml 加一行。
// 忘了加本表不会静默失效——yaml 里写了该 key 而表里没有 → 报 unknown key。
enum Kind { kDouble, kInt, kBool };

struct Binding {
  const char* name;
  Kind kind;
  void* ptr;
};

std::vector<Binding> makeBindings(NavParams* p) {
  return {
      // 车辆能力
      {"v_max", kDouble, &p->v_max},
      {"w_max", kDouble, &p->w_max},
      {"a_decel_max", kDouble, &p->a_decel_max},
      {"a_lat_max", kDouble, &p->a_lat_max},
      {"robot_radius", kDouble, &p->robot_radius},
      // 感知尺度
      {"sensor_range", kDouble, &p->sensor_range},
      // 栅格预处理
      {"inflation_radius", kDouble, &p->inflation_radius},
      {"footprint_clear_radius", kDouble, &p->footprint_clear_radius},
      {"inflate_unknown", kBool, &p->inflate_unknown},
      // 滚动时域规划
      {"lookahead_ratio", kDouble, &p->lookahead_ratio},
      {"subgoal_min_ratio", kDouble, &p->subgoal_min_ratio},
      {"path_spacing", kDouble, &p->path_spacing},
      {"curvature_baseline", kDouble, &p->curvature_baseline},
      {"goal_tolerance", kDouble, &p->goal_tolerance},
      {"goal_snap_dist", kDouble, &p->goal_snap_dist},
      // 子目标扇形选取（终点模式专用）
      {"subgoal_fan_half_deg", kDouble, &p->subgoal_fan_half_deg},
      {"subgoal_fan_step_deg", kDouble, &p->subgoal_fan_step_deg},
      {"subgoal_align_w", kDouble, &p->subgoal_align_w},
      {"subgoal_free_w", kDouble, &p->subgoal_free_w},
      {"subgoal_prev_w", kDouble, &p->subgoal_prev_w},
      {"subgoal_clearance", kDouble, &p->subgoal_clearance},
      {"goal_clear_radius", kDouble, &p->goal_clear_radius},
      {"plan_freq", kDouble, &p->plan_freq},
      // A* 搜索
      {"astar_max_iter", kInt, &p->astar_max_iter},
      {"unknown_cost", kDouble, &p->unknown_cost},
      {"astar_w", kDouble, &p->astar_w},
      {"obstacle_cost_k", kDouble, &p->obstacle_cost_k},
      {"obstacle_cost_sigma", kDouble, &p->obstacle_cost_sigma},
      {"consistency_k", kDouble, &p->consistency_k},
      {"consistency_sigma", kDouble, &p->consistency_sigma},
      // 速度规划
      {"safety_margin", kDouble, &p->safety_margin},
      {"t_reaction", kDouble, &p->t_reaction},
      {"kappa_max", kDouble, &p->kappa_max},
      {"dk_max", kDouble, &p->dk_max},
      // 路径平滑
      {"smooth_corner_speed", kDouble, &p->smooth_corner_speed},
      {"smooth_laplacian_iters", kInt, &p->smooth_laplacian_iters},
      {"smooth_laplacian_lambda", kDouble, &p->smooth_laplacian_lambda},
      {"smooth_shrink_retry", kInt, &p->smooth_shrink_retry},
      // 卡死 / 脱困
      {"stuck_time", kDouble, &p->stuck_time},
      {"stuck_dist", kDouble, &p->stuck_dist},
      {"recovery_max_retry", kInt, &p->recovery_max_retry},
      // 控制器
      {"pursuit_lookahead", kDouble, &p->pursuit_lookahead},
      // 沿路模式（无定位）
      {"follow_road", kBool, &p->follow_road},
      {"road_fan_half_deg", kDouble, &p->road_fan_half_deg},
      {"road_fan_step_deg", kDouble, &p->road_fan_step_deg},
      {"road_lookahead_ratio", kDouble, &p->road_lookahead_ratio},
      {"road_free_w", kDouble, &p->road_free_w},
      {"road_align_w", kDouble, &p->road_align_w},
  };
}

}  // namespace

bool loadNavParams(const std::string& yaml_path, NavParams* out,
                   std::string* err) {
  if (out == nullptr) return false;
  std::vector<std::string> problems;

  YAML::Node root;
  try {
    root = YAML::LoadFile(yaml_path);
  } catch (const YAML::Exception& e) {
    problems.push_back("cannot load '" + yaml_path + "': " + e.msg);
  }

  if (problems.empty() && !root.IsMap()) {
    problems.push_back("top level of '" + yaml_path + "' is not a key: value map");
  }

  if (problems.empty()) {
    const std::vector<Binding> bindings = makeBindings(out);
    // 遍历 yaml 实际出现的 key（而非绑定表）：多写/拼错的 key 才能被发现
    for (YAML::const_iterator it = root.begin(); it != root.end(); ++it) {
      std::string key;
      try {
        key = it->first.as<std::string>();
      } catch (const YAML::Exception&) {
        problems.push_back("non-string key is not allowed");
        continue;
      }
      const Binding* b = nullptr;
      for (const Binding& e : bindings) {
        if (key == e.name) {
          b = &e;
          break;
        }
      }
      if (b == nullptr) {
        problems.push_back("unknown key '" + key + "'");
        continue;
      }
      try {
        switch (b->kind) {
          case kDouble: *static_cast<double*>(b->ptr) = it->second.as<double>(); break;
          case kInt:    *static_cast<int*>(b->ptr) = it->second.as<int>(); break;
          case kBool:   *static_cast<bool*>(b->ptr) = it->second.as<bool>(); break;
        }
      } catch (const YAML::Exception&) {
        problems.push_back("key '" + key + "' value is not convertible to expected type");
      }
    }
  }

  if (!problems.empty()) {
    if (err != nullptr) {
      std::ostringstream oss;
      for (size_t i = 0; i < problems.size(); ++i) {
        if (i != 0) oss << "; ";
        oss << problems[i];
      }
      *err = oss.str();
    }
    return false;
  }

  // 关系校验：消除“设了但不生效的旋钮”
  if (out->subgoal_min_ratio > out->lookahead_ratio + 1e-9)
    problems.push_back("subgoal_min_ratio must be <= lookahead_ratio");
  if (out->footprint_clear_radius >= out->inflation_radius - 1e-9)
    problems.push_back("footprint_clear_radius must be < inflation_radius");
  if (out->goal_tolerance <= out->safety_margin + 1e-9)
    problems.push_back("goal_tolerance must be > safety_margin (otherwise stop_horizon deadlock)");
  if (out->subgoal_fan_half_deg > 0.0 && out->subgoal_fan_step_deg <= 0.0)
    problems.push_back("subgoal_fan_step_deg must be > 0 when subgoal_fan_half_deg > 0");
  if (out->subgoal_clearance < 0.0)
    problems.push_back("subgoal_clearance must be >= 0");
  if (out->goal_clear_radius < 0.0)
    problems.push_back("goal_clear_radius must be >= 0");
  if (out->subgoal_align_w <= out->subgoal_free_w + 1e-9)
    problems.push_back("subgoal_align_w must be > subgoal_free_w");

  if (!problems.empty()) {
    if (err != nullptr) {
      std::ostringstream oss;
      for (size_t i = 0; i < problems.size(); ++i) {
        if (i != 0) oss << "; ";
        oss << problems[i];
      }
      *err = oss.str();
    }
    return false;
  }
  return true;
}

}  // namespace unk
