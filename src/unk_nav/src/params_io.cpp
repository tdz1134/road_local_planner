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
      {"plan_freq", kDouble, &p->plan_freq},
      // A* 搜索
      {"astar_max_iter", kInt, &p->astar_max_iter},
      {"unknown_cost", kDouble, &p->unknown_cost},
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
  return true;
}

}  // namespace unk
