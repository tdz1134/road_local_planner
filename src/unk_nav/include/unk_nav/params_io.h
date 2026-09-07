#pragma once
// YAML 配置加载：unk_nav 与「参数来源」之间唯一的桥梁。
//
// 本文件是全库唯一使用第三方库（yaml-cpp）的模块。核心算法
// （astar / subgoal / speed_planner / behavior_fsm ...）仍然只依赖标准库；
// 集成方（nav_node、demo、MDC 上的正式集成）有两种等价用法：
//   a) 调 loadNavParams() 从 yaml 填充 NavParams（推荐，仿真/实车共用一份配置）；
//   b) 不用本模块，直接逐字段给 NavParams 赋值（若目标平台不想引 yaml-cpp，
//      把 params_io.cpp 从构建里剔除即可，核心库不受影响）。
//
// yaml 格式：顶层扁平 `key: value  # 注释`，key 与 NavParams 字段名一一对应。
#include <string>

#include "unk_nav/types.h"

namespace unk {

// 从扁平 YAML 文件加载 NavParams。
//   - 只覆盖 yaml 里出现的字段，未写的保持 *out 当前值（默认值即安全回退）；
//   - 严格校验：文件不存在 / 顶层不是 map / 未知 key / 类型不可转换 → 返回 false，
//     并把全部问题拼成人类可读的原因写入 err。
//     拼错的 key 静默失效比立刻失败难查十倍，因此宁可拒绝启动。
bool loadNavParams(const std::string& yaml_path, NavParams* out,
                   std::string* err = nullptr);

}  // namespace unk
