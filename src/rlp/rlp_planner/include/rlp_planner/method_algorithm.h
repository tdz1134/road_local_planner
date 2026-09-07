#pragma once
// 算法层：规划方法内部的候选生成算法接口与带算法管理的方法基类。
//
// 分层关系：
//   PlannerRouter  →  PlannerBase 方法（follow/search/free，由道路+定位情况路由）
//                        └── MethodPlannerBase：内部注册多个 MethodAlgorithm，
//                            运行时按参数（PlannerParams::xxx_alg）按名字选择一个生效。
//
// 扩展方式：新增算法 = 继承 MethodAlgorithm 实现 name()/candidates()，
// 并在对应方法的构造函数中 addAlgorithm() 注册，参数配置该算法名字即可启用。
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rlp_planner/planner_base.h"

namespace rlp {
namespace planner {

// 单个候选生成算法接口：只负责"给上下文，产候选路径"。
// 不同算法之间互相独立，便于单独实现/替换/对比实验。
//
// 两种工作模式：
//   采样式：实现 candidates() 返回路径族，由 CostEvaluator 统一评价选最优。
//   搜索式：实现 directPlan() 直接返回最终路径（如 A*/RRT），跳过 CostEvaluator。
//   默认 directPlan() 返回空路径 → 走采样式流程。
class MethodAlgorithm {
 public:
  virtual ~MethodAlgorithm() = default;
  // 算法唯一名字（用于参数选择与状态输出，如 "offset"/"hybrid"/"fan"/"astar"）
  virtual const char* name() const = 0;
  // 采样式：不适用时返回空（由上层停车保护），不要抛异常
  virtual std::vector<Path> candidates(const PlanningContext& ctx) const = 0;
  // 搜索式：直接返回最终路径。返回空路径 = 不支持直接规划或搜索失败，
  // 上层回退到 candidates() 采样式流程。默认不支持。
  virtual Path directPlan(const PlanningContext& ctx) const {
    (void)ctx;
    return {};
  }
};

// 带算法管理的规划方法基类：子类只需声明 mode()/name()、
// 在构造函数中注册算法、实现 algorithmName() 指明从哪个参数字段取算法名。
class MethodPlannerBase : public PlannerBase {
 public:
  // final：子类不得绕过算法选择机制自行实现 candidates
  std::vector<Path> candidates(const PlanningContext& ctx) const final {
    const MethodAlgorithm* alg = selected();
    return alg ? alg->candidates(ctx) : std::vector<Path>();
  }

  // 转发给当前选中算法的 directPlan（搜索式）
  Path directPlan(const PlanningContext& ctx) const final {
    const MethodAlgorithm* alg = selected();
    return alg ? alg->directPlan(ctx) : Path();
  }

  const char* algorithm() const final {
    const MethodAlgorithm* alg = selected();
    return alg ? alg->name() : "none";
  }

 protected:
  // 子类返回当前应选择的算法名（通常直接返回 PlannerParams 中对应字段）
  virtual const std::string& algorithmName() const = 0;

  // 注册一个算法（构造时调用）；第一个注册的为默认算法
  void addAlgorithm(std::unique_ptr<MethodAlgorithm> alg) {
    algs_.push_back(std::move(alg));
  }

 private:
  // 按名字选择算法；无匹配（如参数写错）时回退到第一个注册的默认算法
  const MethodAlgorithm* selected() const {
    if (algs_.empty()) return nullptr;
    const std::string& want = algorithmName();
    for (const auto& a : algs_) {
      if (want == a->name()) return a.get();
    }
    return algs_.front().get();
  }

  std::vector<std::unique_ptr<MethodAlgorithm>> algs_;
};

}  // namespace planner
}  // namespace rlp
