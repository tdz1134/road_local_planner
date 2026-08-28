#include "rlp_planner/free_planner.h"

#include "rlp_planner/algorithms/free_fan.h"

namespace rlp {
namespace planner {

FreePlanner::FreePlanner(const PlannerParams& p) : p_(p) {
  // free 方法可用的候选生成算法；第一个注册的为默认算法（参数写错时回退到它）。
  // 新增算法：实现 MethodAlgorithm 后在此 addAlgorithm 注册，参数 free_alg 配置其名字。
  addAlgorithm(std::make_unique<FreeFanAlg>(p));
}

}  // namespace planner
}  // namespace rlp
