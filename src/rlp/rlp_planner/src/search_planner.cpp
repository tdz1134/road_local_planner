#include "rlp_planner/search_planner.h"

#include "rlp_planner/algorithms/search_astar.h"
#include "rlp_planner/algorithms/search_hybrid.h"
#include "rlp_planner/algorithms/search_rrt.h"

namespace rlp {
namespace planner {

SearchPlanner::SearchPlanner(const PlannerParams& p) : p_(p) {
  // search 方法可用的候选生成算法；第一个注册的为默认算法（参数写错时回退到它）。
  // 新增算法：实现 MethodAlgorithm 后在此 addAlgorithm 注册，参数 search_alg 配置其名字。
  addAlgorithm(std::make_unique<SearchHybridAlg>(p));
  addAlgorithm(std::make_unique<SearchAStarAlg>(p));
  addAlgorithm(std::make_unique<SearchRRTAlg>(p));
}

}  // namespace planner
}  // namespace rlp
