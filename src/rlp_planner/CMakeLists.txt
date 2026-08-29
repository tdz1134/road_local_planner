cmake_minimum_required(VERSION 3.0.2)
project(rlp_planner)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS rlp_road)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rlp_planner
  CATKIN_DEPENDS rlp_road
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/candidate_gen.cpp
  src/algorithms/follow_offset.cpp
  src/algorithms/search_hybrid.cpp
  src/algorithms/search_astar.cpp
  src/algorithms/search_rrt.cpp
  src/algorithms/free_fan.cpp
  src/follow_planner.cpp
  src/search_planner.cpp
  src/free_planner.cpp
  src/planner_router.cpp
  src/cost_evaluator.cpp
  src/safety_checker.cpp
  src/planner_core.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
