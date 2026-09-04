cmake_minimum_required(VERSION 3.0.2)
project(unk_nav)

# ── 零第三方依赖构建 ──────────────────────────────────────────────────
# 本包只依赖 C++14 标准库。交付到 MDC 的就是 include/ + src/ 这棵树，
# 用任意工具链执行下面三步即可独立构建（不需要 catkin、不需要 ROS）：
#   mkdir -p /tmp/unk_build && cd /tmp/unk_build
#   cmake /path/to/src/unk_nav && make
#   ./unk_nav_test
#
# 同时保留 catkin_package() 调用（仅在 catkin 环境下生效），使本包能被
# 工作区根目录的 catkin_make 一并编入，方便与 rlp_* 共存开发。
# catkin 只出现在构建脚本里，源码中不含任何 ROS 引用。

add_compile_options(-std=c++14 -Wall -Wextra)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(include)

##########################
# 核心算法库（MDC 交付物）
##########################
add_library(${PROJECT_NAME}_core
  src/geom_util.cpp
  src/grid_util.cpp
  src/astar.cpp
  src/path_smooth.cpp
  src/subgoal.cpp
  src/speed_planner.cpp
  src/behavior_fsm.cpp
  src/nav_core.cpp
)

##########################
# 离线单测（裸 main + assert，不依赖 gtest / ROS）
##########################
add_executable(${PROJECT_NAME}_test test/core_test.cpp)
target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME}_core)

##########################
# 离线 demo（不依赖 ROS / Gazebo，自带世界渲染与纯跟踪替身）
##########################
add_executable(${PROJECT_NAME}_demo demo/straight_line_demo.cpp)
target_link_libraries(${PROJECT_NAME}_demo ${PROJECT_NAME}_core)

enable_testing()
add_test(NAME ${PROJECT_NAME}_test COMMAND ${PROJECT_NAME}_test)

##########################
# catkin 集成（可选，独立构建时自动跳过）
##########################
find_package(catkin QUIET)
if(catkin_FOUND)
  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}_core
  )
  install(TARGETS ${PROJECT_NAME}_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
endif()
