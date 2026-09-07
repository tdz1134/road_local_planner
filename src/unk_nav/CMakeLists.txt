cmake_minimum_required(VERSION 3.0.2)
project(unk_nav)

# ── 构建说明 ────────────────────────────────────────────────────────
# 核心算法只依赖 C++14 标准库；唯一的第三方依赖是 yaml-cpp（仅 params_io.cpp
# 用于读配置文件，纯 C++ 库，MDC 交叉编译无障碍）。
# 交付到 MDC 的就是 include/ + src/ + config/ 这棵树，独立构建：
#   mkdir -p /tmp/unk_build && cd /tmp/unk_build
#   cmake /path/to/src/unk_nav && make
#   ./unk_nav_test
#
# 若目标平台不想引 yaml-cpp：剔除 params_io.cpp，自己填 NavParams 即可，
# 核心库不受影响（见 include/unk_nav/params_io.h 头注释）。
#
# 同时保留 catkin_package() 调用（仅在 catkin 环境下生效），使本包能被
# 工作区根目录的 catkin_make 一并编入，方便与 rlp_* 共存开发。
# catkin 只出现在构建脚本里，源码中不含任何 ROS 引用。

add_compile_options(-std=c++14 -Wall -Wextra)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 唯一的第三方依赖：yaml 配置加载（params_io.cpp）
find_package(yaml-cpp REQUIRED)

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
  src/road_follow.cpp
  src/speed_planner.cpp
  src/behavior_fsm.cpp
  src/nav_core.cpp
  src/params_io.cpp
)
# PUBLIC：集成方（unk_nav_sim 等）随核心库自动获得 yaml-cpp 链接，无需自己声明
target_link_libraries(${PROJECT_NAME}_core PUBLIC yaml-cpp)

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
    CATKIN_DEPENDS
    DEPENDS yaml-cpp
  )
  install(TARGETS ${PROJECT_NAME}_core
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
endif()
