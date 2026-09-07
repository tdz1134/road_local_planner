cmake_minimum_required(VERSION 3.0.2)
project(unk_nav_sim)

add_compile_options(-std=c++14 -Wall -Wextra)

# ── 依赖 ──────────────────────────────────────────────────────────────
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  gazebo_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  unk_nav            # 规划核心（纯 C++，catkin_package 导出 include + library）
)

find_package(PCL REQUIRED COMPONENTS common)
# unk_nav 的 params_io 需要 yaml-cpp（catkin 的 *_LIBRARIES 列表不传播
# 依赖的 PUBLIC 接口，下游必须自己显式链接）
find_package(yaml-cpp REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ── 三个独立节点（松耦合，通过标准 ROS 话题通信）──────────────────────

# 1) 定位节点：Gazebo 真值 → /odom + TF(odom→base_link)
add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node ${catkin_LIBRARIES})

# 2) 栅格节点：/velodyne_points → /local_grid (nav_msgs/OccupancyGrid)
add_executable(grid_node src/grid_node.cpp)
target_link_libraries(grid_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# 3) 导航节点：pose + grid + goal → unk::NavCore → /cmd_vel
add_executable(nav_node src/nav_node.cpp)
target_link_libraries(nav_node ${catkin_LIBRARIES} yaml-cpp)

# ── 安装 ──────────────────────────────────────────────────────────────
install(TARGETS localization_node grid_node nav_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY worlds launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
