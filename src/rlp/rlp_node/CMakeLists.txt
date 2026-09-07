cmake_minimum_required(VERSION 3.0.2)
project(rlp_node)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  message_generation
  rlp_planner
)

##########################
# Custom messages        #
##########################
add_message_files(
  FILES
  Boundary.msg
  LocalizationQuality.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs
                 tf2 tf2_ros tf2_geometry_msgs message_runtime
                 rlp_planner
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

##########################
# ROS node               #
##########################
add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/planner_node.cpp
)
add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

##########################
# Offline core test      #
##########################
add_executable(core_test test/core_test.cpp)
target_link_libraries(core_test ${catkin_LIBRARIES})
enable_testing()
add_test(NAME core_test COMMAND core_test)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
