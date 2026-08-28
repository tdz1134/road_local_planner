cmake_minimum_required(VERSION 3.0.2)
project(rlp_common)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rlp_common
)

include_directories(include)

add_library(${PROJECT_NAME}
  src/geometry_util.cpp
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
