cmake_minimum_required(VERSION 3.0.2)
project(unk_nav_sim)

find_package(catkin REQUIRED)

catkin_package()

# 安装 worlds/launch/config 到 devel 和 install 空间
install(DIRECTORY worlds launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
