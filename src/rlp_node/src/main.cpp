#include <ros/ros.h>

#include "rlp_node/planner_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "road_local_planner");
  ros::NodeHandle pnh("~");
  rlp::PlannerNode node(pnh);
  ros::spin();
  return 0;
}
