#pragma once
// ROS 节点层：只负责 消息转换 / tf / 定时调度 / 参数加载，
// 不含任何规划算法逻辑（算法全部在 rlp_planner 与 rlp_road 中）。
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <rlp_node/Boundary.h>
#include <rlp_node/LocalizationQuality.h>

#include "rlp_planner/planner_core.h"

namespace rlp {

class PlannerNode {
 public:
  explicit PlannerNode(ros::NodeHandle& pnh);

 private:
  void loadParams();

  // ---- 输入回调：仅缓存最新一帧 ----
  void mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void leftCb(const rlp_node::Boundary::ConstPtr& msg);
  void rightCb(const rlp_node::Boundary::ConstPtr& msg);
  void qualityCb(const rlp_node::LocalizationQuality::ConstPtr& msg);
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
  void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg);

  // ---- 主循环：定时调用 PlannerCore::plan 并发布结果 ----
  void onTimer(const ros::TimerEvent& event);

  ros::NodeHandle pnh_;                              // 私有节点句柄，用于加载参数、创建话题
  PlannerParams params_;                             // 从 params.yaml 加载的全部可调参数
  std::string vehicle_frame_;                        // 车体坐标系 frame_id（默认 "base_link"）
  std::unique_ptr<planner::PlannerCore> core_;       // 规划核心，持有 RoadModel + PlannerRouter

  // ---- ROS 通信 ----
  ros::Subscriber sub_map_, sub_left_, sub_right_, sub_quality_, sub_odom_, sub_goal_;
  ros::Publisher pub_path_, pub_cmd_, pub_status_;   // 输出：局部路径 / 速度指令 / 状态调试
  ros::Timer timer_;                                 // 按 plan_freq 定时触发 onTimer

  // ---- tf ----
  tf2_ros::Buffer tf_buffer_;                        // tf 缓存，用于全局→车体系坐标变换
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;  // tf 监听器，持续接收变换

  // ---- 输入缓存（受 mtx_ 保护）----
  std::mutex mtx_;
  GridMap map_;                                      // 最新局部占据栅格（车体系）
  bool have_map_ = false;                            // 是否收到过至少一帧栅格
  road::BoundarySet boundary_cache_;                 // 最新左右边界点列
  double quality_ = 0.0;                             // 最新定位质量 [0,1]
  geometry_msgs::PoseStamped goal_;                  // 最新全局终点（全局系，onTimer 中经 tf 转车体系）
  bool goal_received_ = false;                       // 是否收到过至少一帧终点
  double speed_ = 0.0;                               // 最新车速（m/s，取自 odom）
};

}  // namespace rlp
