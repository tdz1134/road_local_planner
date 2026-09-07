// ─────────────────────────────────────────────────────────────────────────────
// localization_node.cpp
//
// 职责：从 Gazebo 真值获取 Scout v2 位姿，发布 /odom 和 TF(odom→base_link)。
//
// 话题接口（松耦合，可独立替换）：
//   订阅：/gazebo/model_states (gazebo_msgs/ModelStates)
//   发布：/odom                (nav_msgs/Odometry)
//         /tf                  (odom → base_link)
//
// 替换方式：实车上把本节点换成真实定位系统（如 wheel_odom + EKF），
//           只要输出相同的 /odom 话题和 TF，下游无需改动。
// ─────────────────────────────────────────────────────────────────────────────

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <geometry_msgs/TransformStamped.h>
#include <string>
#include <cmath>

class LocalizationNode {
public:
  LocalizationNode() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ── 参数（可通过 launch/yaml 覆盖）──
    pnh.param<std::string>("model_name", model_name_, "scout/");
    pnh.param<std::string>("odom_frame", odom_frame_, "odom");
    pnh.param<std::string>("base_frame", base_frame_, "base_link");

    // ── 话题 ──
    sub_ = nh.subscribe("/gazebo/model_states", 10,
                        &LocalizationNode::modelStatesCb, this);
    pub_odom_ = nh.advertise<nav_msgs::Odometry>("/odom", 10);

    ROS_INFO("[localization] 等待 /gazebo/model_states，模型名='%s'",
             model_name_.c_str());
  }

private:
  void modelStatesCb(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    // 在模型列表中查找目标机器人
    int idx = -1;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == model_name_) {
        idx = static_cast<int>(i);
        break;
      }
    }
    if (idx < 0) return;  // 模型尚未生成，静默等待

    const auto& pose = msg->pose[idx];
    const auto& twist = msg->twist[idx];

    // ── 发布 Odometry ──
    // Gazebo model_states 的 twist 是世界系，但 REP-103 要求 Odometry.twist
    // 在 child_frame (body frame) 下。必须旋转 -yaw 转到车体系。
    double yaw = tf2::getYaw(pose.orientation);
    double cos_yaw = std::cos(-yaw);
    double sin_yaw = std::sin(-yaw);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose = pose;
    // 世界系 twist → 车体系 twist
    odom.twist.twist.linear.x = twist.linear.x * cos_yaw - twist.linear.y * sin_yaw;
    odom.twist.twist.linear.y = twist.linear.x * sin_yaw + twist.linear.y * cos_yaw;
    odom.twist.twist.linear.z = twist.linear.z;
    odom.twist.twist.angular.x = twist.angular.x;
    odom.twist.twist.angular.y = twist.angular.y;
    odom.twist.twist.angular.z = twist.angular.z;  // yaw rate 不受 2D 旋转影响
    pub_odom_.publish(odom);

    // ── 发布 TF (odom → base_link) ──
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = odom.header.stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = pose.position.x;
    tf.transform.translation.y = pose.position.y;
    tf.transform.translation.z = pose.position.z;
    tf.transform.rotation = pose.orientation;
    tf_broadcaster_.sendTransform(tf);
  }

  // ── 成员 ──
  std::string model_name_;
  std::string odom_frame_;
  std::string base_frame_;

  ros::Subscriber sub_;
  ros::Publisher pub_odom_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "localization_node");
  LocalizationNode node;
  ros::spin();
  return 0;
}
