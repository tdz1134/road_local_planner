// ─────────────────────────────────────────────────────────────────────────────
// nav_node.cpp
//
// 职责：unk_nav 规划核心的 ROS 接口壳。
//       订阅 ROS 话题 → 组装 unk::NavInput → 调用 unk::NavCore::plan()
//       → 调用 unk::PurePursuitController → 发布 geometry_msgs::Twist
//
// 本文件不含任何算法，所有规划与控制逻辑在 unk_nav 库中。
//
// 参数：全部算法参数读自 unk_nav/config/nav_params.yaml（unk::loadNavParams）。
// ROS 侧只传一个私有参数 config_file（文件路径），参数不走参数服务器——
// 仿真 / 离线 demo / 实车 MDC 共用同一份配置、同一个加载入口。
//
// 沿路模式（无定位）：当配置里 follow_road=true（road_follow.launch 传
// nav_params_road.yaml），NavCore 走 road_follow 从栅格走廊几何直接推子目标，本
// 节点不再需要全局终点；/odom 仅用于提供 body 系车速（本体感知），规划完全不读
// pose。此时路径以 base_link 系发布（RViz Fixed Frame 设 base_link）。
//
// 话题接口（松耦合，可独立替换）：
//   订阅：/odom        (nav_msgs/Odometry)     ← 来自 localization_node
//         /local_grid  (nav_msgs/OccupancyGrid)← 来自 grid_node
//         /move_base_simple/goal (geometry_msgs/PoseStamped) ← 来自 RViz "2D Nav Goal"
//   发布：/cmd_vel     (geometry_msgs/Twist)   → Scout v2 速度指令
//         /unk_nav/path (nav_msgs/Path)        → RViz 路径可视化
//         /unk_nav/state (std_msgs/String)     → 当前导航状态（GO/IDLE/ABORT...）
//         /unk_nav/work_grid (nav_msgs/OccupancyGrid) → 膨胀后的工作栅格（调试：规划器眼中的世界）
//
// 替换方式：
//   - 换控制器：改 unk_nav/controller.h（或新建控制器类）
//   - 换规划器：改 unk_nav/nav_core.h
//   - 换定位源：只改 /odom 话题名
// ─────────────────────────────────────────────────────────────────────────────

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include "unk_nav/nav_core.h"
#include "unk_nav/controller.h"
#include "unk_nav/params_io.h"
#include "unk_nav/types.h"

#include <cmath>
#include <string>

// ═══════════════════════════════════════════════════════════════════════════
// 导航节点主体（纯 ROS 接口壳，不含算法）
// ═══════════════════════════════════════════════════════════════════════════
class NavNode {
public:
  NavNode() {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ── 从算法库 YAML 配置加载全部 unk_nav 参数 ──
    std::string config_file;
    pnh.param<std::string>("config_file", config_file, "");
    if (config_file.empty()) {
      ROS_FATAL("[nav_node] 缺少私有参数 config_file，例："
                "$(find unk_nav)/config/nav_params.yaml");
      ros::shutdown();
      return;
    }
    std::string err;
    if (!unk::loadNavParams(config_file, &params_, &err)) {
      ROS_FATAL("[nav_node] 配置加载失败 [%s]：%s", config_file.c_str(), err.c_str());
      ros::shutdown();
      return;
    }
    ROS_INFO("[nav_node] 配置已加载：%s", config_file.c_str());

    // ── 初始化规划核心 + 控制器 ──
    core_ = std::make_unique<unk::NavCore>(params_);
    controller_ = std::make_unique<unk::PurePursuitController>(
        params_.pursuit_lookahead, params_.w_max);

    // ── 话题：订阅 ──
    sub_odom_ = nh.subscribe("/odom", 10, &NavNode::odomCb, this);
    sub_grid_ = nh.subscribe("/local_grid", 1, &NavNode::gridCb, this);
    sub_goal_ = nh.subscribe("/move_base_simple/goal", 1, &NavNode::goalCb, this);

    // ── 话题：发布 ──
    pub_cmd_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    pub_path_ = nh.advertise<nav_msgs::Path>("/unk_nav/path", 1, true);
    pub_state_ = nh.advertise<std_msgs::String>("/unk_nav/state", 1, true);
    pub_work_grid_ =
        nh.advertise<nav_msgs::OccupancyGrid>("/unk_nav/work_grid", 1, true);
    pub_fan_ =
        nh.advertise<visualization_msgs::Marker>("/unk_nav/fan_candidates", 1);
    pub_goal_ =
        nh.advertise<visualization_msgs::Marker>("/unk_nav/goal_marker", 1);

    // ── 规划定时器 ──
    timer_ = nh.createTimer(ros::Duration(1.0 / params_.plan_freq), &NavNode::planCb, this);

    ROS_INFO("[nav_node] 启动：plan_freq=%.1fHz, sensor_range=%.1fm, v_max=%.2fm/s",
             params_.plan_freq, params_.sensor_range, params_.v_max);
  }

private:
  // ── 回调：里程计 ──
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_odom_ = *msg;
    has_odom_ = true;

    // 提取 yaw
    current_pose_.x = msg->pose.pose.position.x;
    current_pose_.y = msg->pose.pose.position.y;
    current_pose_.yaw = tf2::getYaw(msg->pose.pose.orientation);

    // 提取当前速度（车体系线速度）
    current_speed_ = msg->twist.twist.linear.x;
  }

  // ── 回调：栅格 ──
  void gridCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    latest_grid_ = *msg;
    has_grid_ = true;
  }

  // ── 回调：RViz 目标点 ──
  void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_.x = msg->pose.position.x;
    goal_.y = msg->pose.position.y;
    goal_valid_ = true;

    // 通知 NavCore 终点变化（清除 ARRIVED/ABORT 锁存）
    core_->reset();

    ROS_INFO("[nav_node] 收到新终点: (%.2f, %.2f)", goal_.x, goal_.y);
  }

  // ── 定时器：规划主循环 ──
  void planCb(const ros::TimerEvent&) {
    // 沿路模式只需栅格（不依赖定位/终点）；终点模式仍需 odom 提供 pose。
    if (!has_grid_ || (!params_.follow_road && !has_odom_)) return;

    // ── 1. 组装 NavInput ──
    unk::NavInput in;
    in.now = ros::Time::now().toSec();
    in.vehicle_pose = current_pose_;
    in.goal = goal_;
    // 沿路模式：不使用全局终点（NavCore 走 road_follow，忽略 goal/pose）
    in.goal_valid = params_.follow_road ? false : goal_valid_;
    in.current_speed = current_speed_;
    // 沿路模式前进位移棘轮依赖有效车速；无 odom 时置 false（退化为仅规划失败判定）
    in.speed_valid = has_odom_;

    // OccupancyGrid → unk::GridMap（字段一一对应，直接拷贝）
    in.local_grid = convertGrid(latest_grid_);

    // ── 2. 调用规划核心 ──
    unk::NavResult result = core_->plan(in);

    // ── 2.5 发布膨胀后的工作栅格（调试：规划器眼中的世界）──
    pub_work_grid_.publish(
        toRosGrid(core_->workGrid(), latest_grid_.header.frame_id));

    // ── 3. 计算速度指令 ──
    geometry_msgs::Twist cmd;
    if (result.emergency_stop || result.state == unk::NavState::ABORT ||
        result.state == unk::NavState::IDLE) {
      // 急停 / 放弃 / 无目标 → 零速
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    } else if (result.state == unk::NavState::ARRIVED) {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    } else {
      // GO / RECOVERY → 纯跟踪（传入膨胀后的工作栅格做弦碰撞检测）
      unk::TwistCmd tc = controller_->compute(result.path, result.recommended_speed,
                                              core_->workGrid());
      cmd.linear.x = tc.v;
      cmd.angular.z = tc.w;
    }
    pub_cmd_.publish(cmd);

    // ── 4. 发布状态 ──
    std_msgs::String state_msg;
    state_msg.data = std::string(unk::navStateName(result.state)) +
                     " | " + result.reason;
    pub_state_.publish(state_msg);

    // ── 5. 发布路径可视化 ──
    publishPathViz(result.path);

    // ── 6. 发布扇形候选可视化（调试：蓝色=候选，红色=选中）──
    publishFanCandidates(result);

    // ── 7. 发布终点 Marker（base_link 系绿色圆柱）──
    publishGoalMarker(result);

    // 调试日志（1Hz 节流）
    ROS_INFO_THROTTLE(1.0,
        "[nav_node] state=%s speed=%.2f path_pts=%zu reason=%s",
        unk::navStateName(result.state),
        result.recommended_speed,
        result.path.size(),
        result.reason.c_str());
  }

  // ── nav_msgs/OccupancyGrid → unk::GridMap ──
  unk::GridMap convertGrid(const nav_msgs::OccupancyGrid& ros_grid) const {
    unk::GridMap g;
    g.resolution = ros_grid.info.resolution;
    g.origin_x = ros_grid.info.origin.position.x;
    g.origin_y = ros_grid.info.origin.position.y;
    g.width = static_cast<int>(ros_grid.info.width);
    g.height = static_cast<int>(ros_grid.info.height);
    g.data = ros_grid.data;  // int8_t 直接对应，值域一致
    return g;
  }

  // ── unk::GridMap → nav_msgs/OccupancyGrid（反向，用于调试可视化）──
  nav_msgs::OccupancyGrid toRosGrid(const unk::GridMap& g,
                                    const std::string& frame) const {
    nav_msgs::OccupancyGrid m;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = frame;
    m.info.resolution = static_cast<float>(g.resolution);
    m.info.width = static_cast<uint32_t>(g.width);
    m.info.height = static_cast<uint32_t>(g.height);
    m.info.origin.position.x = g.origin_x;
    m.info.origin.position.y = g.origin_y;
    m.info.origin.orientation.w = 1.0;
    m.data = g.data;  // 行优先 data[gy*width+gx]，布局一致，直接拷贝
    return m;
  }

  // ── 路径可视化（沿路模式：base_link 系；终点模式：车体系 → odom 系）──
  void publishPathViz(const unk::Path& path) {
    if (path.empty()) return;

    nav_msgs::Path viz;
    viz.header.stamp = ros::Time::now();
    // 沿路模式无定位（或无 odom）：路径本就在车体系，直接以 base_link 发布，
    // RViz Fixed Frame 设 base_link 即见车在原点、走廊随车滚动。
    const bool base_frame = params_.follow_road || !has_odom_;
    viz.header.frame_id = base_frame ? "base_link" : "odom";

    const double cos_yaw = std::cos(current_pose_.yaw);
    const double sin_yaw = std::sin(current_pose_.yaw);

    for (const auto& pt : path) {
      geometry_msgs::PoseStamped ps;
      if (base_frame) {
        ps.pose.position.x = pt.p.x;
        ps.pose.position.y = pt.p.y;
      } else {
        // 车体系 → odom 系
        ps.pose.position.x = current_pose_.x + pt.p.x * cos_yaw - pt.p.y * sin_yaw;
        ps.pose.position.y = current_pose_.y + pt.p.x * sin_yaw + pt.p.y * cos_yaw;
      }
      ps.pose.position.z = 0.1;  // 略高于地面，RViz 里好看
      ps.pose.orientation.w = 1.0;
      viz.poses.push_back(ps);
    }
    pub_path_.publish(viz);
  }

  // ── 成员 ──
  unk::NavParams params_;
  std::unique_ptr<unk::NavCore> core_;
  std::unique_ptr<unk::PurePursuitController> controller_;

  // 状态
  bool has_odom_ = false;
  bool has_grid_ = false;
  bool goal_valid_ = false;
  double current_speed_ = 0.0;
  unk::Pose2D current_pose_;
  unk::Point2D goal_;
  nav_msgs::Odometry latest_odom_;
  nav_msgs::OccupancyGrid latest_grid_;

  // ROS
  ros::Subscriber sub_odom_, sub_grid_, sub_goal_;
  ros::Publisher pub_cmd_, pub_path_, pub_state_, pub_work_grid_, pub_fan_, pub_goal_;
  ros::Timer timer_;

  // ── 发布扇形候选可视化 ──
  // 单个 LINE_LIST Marker + 逐顶点着色：蓝色=候选射线，红色=选中
  void publishFanCandidates(const unk::NavResult& result) {
    // 蓝色：候选射线
    std_msgs::ColorRGBA blue;
    blue.r = 0.2; blue.g = 0.5; blue.b = 1.0; blue.a = 0.6;
    // 红色：选中
    std_msgs::ColorRGBA red;
    red.r = 1.0; red.g = 0.1; red.b = 0.1; red.a = 0.95;

    visualization_msgs::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = ros::Time::now();
    m.ns = "fan";
    m.id = 0;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.06;

    for (const auto& cand : result.fan_candidates) {
      if (!cand.feasible) continue;
      geometry_msgs::Point p0, p1;
      p0.x = 0; p0.y = 0; p0.z = 0.06;
      p1.x = std::cos(cand.bearing) * cand.d_free;
      p1.y = std::sin(cand.bearing) * cand.d_free;
      p1.z = 0.06;
      m.points.push_back(p0);
      m.points.push_back(p1);
      const std_msgs::ColorRGBA& c = cand.selected ? red : blue;
      m.colors.push_back(c);
      m.colors.push_back(c);
    }

    if (m.points.empty()) {
      // 无可行候选：发 DELETE 清除上一帧，不发空 LINE_LIST（RViz 报错）
      m.action = visualization_msgs::Marker::DELETE;
    } else {
      m.action = visualization_msgs::Marker::ADD;
    }
    pub_fan_.publish(m);
  }

  // ── 发布终点 Marker（base_link 系绿色圆柱）──
  void publishGoalMarker(const unk::NavResult& result) {
    visualization_msgs::Marker m;
    m.header.frame_id = "base_link";
    m.header.stamp = ros::Time::now();
    m.ns = "goal";
    m.id = 0;
    m.type = visualization_msgs::Marker::CYLINDER;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.3;  // 直径
    m.scale.y = 0.3;
    m.scale.z = 0.6;  // 高度
    m.color.r = 0.0;
    m.color.g = 1.0;
    m.color.b = 0.0;
    m.color.a = 0.8;

    if (result.goal_base_valid) {
      m.action = visualization_msgs::Marker::ADD;
      m.pose.position.x = result.goal_base.x;
      m.pose.position.y = result.goal_base.y;
      m.pose.position.z = 0.3;  // 圆柱中心抬高，底部贴地
    } else {
      m.action = visualization_msgs::Marker::DELETE;
    }
    pub_goal_.publish(m);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "nav_node");
  NavNode node;
  ros::spin();
  return 0;
}
