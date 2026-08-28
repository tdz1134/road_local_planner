#include "rlp_node/planner_node.h"

#include <algorithm>
#include <cmath>
#include <sstream>

#include <tf2/utils.h>

namespace rlp {

// ── 内部辅助（仅本文件使用，不对外暴露）──────────────────────────
namespace {

rlp::GridMap toCoreGrid(const nav_msgs::OccupancyGrid& m) {
  rlp::GridMap g;
  g.resolution = m.info.resolution;
  g.origin_x = m.info.origin.position.x;
  g.origin_y = m.info.origin.position.y;
  g.width = static_cast<int>(m.info.width);
  g.height = static_cast<int>(m.info.height);
  g.data = m.data;
  return g;
}

std::vector<rlp::Point2D> toCorePoints(const std::vector<geometry_msgs::Point32>& pts) {
  std::vector<rlp::Point2D> out;
  out.reserve(pts.size());
  for (const auto& p : pts) out.push_back({p.x, p.y});
  return out;
}

double normalizeAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace

// ── 公共接口（在 planner_node.h 中声明）──────────────────────────

#define RLP_GET_PARAM(name) pnh_.param(#name, params_.name, params_.name)

PlannerNode::PlannerNode(ros::NodeHandle& pnh) : pnh_(pnh) {
  loadParams();
  core_.reset(new planner::PlannerCore(params_));
  tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  // 输入（私有话题，launch 中可 remap）
  sub_map_ = pnh_.subscribe("local_map", 1, &PlannerNode::mapCb, this);
  sub_left_ = pnh_.subscribe("boundaries/left", 1, &PlannerNode::leftCb, this);
  sub_right_ = pnh_.subscribe("boundaries/right", 1, &PlannerNode::rightCb, this);
  sub_quality_ = pnh_.subscribe("localization/quality", 1, &PlannerNode::qualityCb, this);
  sub_odom_ = pnh_.subscribe("odom", 1, &PlannerNode::odomCb, this);
  sub_goal_ = pnh_.subscribe("global_goal", 1, &PlannerNode::goalCb, this);

  // 输出
  pub_path_ = pnh_.advertise<nav_msgs::Path>("plan", 1);
  pub_cmd_ = pnh_.advertise<geometry_msgs::Twist>("speed_cmd", 1);
  pub_status_ = pnh_.advertise<std_msgs::String>("status", 1);

  timer_ = pnh_.createTimer(ros::Duration(1.0 / std::max(0.5, params_.plan_freq)),
                            &PlannerNode::onTimer, this);
  ROS_INFO("road_local_planner ready: freq=%.1fHz frame=%s v_max=%.1fm/s",
           params_.plan_freq, vehicle_frame_.c_str(), params_.v_max);
}

void PlannerNode::loadParams() {
  RLP_GET_PARAM(v_max);
  RLP_GET_PARAM(a_decel_max);
  RLP_GET_PARAM(a_lat_max);
  RLP_GET_PARAM(w_max);
  RLP_GET_PARAM(t_reaction);
  RLP_GET_PARAM(safety_margin);
  RLP_GET_PARAM(plan_freq);
  RLP_GET_PARAM(path_spacing);
  RLP_GET_PARAM(lookahead_time);
  RLP_GET_PARAM(min_lookahead);
  RLP_GET_PARAM(lateral_offsets);
  RLP_GET_PARAM(n_goal_bearings);
  RLP_GET_PARAM(goal_fan_deg);
  RLP_GET_PARAM(follow_alg);
  RLP_GET_PARAM(search_alg);
  RLP_GET_PARAM(free_alg);
  RLP_GET_PARAM(boundary_timeout);
  RLP_GET_PARAM(corridor_hold_max);
  RLP_GET_PARAM(corridor_inflate_rate);
  RLP_GET_PARAM(width_min);
  RLP_GET_PARAM(width_max);
  RLP_GET_PARAM(boundary_margin_base);
  RLP_GET_PARAM(boundary_margin_speed_gain);
  RLP_GET_PARAM(q_high);
  RLP_GET_PARAM(q_low);
  RLP_GET_PARAM(mode_dwell);
  RLP_GET_PARAM(w_offroad);
  RLP_GET_PARAM(w_smooth);
  RLP_GET_PARAM(w_progress);
  RLP_GET_PARAM(w_consistency);
  RLP_GET_PARAM(collision_cost);
  pnh_.param("vehicle_frame", vehicle_frame_, std::string("base_link"));
}

// ---------------- 输入回调：只缓存最新一帧 ----------------

void PlannerNode::mapCb(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  // 假定局部栅格已给出在车体系（若否，需在此处按 msg->header.frame_id 变换）
  // TODO(后续): frame 不一致时用 tf 把栅格重投影到车体系
  std::lock_guard<std::mutex> lk(mtx_);
  map_ = toCoreGrid(*msg);
  have_map_ = true;
}

void PlannerNode::leftCb(const rlp_node::Boundary::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  // 空点列不覆盖旧数据，缺失由道路层超时机制判定
  boundary_cache_.left = toCorePoints(msg->points);
}

void PlannerNode::rightCb(const rlp_node::Boundary::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  boundary_cache_.right = toCorePoints(msg->points);
}

void PlannerNode::qualityCb(const rlp_node::LocalizationQuality::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  quality_ = msg->quality;
}

void PlannerNode::odomCb(const nav_msgs::Odometry::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  speed_ = msg->twist.twist.linear.x;
}

void PlannerNode::goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  std::lock_guard<std::mutex> lk(mtx_);
  goal_ = *msg;
  goal_received_ = true;
}

// ---------------- 主循环 ----------------

void PlannerNode::onTimer(const ros::TimerEvent& event) {
  (void)event;
  std::lock_guard<std::mutex> lk(mtx_);
  if (!have_map_) {
    ROS_WARN_THROTTLE(2.0, "waiting for ~local_map ...");
    return;
  }

  planner::PlannerInput in;
  in.now = ros::Time::now().toSec();
  in.map = map_;
  in.boundaries = boundary_cache_;
  in.boundaries.stamp = in.now;
  in.localization_quality = quality_;
  in.current_speed = speed_;

  // 全局终点 → 车体系（定位差的车时 tf 可能不可用 → goal_valid=false）
  in.goal_valid = false;
  if (goal_received_) {
    try {
      const geometry_msgs::TransformStamped tr = tf_buffer_.lookupTransform(
          vehicle_frame_, goal_.header.frame_id, ros::Time(0), ros::Duration(0.05));
      const double yaw = tf2::getYaw(tr.transform.rotation);
      const double c = std::cos(yaw), s = std::sin(yaw);
      const double gx = goal_.pose.position.x, gy = goal_.pose.position.y;
      in.goal.x = c * gx - s * gy + tr.transform.translation.x;
      in.goal.y = s * gx + c * gy + tr.transform.translation.y;
      in.goal_valid = true;
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "global_goal tf failed: %s", ex.what());
    }
  }

  const planner::PlanResult res = core_->plan(in);

  // ---- 发布路径 ----
  nav_msgs::Path pm;
  pm.header.frame_id = vehicle_frame_;
  pm.header.stamp = ros::Time::now();
  const size_t n = res.path.size();
  for (size_t i = 0; i < n; ++i) {
    geometry_msgs::PoseStamped ps;
    ps.header = pm.header;
    ps.pose.position.x = res.path[i].p.x;
    ps.pose.position.y = res.path[i].p.y;
    const size_t a = i, b = std::min(i + 1, n - 1);
    double h = 0.0;
    if (a != b) {
      h = std::atan2(res.path[b].p.y - res.path[a].p.y, res.path[b].p.x - res.path[a].p.x);
    } else if (i >= 1) {
      h = std::atan2(res.path[i].p.y - res.path[i - 1].p.y,
                     res.path[i].p.x - res.path[i - 1].p.x);
    }
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, h);
    ps.pose.orientation = tf2::toMsg(q);
    pm.poses.push_back(ps);
  }
  pub_path_.publish(pm);

  // ---- 发布推荐速度（v1：对路径上 ~2m 预瞄点做纯跟踪，TODO 后续交给速度规划层）----
  geometry_msgs::Twist tw;
  tw.linear.x = res.emergency_stop ? 0.0 : res.recommended_speed;
  if (!res.path.empty() && tw.linear.x > 0.01) {
    const double ld = 2.0;
    auto tgt = res.path.back();
    for (const auto& pp : res.path) {
      if (pp.s >= ld) {
        tgt = pp;
        break;
      }
    }
    const double alpha = std::atan2(tgt.p.y, tgt.p.x);
    const double l = std::max(std::hypot(tgt.p.x, tgt.p.y), 0.5);
    tw.angular.z = tw.linear.x * 2.0 * std::sin(normalizeAngle(alpha)) / l;
  }
  pub_cmd_.publish(tw);

  // ---- 状态 ----
  std_msgs::String st;
  std::ostringstream oss;
  oss << "method=" << res.method << " alg=" << res.algorithm
      << " mode=" << modeName(res.mode)
      << " boundary=" << road::boundaryStateName(res.boundary_state)
      << " conf=" << res.corridor_confidence << " v_rec=" << res.recommended_speed
      << " estop=" << (res.emergency_stop ? 1 : 0) << " reason=" << res.reason;
  st.data = oss.str();
  pub_status_.publish(st);
  if (res.emergency_stop) {
    ROS_WARN_THROTTLE(0.5, "EMERGENCY STOP: %s", res.reason.c_str());
  }
}

}  // namespace rlp
