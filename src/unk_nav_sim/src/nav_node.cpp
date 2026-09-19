// ─────────────────────────────────────────────────────────────────────────────
// nav_node.cpp
//
// 职责：unk_nav 规划核心的 ROS 接口壳。
//       订阅 ROS 话题 → 组装 unk::NavInput → 调用 unk::NavCore::plan()
//       → 调用 unk::PurePursuitController → 发布 geometry_msgs::Twist
//
// 两个定时器（控制环与规划环解耦）：
//   planCb   按 plan_freq（默认 10Hz）跑子目标 + A* + 限速，结果缓存下来；
//   controlCb 按 control_freq（默认 50Hz）拿缓存路径算 (v,w) 并发 /cmd_vel。
//   两者分开才有意义：同频时车在一个周期内已走了 v/freq（1.5m/s@10Hz=15cm）才被
//   重新指令一次。控制环用 currentCarInPlanFrame() 补回车在当前栅格系里的位置。
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
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include "unk_nav/nav_core.h"
#include "unk_nav/controller.h"
#include "unk_nav/geom_util.h"
#include "unk_nav/params_io.h"
#include "unk_nav/types.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

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
    slew_ = std::make_unique<unk::TwistSlewLimiter>(params_.cmd_a_max,
                                                   params_.cmd_w_dot_max);
    // 控制频率不高于规划频率时拆环没有信息增益（缓存路径不会变得更新），
    // 此时退回与规划同频，行为等同于旧版单定时器。
    const double ctrl_hz =
        (params_.control_freq > params_.plan_freq) ? params_.control_freq
                                                   : params_.plan_freq;
    ctrl_dt_ = 1.0 / ctrl_hz;

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
    pub_chain_ =
        nh.advertise<visualization_msgs::MarkerArray>("/unk_nav/chain", 1);

    // ── 规划定时器 ──
    timer_ = nh.createTimer(ros::Duration(1.0 / params_.plan_freq), &NavNode::planCb, this);
    // 控制定时器：默认单线程 spinner，两个定时器回调与订阅回调串在同一线程，
    // 故 workGrid() / 缓存路径的读写不需要加锁。
    ctrl_timer_ = nh.createTimer(ros::Duration(ctrl_dt_), &NavNode::controlCb, this);

    ROS_INFO("[nav_node] 启动：plan_freq=%.1fHz control_freq=%.1fHz, "
             "perception_range=%.1fm, v_max=%.2fm/s w_max=%.2frad/s",
             params_.plan_freq, 1.0 / ctrl_dt_, params_.perception_range, params_.v_max,
             params_.w_max);
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
    // 旧终点的路径不能再当指令源：等下一帧规划重新有路径才恢复发速
    have_plan_ = false;

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
    // 沿路无 odom 时用车体指令速度作车速估计（与“无定位控制层积分指令得相对位姿”同一假设），
    // 让“看多深随速度”在无定位下也能生效；speed_valid 仍只在真有 odom 时为真，
    // 避免拿指令速度误判前进位移棘轮（被顶住不动时指令速度仍 >0）。
    in.current_speed = has_odom_ ? current_speed_ : last_cmd_v_;
    last_plan_speed_ = in.current_speed;  // 供 publishChain 的 RViz 车速/看多深文本显示
    // 沿路模式前进位移棘轮依赖有效车速；无 odom 时置 false（退化为仅规划失败判定）
    in.speed_valid = has_odom_;

    // OccupancyGrid → unk::GridMap（字段一一对应，直接拷贝）
    in.local_grid = convertGrid(latest_grid_);

    // ── 2. 调用规划核心 ──
    unk::NavResult result = core_->plan(in);

    // ── 2.5 发布膨胀后的工作栅格（调试：规划器眼中的世界）──
    pub_work_grid_.publish(
        toRosGrid(core_->workGrid(), latest_grid_.header.frame_id));

    // ── 3. 缓存本帧规划输出给控制环发速；速度指令不在这里发 ──
    // 缓存的 path 与 work_grid 同在「本帧规划时刻的车体系」，控制环靠
    // currentCarInPlanFrame() 把车已走开的那一段补回来。
    ctrl_path_ = result.path;
    ctrl_speed_ = result.recommended_speed;
    // 路径为空时 compute() 本就返回零速，这里显式归入停车一类，避免沿用旧缓存
    ctrl_stop_ = result.emergency_stop || ctrl_path_.empty() ||
                 result.state == unk::NavState::ABORT ||
                 result.state == unk::NavState::IDLE ||
                 result.state == unk::NavState::ARRIVED;
    have_plan_ = true;
    // 重新锚定相对位姿：从此刻起车就是这个 path/grid 系的原点
    rel_ = unk::Pose2D();
    plan_pose_ = current_pose_;
    plan_pose_valid_ = has_odom_;

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

    // ── 8. 发布链式前瞻可视化（接力折线 + 跳点 + 终点切向 + κ 文本）──
    publishChain(result);

    // 调试日志（1Hz 节流）
    ROS_INFO_THROTTLE(1.0,
        "[nav_node] state=%s speed=%.2f path_pts=%zu reason=%s",
        unk::navStateName(result.state),
        result.recommended_speed,
        result.path.size(),
        result.reason.c_str());
  }

  // ── 定时器：控制主循环（只发 /cmd_vel，不做任何规划）──
  void controlCb(const ros::TimerEvent&) {
    geometry_msgs::Twist cmd;  // 默认全零
    // 还没规划过 / 急停 / 终态 / 无路径 → 立即零速，且绕过斜率限幅：
    // 限幅只用于把正常行驶抹柔，绝不能拖慢「能立刻停住」这件事。
    if (!have_plan_ || ctrl_stop_) {
      slew_->reset();
      last_cmd_v_ = 0.0;
      pub_cmd_.publish(cmd);
      return;
    }

    const unk::Pose2D car = currentCarInPlanFrame();
    unk::TwistCmd tc = controller_->compute(ctrl_path_, ctrl_speed_,
                                            core_->workGrid(), car);
    tc = slew_->limit(tc, ctrl_dt_);
    cmd.linear.x = tc.v;
    cmd.angular.z = tc.w;
    last_cmd_v_ = tc.v;  // 无 odom 时供 planCb 作车速估计
    pub_cmd_.publish(cmd);
    // 把这一 tick 走掉的位移记入相对位姿，供下个 tick 使用
    integrateRel(tc);
  }

  // 车在「规划时刻车体系」（= 缓存 path 与 work_grid 所在系）下的当前位姿。
  // 有 odom：用 odom 位姿差 T_plan⁻¹·T_now，实测值，准；
  // 无 odom（沿路不定位）：退化为积分自己发出的 (v,w)。一个规划周期（100ms）内
  // 轮地滑移只是二阶小量，远小于 10Hz 零阶保持本身带来的 15cm 滞后。
  unk::Pose2D currentCarInPlanFrame() const {
    if (plan_pose_valid_ && has_odom_) {
      const double dx = current_pose_.x - plan_pose_.x;
      const double dy = current_pose_.y - plan_pose_.y;
      const double c = std::cos(plan_pose_.yaw), s = std::sin(plan_pose_.yaw);
      unk::Pose2D r;
      r.x = c * dx + s * dy;
      r.y = -s * dx + c * dy;
      r.yaw = unk::geom::normalizeAngle(current_pose_.yaw - plan_pose_.yaw);
      return r;
    }
    return rel_;
  }

  // 常值 (v, w) 的圆弧精确积分（|w| 极小时退化为直线，避开 v/w 除零）
  void integrateRel(const unk::TwistCmd& tc) {
    if (std::fabs(tc.w) < 1e-3) {
      rel_.x += tc.v * ctrl_dt_ * std::cos(rel_.yaw);
      rel_.y += tc.v * ctrl_dt_ * std::sin(rel_.yaw);
      return;
    }
    const double dth = tc.w * ctrl_dt_;
    rel_.x += (tc.v / tc.w) * (std::sin(rel_.yaw + dth) - std::sin(rel_.yaw));
    rel_.y -= (tc.v / tc.w) * (std::cos(rel_.yaw + dth) - std::cos(rel_.yaw));
    rel_.yaw = unk::geom::normalizeAngle(rel_.yaw + dth);
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
  std::unique_ptr<unk::TwistSlewLimiter> slew_;

  // 状态
  bool has_odom_ = false;
  bool has_grid_ = false;
  bool goal_valid_ = false;
  double current_speed_ = 0.0;
  double last_cmd_v_ = 0.0;       // 最近一次下发的指令线速度（无 odom 时作车速估计）
  double last_plan_speed_ = 0.0;  // 本帧规划实际用于 lookahead 的车速（供 RViz 文本显示）
  unk::Pose2D current_pose_;
  unk::Point2D goal_;
  nav_msgs::Odometry latest_odom_;
  nav_msgs::OccupancyGrid latest_grid_;

  // 控制环（与规划环解耦）：缓存本帧规划结果 + 周期内相对位姿
  unk::Path ctrl_path_;
  double ctrl_speed_ = 0.0;
  bool have_plan_ = false;
  bool ctrl_stop_ = true;      // 初始为 true：首次规划前保持零速
  double ctrl_dt_ = 0.1;
  unk::Pose2D rel_;            // 无 odom：自上次规划以来的指令积分
  unk::Pose2D plan_pose_;      // 上次规划时刻的 odom 位姿
  bool plan_pose_valid_ = false;

  // ROS
  ros::Subscriber sub_odom_, sub_grid_, sub_goal_;
  ros::Publisher pub_cmd_, pub_path_, pub_state_, pub_work_grid_, pub_fan_, pub_goal_,
      pub_chain_;
  ros::Timer timer_, ctrl_timer_;

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

  // ── 发布链式前瞻可视化（base_link 系，MarkerArray）──
  // 原点→P1→P2→P3 接力折线（黄，即样条控制骨架）+ 各跳点球（P1 绿、后续黄）+ 最远跳
  // 点出射方向箭头（青）+ κ/curve 状态文本。直观看出"接力看了几跳、路往哪弯"。
  // 仅沿路模式且 curve_fit_enable 时 chain_hop_count>0；否则发 DELETE 清屏。
  void publishChain(const unk::NavResult& result) {
    visualization_msgs::MarkerArray arr;
    const ros::Time t = ros::Time::now();

    // (id=4) 车速 + 当前看多深 L 文本：始终显示（不依赖链），直观验证“看多深随速度”。
    // L = roadLookahead(v) = clamp(基准 + k·v, ≤ perception_range)；无 odom 时 v 取指令速度。
    {
      visualization_msgs::Marker sv;
      sv.header.frame_id = "base_link";
      sv.header.stamp = t;
      sv.ns = "chain";
      sv.id = 4;
      sv.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      sv.action = visualization_msgs::Marker::ADD;
      sv.pose.orientation.w = 1.0;
      sv.pose.position.z = 1.3;  // 车体上方
      sv.scale.z = 0.5;
      sv.color.r = 1.0; sv.color.g = 1.0; sv.color.b = 0.2; sv.color.a = 0.95;
      const double v = last_plan_speed_;
      const double L = params_.roadLookahead(v);
      char sbuf[96];
      std::snprintf(sbuf, sizeof(sbuf), "v=%.2f m/s  L=%.1f m", v, L);
      sv.text = sbuf;
      arr.markers.push_back(sv);
    }

    // 无链（终点模式 / curve_fit 关闭 / 无有效跳点）→ 逐个 DELETE 清掉上一帧
    if (result.chain_hop_count <= 0) {
      for (int id = 0; id < 4; ++id) {
        visualization_msgs::Marker d;
        d.header.frame_id = "base_link";
        d.header.stamp = t;
        d.ns = "chain";
        d.id = id;
        d.action = visualization_msgs::Marker::DELETE;
        arr.markers.push_back(d);
      }
      pub_chain_.publish(arr);
      return;
    }

    // 折线顶点：原点(0,0) → 各跳点
    std::vector<geometry_msgs::Point> pts;
    geometry_msgs::Point o;
    o.x = 0.0; o.y = 0.0; o.z = 0.10;
    pts.push_back(o);
    for (int i = 0; i < result.chain_hop_count; ++i) {
      geometry_msgs::Point p;
      p.x = result.chain_hops[i].x;
      p.y = result.chain_hops[i].y;
      p.z = 0.10;
      pts.push_back(p);
    }

    // (id=0) 接力折线 LINE_STRIP（黄）
    visualization_msgs::Marker line;
    line.header.frame_id = "base_link";
    line.header.stamp = t;
    line.ns = "chain";
    line.id = 0;
    line.type = visualization_msgs::Marker::LINE_STRIP;
    line.action = visualization_msgs::Marker::ADD;
    line.pose.orientation.w = 1.0;
    line.scale.x = 0.04;
    line.color.r = 1.0; line.color.g = 0.8; line.color.b = 0.0; line.color.a = 0.9;
    line.points = pts;
    arr.markers.push_back(line);

    // (id=1) 跳点球 SPHERE_LIST（P1 绿，后续黄）
    visualization_msgs::Marker spheres;
    spheres.header.frame_id = "base_link";
    spheres.header.stamp = t;
    spheres.ns = "chain";
    spheres.id = 1;
    spheres.type = visualization_msgs::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::Marker::ADD;
    spheres.pose.orientation.w = 1.0;
    spheres.scale.x = spheres.scale.y = spheres.scale.z = 0.22;
    for (size_t i = 1; i < pts.size(); ++i) {  // 跳过原点
      spheres.points.push_back(pts[i]);
      std_msgs::ColorRGBA c;
      if (i == 1) { c.r = 0.0; c.g = 1.0; c.b = 0.0; c.a = 0.95; }  // P1 绿
      else        { c.r = 1.0; c.g = 0.8; c.b = 0.0; c.a = 0.90; }  // 后续黄
      spheres.colors.push_back(c);
    }
    arr.markers.push_back(spheres);

    // (id=2) 样条终点出射方向箭头（青）——从最远跳点沿末段 hops 方向，仅当有第二跳
    visualization_msgs::Marker arrow;
    arrow.header.frame_id = "base_link";
    arrow.header.stamp = t;
    arrow.ns = "chain";
    arrow.id = 2;
    arrow.type = visualization_msgs::Marker::ARROW;
    arrow.pose.orientation.w = 1.0;
    arrow.scale.x = 0.05;  // 杆径
    arrow.scale.y = 0.12;  // 头径
    arrow.scale.z = 0.12;  // 头长
    arrow.color.r = 0.0; arrow.color.g = 1.0; arrow.color.b = 1.0; arrow.color.a = 0.95;
    if (result.chain_hop_count >= 2) {
      const unk::Point2D& pend = result.chain_hops[result.chain_hop_count - 1];
      const unk::Point2D& pprev = result.chain_hops[result.chain_hop_count - 2];
      const double th = std::atan2(pend.y - pprev.y, pend.x - pprev.x);
      const double L = 0.8;  // 箭头长度
      geometry_msgs::Point a, b;
      a.x = pend.x; a.y = pend.y; a.z = 0.12;
      b.x = pend.x + L * std::cos(th);
      b.y = pend.y + L * std::sin(th);
      b.z = 0.12;
      arrow.points.push_back(a);
      arrow.points.push_back(b);
      arrow.action = visualization_msgs::Marker::ADD;
    } else {
      arrow.action = visualization_msgs::Marker::DELETE;
    }
    arr.markers.push_back(arrow);

    // (id=3) κ / curve 状态文本（P1 上方）
    visualization_msgs::Marker txt;
    txt.header.frame_id = "base_link";
    txt.header.stamp = t;
    txt.ns = "chain";
    txt.id = 3;
    txt.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    txt.action = visualization_msgs::Marker::ADD;
    txt.pose.orientation.w = 1.0;
    txt.pose.position.x = result.chain_hops[0].x;
    txt.pose.position.y = result.chain_hops[0].y;
    txt.pose.position.z = 0.6;
    txt.scale.z = 0.3;  // 字高
    txt.color.r = 1.0; txt.color.g = 1.0; txt.color.b = 1.0; txt.color.a = 0.95;
    char buf[96];
    std::snprintf(buf, sizeof(buf), "kappa=%.3f\n%s", result.kappa_est,
                  result.curve_used ? "curve" : "line");
    txt.text = buf;
    arr.markers.push_back(txt);

    pub_chain_.publish(arr);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "nav_node");
  NavNode node;
  ros::spin();
  return 0;
}
