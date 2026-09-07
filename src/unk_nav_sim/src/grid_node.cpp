// ─────────────────────────────────────────────────────────────────────────────
// grid_node.cpp
//
// 职责：将 3D 点云（Velodyne HDL-32E）转换为 2D 占据栅格（base_link 坐标系）。
//
// 话题接口（松耦合，可独立替换）：
//   订阅：/velodyne_points (sensor_msgs/PointCloud2)
//   发布：/local_grid      (nav_msgs/OccupancyGrid)
//
// 算法：
//   1. 将点云从传感器坐标系变换到 base_link
//   2. Z 轴切片过滤（只保留 z ∈ [z_min, z_max] 的点）
//   3. 射线追踪：从原点到每个命中点之间的格子标记为 free(0)
//   4. 命中点所在格子标记为 occupied(100)
//   5. 未探测到的格子标记为 unknown(-1)
//
// 替换方式：实车上换为 2D 激光雷达时，只需改订阅话题和去掉 Z 过滤，
//           输出相同的 /local_grid 话题，下游无需改动。
// ─────────────────────────────────────────────────────────────────────────────

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <cmath>
#include <string>
#include <vector>

class GridNode {
public:
  GridNode()
      : tf_buffer_(), tf_listener_(tf_buffer_) {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // ── 参数 ──
    pnh.param<std::string>("input_topic", input_topic_, "/velodyne_points");
    pnh.param<std::string>("sensor_frame", sensor_frame_, "velodyne");
    pnh.param<std::string>("base_frame", base_frame_, "base_link");
    pnh.param<double>("resolution", resolution_, 0.05);
    pnh.param<double>("window_size", window_size_, 20.4);  // 1.7 × 12m
    pnh.param<double>("z_min", z_min_, 0.1);               // 过滤地面点（地面在 base_link 下约 0.23m）
    pnh.param<double>("z_max", z_max_, 1.5);               // base_link 上方 1.5m
    pnh.param<double>("max_range", max_range_, 12.0);      // 裁剪超过此距离的点

    // ── 栅格尺寸 ──
    grid_width_ = static_cast<int>(std::ceil(window_size_ / resolution_));
    grid_height_ = grid_width_;
    // 栅格原点在 base_link 坐标系中的位置（窗口以车为中心）
    origin_x_ = -window_size_ / 2.0;
    origin_y_ = -window_size_ / 2.0;

    // ── 话题 ──
    sub_ = nh.subscribe(input_topic_, 1, &GridNode::cloudCb, this);
    pub_grid_ = nh.advertise<nav_msgs::OccupancyGrid>("/local_grid", 1);

    ROS_INFO("[grid_node] 栅格 %d×%d, res=%.3f, window=%.1fm, z=[%.1f,%.1f]",
             grid_width_, grid_height_, resolution_, window_size_, z_min_, z_max_);
  }

private:
  void cloudCb(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // ── 1. TF 变换：sensor_frame → base_frame ──
    geometry_msgs::TransformStamped tf_msg;
    try {
      tf_msg = tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id,
                                          ros::Time(0), ros::Duration(0.05));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(2.0, "[grid_node] TF 变换失败: %s", ex.what());
      return;
    }

    // ── 2. 初始化栅格为 unknown(-1) ──
    nav_msgs::OccupancyGrid grid;
    grid.header.stamp = ros::Time::now();
    grid.header.frame_id = base_frame_;
    grid.info.resolution = static_cast<float>(resolution_);
    grid.info.width = static_cast<uint32_t>(grid_width_);
    grid.info.height = static_cast<uint32_t>(grid_height_);
    grid.info.origin.position.x = origin_x_;
    grid.info.origin.position.y = origin_y_;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(static_cast<size_t>(grid_width_) * grid_height_, -1);

    // ── 3. 遍历点云，变换 + Z 过滤 + 栅格化 ──
    const double tx = tf_msg.transform.translation.x;
    const double ty = tf_msg.transform.translation.y;
    const double tz = tf_msg.transform.translation.z;
    // 简化：Velodyne 与 base_link 之间只有平移（固定安装，无旋转偏差）
    // 若需要精确旋转，可用 tf2::doTransform 逐点变换

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    // 角度覆盖：记录哪些水平方向有回波（用于给无回波方向补 free）
    const int kNumAngleBins = 720;  // 0.5°/bin
    std::vector<char> hit_angle(static_cast<size_t>(kNumAngleBins), 0);

    int hit_count = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      // 变换到 base_link
      double px = static_cast<double>(*iter_x) + tx;
      double py = static_cast<double>(*iter_y) + ty;
      double pz = static_cast<double>(*iter_z) + tz;

      // 跳过 NaN / Inf
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz))
        continue;

      // Z 轴切片过滤（模拟 2D 激光雷达的水平面）
      if (pz < z_min_ || pz > z_max_)
        continue;

      // 距离裁剪
      double dist = std::sqrt(px * px + py * py);
      if (dist > max_range_ || dist < 0.1)
        continue;

      // 记录该回波的方位角落在哪个角度 bin
      const double angle = std::atan2(py, px);  // [-pi, pi]
      int bin = static_cast<int>((angle + M_PI) / (2.0 * M_PI) * kNumAngleBins);
      if (bin < 0) bin = 0;
      if (bin >= kNumAngleBins) bin = kNumAngleBins - 1;
      hit_angle[static_cast<size_t>(bin)] = 1;

      // 射线追踪：从原点(0,0)到命中点(px,py)，沿途标记 free
      rayTrace(grid.data, 0.0, 0.0, px, py);

      // 命中点标记 occupied
      int gx = 0, gy = 0;
      if (worldToGrid(px, py, &gx, &gy)) {
        grid.data[static_cast<size_t>(gy) * grid_width_ + gx] = 100;
        ++hit_count;
      }
    }

    // ── 3.5 无回波方向补 free ──
    // 旋转雷达假设水平 360° 全覆盖：某方向量程内无回波 = 该射线路径无障碍，
    // 沿途应标 free 到量程边界（而非留 unknown）。仿真世界为实心障碍，
    // 不存在玻璃/吸光材质“有障碍却无回波”的误判。
    for (int b = 0; b < kNumAngleBins; ++b) {
      if (hit_angle[static_cast<size_t>(b)]) continue;
      const double a =
          (static_cast<double>(b) + 0.5) / kNumAngleBins * 2.0 * M_PI - M_PI;
      rayTrace(grid.data, 0.0, 0.0, std::cos(a) * max_range_, std::sin(a) * max_range_);
    }

    // ── 4. 发布栅格 ──
    pub_grid_.publish(grid);

    ROS_DEBUG_THROTTLE(1.0, "[grid_node] 点云→栅格：命中 %d 点", hit_count);
  }

  // 世界坐标(base_link系) → 栅格索引
  bool worldToGrid(double x, double y, int* gx, int* gy) const {
    int mx = static_cast<int>(std::floor((x - origin_x_) / resolution_));
    int my = static_cast<int>(std::floor((y - origin_y_) / resolution_));
    if (mx < 0 || my < 0 || mx >= grid_width_ || my >= grid_height_)
      return false;
    *gx = mx;
    *gy = my;
    return true;
  }

  // 射线追踪：从 (x0,y0) 到 (x1,y1) 沿途标记 free(0)，终点不标（由调用方决定）。
  // 用 DDA 栅格射线遍历。终点即使落在栅格外也照常沿方向步进，一旦离开栅格
  // 边界即停止——这样量程 > 半窗时，界外回波点/无回波射线的沿途 free 不会被漏标。
  void rayTrace(std::vector<int8_t>& data,
                double x0, double y0, double x1, double y1) const {
    int gx0 = 0, gy0 = 0;
    if (!worldToGrid(x0, y0, &gx0, &gy0)) {
      // 原点在栅格外（不应发生），用中心格代替
      gx0 = grid_width_ / 2;
      gy0 = grid_height_ / 2;
    }
    // 终点索引：允许落在栅格外，仅用于确定射线方向
    const int gx1 = static_cast<int>(std::floor((x1 - origin_x_) / resolution_));
    const int gy1 = static_cast<int>(std::floor((y1 - origin_y_) / resolution_));

    // DDA 射线遍历
    const int dx = std::abs(gx1 - gx0);
    const int dy = std::abs(gy1 - gy0);
    const int sx = (gx0 < gx1) ? 1 : -1;
    const int sy = (gy0 < gy1) ? 1 : -1;
    int err = dx - dy;

    int cx = gx0, cy = gy0;
    // 不标记起点（车体自身位置），也不标记终点（由调用方标记 occupied）
    while (cx != gx1 || cy != gy1) {
      const int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; cx += sx; }
      if (e2 < dx)  { err += dx; cy += sy; }
      if (cx == gx1 && cy == gy1) break;  // 终点不在此标记
      if (cx < 0 || cy < 0 || cx >= grid_width_ || cy >= grid_height_)
        break;  // 从界内原点出发的射线，一旦出界即停止
      size_t idx = static_cast<size_t>(cy) * grid_width_ + cx;
      if (data[idx] == -1) data[idx] = 0;  // 只把 unknown 改为 free
    }
  }

  // ── 参数 ──
  std::string input_topic_;
  std::string sensor_frame_;
  std::string base_frame_;
  double resolution_;
  double window_size_;
  double z_min_, z_max_;
  double max_range_;

  // ── 栅格几何 ──
  int grid_width_ = 0, grid_height_ = 0;
  double origin_x_ = 0.0, origin_y_ = 0.0;

  // ── ROS ──
  ros::Subscriber sub_;
  ros::Publisher pub_grid_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "grid_node");
  GridNode node;
  ros::spin();
  return 0;
}
