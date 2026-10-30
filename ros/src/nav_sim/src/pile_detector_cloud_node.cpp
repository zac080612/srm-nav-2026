// 实车感知节点：订阅 Livox 点云，同时产出两样东西：
//   1. /piles（MarkerArray）：桩位置，给 figure8 做 8 字
//   2. /scan（LaserScan）  ：2D 激光，给 figure8 做避障
// 两者都基于同一份「高度切片 + 投影后的 2D 点」，不需要额外装 pointcloud_to_laserscan。

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav/pile_detect.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_set>
#include <vector>

using std::placeholders::_1;

using nav::Vec2;

class PerceptionNode : public rclcpp::Node {
 public:
  PerceptionNode() : Node("perception") {
    // 高度切片：只保留地面以上这一段（桩的高度范围），去掉地面和天花板。
    // 实车放宽：下限负余量（tf_z 有误差）、上限覆盖半圆+细杆下部。
    declare_parameter("min_height", -0.05);
    declare_parameter("max_height", 0.30);
    // 距离过滤：太近是自己车体，太远是墙/杂物
    declare_parameter("min_range", 0.30);
    declare_parameter("max_range", 6.00);
    // 2D 体素降采样边长（米）：点太多会让聚类变慢
    declare_parameter("voxel", 0.05);
    // 雷达相对车体的平面变换（livox_frame → base_link），默认全 0，上车实测后填
    declare_parameter("tf_x", 0.0);
    declare_parameter("tf_y", 0.0);
    declare_parameter("tf_z", 0.0);
    declare_parameter("tf_yaw", 0.0);
    declare_parameter("cloud_topic", "/livox/lidar");
    // 桩几何：下半部是半径 10cm 的半圆（高约 10cm），上半部是半径 1cm 细杆。
    // 半径放宽（拟合误差+距离变化），弧长检查先关（点云视角下弧覆盖不稳定）。
    declare_parameter("min_radius", 0.05);
    declare_parameter("max_radius", 0.35);
    declare_parameter("min_points", 3);
    declare_parameter("cluster_dist", 0.20);
    declare_parameter("max_fit_rms", 0.06);
    declare_parameter("min_arc_deg", 0.0);

    min_h_ = get_parameter("min_height").as_double();
    max_h_ = get_parameter("max_height").as_double();
    min_r_ = get_parameter("min_range").as_double();
    max_r_ = get_parameter("max_range").as_double();
    voxel_ = get_parameter("voxel").as_double();
    tf_x_ = get_parameter("tf_x").as_double();
    tf_y_ = get_parameter("tf_y").as_double();
    tf_z_ = get_parameter("tf_z").as_double();
    tf_yaw_ = get_parameter("tf_yaw").as_double();
    const std::string topic = get_parameter("cloud_topic").as_string();

    params_.min_radius = get_parameter("min_radius").as_double();
    params_.max_radius = get_parameter("max_radius").as_double();
    params_.min_points = static_cast<size_t>(get_parameter("min_points").as_int());
    params_.cluster_dist = get_parameter("cluster_dist").as_double();
    params_.max_fit_rms = get_parameter("max_fit_rms").as_double();
    params_.min_arc_deg = get_parameter("min_arc_deg").as_double();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&PerceptionNode::on_cloud, this, _1));
    piles_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/piles", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);

    RCLCPP_INFO(get_logger(), "感知节点启动：订阅 %s → 发布 /piles + /scan", topic.c_str());
  }

 private:
  // 点云转 2D 点：高度切片 + 坐标变换 + 距离过滤 + 体素降采样
  std::vector<Vec2> to_points(const sensor_msgs::msg::PointCloud2& msg) const {
    std::vector<Vec2> pts;
    const double c = std::cos(tf_yaw_);
    const double s = std::sin(tf_yaw_);
    const double inv_voxel = 1.0 / std::max(voxel_, 1e-3);
    std::unordered_set<int64_t> seen;   // 体素去重

    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const double z = tf_z_ + static_cast<double>(*iz);
      if (z < min_h_ || z > max_h_) continue;      // 高度切片
      const double x = tf_x_ + c * static_cast<double>(*ix) - s * static_cast<double>(*iy);
      const double y = tf_y_ + s * static_cast<double>(*ix) + c * static_cast<double>(*iy);
      const double r = std::hypot(x, y);
      if (r < min_r_ || r > max_r_) continue;      // 距离过滤
      const int gx = static_cast<int>(std::floor(x * inv_voxel));
      const int gy = static_cast<int>(std::floor(y * inv_voxel));
      const int64_t key = (static_cast<int64_t>(gx) << 32) ^
                          static_cast<int64_t>(static_cast<uint32_t>(gy));
      if (!seen.insert(key).second) continue;
      pts.push_back({x, y});
    }
    return pts;
  }

  // 2D 点 → LaserScan（按角度分桶，每桶取最近距离），给避障用
  void publish_scan(const std::vector<Vec2>& pts,
                    const std_msgs::msg::Header& header) {
    sensor_msgs::msg::LaserScan scan;
    scan.header = header;
    scan.angle_min = static_cast<float>(-M_PI);
    scan.angle_max = static_cast<float>(M_PI);
    scan.angle_increment = static_cast<float>(0.5 * M_PI / 180.0);   // 0.5 度
    scan.range_min = static_cast<float>(min_r_);
    scan.range_max = static_cast<float>(max_r_);
    scan.time_increment = 0.0f;
    scan.scan_time = 0.1f;

    const int n = static_cast<int>(
        (scan.angle_max - scan.angle_min) / scan.angle_increment);
    const float kInf = std::numeric_limits<float>::infinity();
    scan.ranges.assign(static_cast<size_t>(n), kInf);

    for (const auto& p : pts) {
      const double r = std::hypot(p.x, p.y);
      const double a = std::atan2(p.y, p.x);
      int idx = static_cast<int>((a - scan.angle_min) / scan.angle_increment);
      if (idx < 0 || idx >= n) continue;
      if (r < scan.ranges[static_cast<size_t>(idx)]) {
        scan.ranges[static_cast<size_t>(idx)] = static_cast<float>(r);
      }
    }
    scan_pub_->publish(scan);
  }

  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const std::vector<Vec2> pts = to_points(*msg);

    // 1) 桩检测 → /piles
    const auto piles = nav::detect_piles(pts, params_);
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (const auto& pl : piles) {
      visualization_msgs::msg::Marker m;
      m.header = msg->header;
      m.ns = "piles";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = pl.center.x;
      m.pose.position.y = pl.center.y;
      m.pose.position.z = 0.5;
      m.pose.orientation.w = 1.0;
      m.scale.x = 2.0 * pl.radius;
      m.scale.y = 2.0 * pl.radius;
      m.scale.z = 1.0;
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      arr.markers.push_back(m);
    }
    piles_pub_->publish(arr);

    // 2) 同一个 2D 点集 → /scan（避障用）
    publish_scan(pts, msg->header);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "2D点=%zu 检测到 %zu 个桩", pts.size(), piles.size());
  }

  double min_h_{0.10}, max_h_{1.50};
  double min_r_{0.30}, max_r_{6.00};
  double voxel_{0.05};
  double tf_x_{0.0}, tf_y_{0.0}, tf_z_{0.0}, tf_yaw_{0.0};

  nav::PileDetectParams params_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr piles_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
