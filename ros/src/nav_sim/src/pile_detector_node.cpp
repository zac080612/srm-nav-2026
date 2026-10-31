// 仿真桩检测：把 Gazebo 的多层水平激光合成「低带/高带」两带，
// 跑与实车感知节点完全相同的 core 算法 detect_piles_structured。
// 实车是 Livox 点云按高度分带，仿真用多层 2D 激光模拟同一件事：
//   低带 z=0.03/0.04（桩半圆截面）→ 圆环验证
//   高带 z=0.45/0.80（细杆截面）→ 细杆簇定位
// 同时把全部层合成 /scan 给避障用（每角度取最近距离）。

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
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

namespace {

// 一层激光的高度带属性
struct Layer {
  std::string topic;
  double z;      // 层高（米）
  int band;      // 0=低带 1=高带 2=中间带（忽略）
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub;
  std::vector<Vec2> pts;   // 本层最近一帧的点（车体系）
  bool fresh{false};
};

// 极坐标 → 2D 点，带距离过滤
std::vector<Vec2> to_points(const sensor_msgs::msg::LaserScan& msg,
                            double min_range, double max_range) {
  std::vector<Vec2> pts;
  pts.reserve(msg.ranges.size());
  for (size_t i = 0; i < msg.ranges.size(); ++i) {
    const float r = msg.ranges[i];
    if (r < min_range || r > max_range) continue;
    const double a = msg.angle_min + i * msg.angle_increment;
    pts.push_back({r * std::cos(a), r * std::sin(a)});
  }
  return pts;
}

// 体素降采样（只给低带用；高带细杆点不做，避免整根杆并成 1 点）
std::vector<Vec2> voxel_filter(const std::vector<Vec2>& pts, double voxel) {
  const double inv = 1.0 / std::max(voxel, 1e-3);
  std::unordered_set<int64_t> seen;
  std::vector<Vec2> out;
  for (const auto& p : pts) {
    const int gx = static_cast<int>(std::floor(p.x * inv));
    const int gy = static_cast<int>(std::floor(p.y * inv));
    const int64_t key = (static_cast<int64_t>(gx) << 32) ^
                        static_cast<int64_t>(static_cast<uint32_t>(gy));
    if (!seen.insert(key).second) continue;
    out.push_back(p);
  }
  return out;
}

}  // namespace

class PileDetectorNode : public rclcpp::Node {
 public:
  PileDetectorNode() : Node("pile_detector") {
    // 与实车感知节点相同的识别参数
    declare_parameter("min_radius", 0.04);
    declare_parameter("max_radius", 0.11);
    declare_parameter("min_points", 3);
    declare_parameter("cluster_dist", 0.20);
    declare_parameter("max_fit_rms", 0.08);
    declare_parameter("high_match_r", 0.10);
    declare_parameter("high_min_pts", 2);
    declare_parameter("high_spread_max", 0.03);
    declare_parameter("min_range", 0.30);
    declare_parameter("max_range", 4.0);
    declare_parameter("voxel", 0.005);

    params_.min_radius = get_parameter("min_radius").as_double();
    params_.max_radius = get_parameter("max_radius").as_double();
    params_.min_points = static_cast<size_t>(get_parameter("min_points").as_int());
    params_.cluster_dist = get_parameter("cluster_dist").as_double();
    params_.max_fit_rms = get_parameter("max_fit_rms").as_double();
    params_.min_arc_deg = 0.0;
    struct_.high_match_r = get_parameter("high_match_r").as_double();
    struct_.high_min_pts = get_parameter("high_min_pts").as_int();
    struct_.high_spread_max = get_parameter("high_spread_max").as_double();
    min_range_ = get_parameter("min_range").as_double();
    max_range_ = get_parameter("max_range").as_double();
    voxel_ = get_parameter("voxel").as_double();

    // 5 层水平激光：低带 2 层、中间 1 层、高带 2 层。
    // 层高对应桩的截面：0.03/0.04 打到半球（截面半径约 0.09），
    // 0.45/0.80 打到细杆（截面半径 0.01）。
    layers_.push_back({"/scan_low1", 0.03, 0, nullptr, {}, false});
    layers_.push_back({"/scan_low2", 0.04, 0, nullptr, {}, false});
    layers_.push_back({"/scan_mid", 0.25, 2, nullptr, {}, false});
    layers_.push_back({"/scan_high1", 0.45, 1, nullptr, {}, false});
    layers_.push_back({"/scan_high2", 0.80, 1, nullptr, {}, false});
    for (auto& L : layers_) {
      L.sub = create_subscription<sensor_msgs::msg::LaserScan>(
          L.topic, 10, [this, &L](const sensor_msgs::msg::LaserScan::SharedPtr m) {
            L.pts = to_points(*m, min_range_, max_range_);
            L.fresh = true;
          });
    }

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/piles", 10);
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               std::bind(&PileDetectorNode::on_timer, this));
    RCLCPP_INFO(get_logger(), "pile_detector 启动，订阅 5 层 /scan_* → /piles + /scan");
  }

 private:
  void on_timer() {
    bool all_fresh = true;
    for (const auto& L : layers_) all_fresh = all_fresh && L.fresh;
    if (!all_fresh) return;

    std::vector<Vec2> low, high, all;
    for (auto& L : layers_) {
      if (L.band == 0) {
        auto p = voxel_filter(L.pts, voxel_);
        low.insert(low.end(), p.begin(), p.end());
      } else if (L.band == 1) {
        high.insert(high.end(), L.pts.begin(), L.pts.end());   // 细杆不降采样
      }
      all.insert(all.end(), L.pts.begin(), L.pts.end());
    }

    // 结构化桩检测（与实车相同的 core 算法）
    const auto piles = nav::detect_piles_structured(low, high, params_, struct_);

    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (const auto& pl : piles) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = "base_link";
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
    pub_->publish(arr);
    publish_scan(all);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "低带=%zu 高带=%zu 检测到 %zu 个桩",
                         low.size(), high.size(), piles.size());
  }

  // 全部层合成 /scan：每角度取最近距离（最保守，避障不放过任何东西）
  void publish_scan(const std::vector<Vec2>& pts) {
    sensor_msgs::msg::LaserScan scan;
    scan.header.frame_id = "base_link";
    scan.angle_min = static_cast<float>(-M_PI);
    scan.angle_max = static_cast<float>(M_PI);
    scan.angle_increment = static_cast<float>(0.5 * M_PI / 180.0);   // 0.5 度
    scan.range_min = static_cast<float>(min_range_);
    scan.range_max = static_cast<float>(max_range_);
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

  nav::PileDetectParams params_;
  nav::PileStructureParams struct_;
  double min_range_{0.30}, max_range_{4.0}, voxel_{0.005};
  std::vector<Layer> layers_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PileDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
