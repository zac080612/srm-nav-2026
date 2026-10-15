#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav/pile_detect.hpp>

#include <cmath>
#include <vector>

using std::placeholders::_1;

using nav::Vec2;

// 订阅 /scan，检测桩，发布 /piles 供 RViz 显示
class PileDetectorNode : public rclcpp::Node {
 public:
  PileDetectorNode() : Node("pile_detector") {
    declare_parameter("pile_radius", 0.08);
    params_.pile_radius = get_parameter("pile_radius").as_double();

    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&PileDetectorNode::on_scan, this, _1));
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/piles", 10);
    RCLCPP_INFO(get_logger(), "pile_detector 启动，订阅 /scan ...");
  }

 private:
  // LaserScan 极坐标转成 2D 点
  std::vector<Vec2> to_points(const sensor_msgs::msg::LaserScan& msg) const {
    std::vector<Vec2> pts;
    pts.reserve(msg.ranges.size());
    for (size_t i = 0; i < msg.ranges.size(); ++i) {
      const float r = msg.ranges[i];
      if (r < msg.range_min || r > msg.range_max) continue;
      const double a = msg.angle_min + i * msg.angle_increment;
      pts.push_back({r * std::cos(a), r * std::sin(a)});
    }
    return pts;
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const auto piles = nav::detect_piles(to_points(*msg), params_);

    visualization_msgs::msg::MarkerArray arr;
    int id = 0;
    for (const auto& c : piles) {
      visualization_msgs::msg::Marker m;
      m.header = msg->header;
      m.ns = "piles";
      m.id = id++;
      m.type = visualization_msgs::msg::Marker::CYLINDER;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.position.x = c.x;
      m.pose.position.y = c.y;
      m.pose.position.z = 0.5;
      m.pose.orientation.w = 1.0;
      m.scale.x = 0.16;
      m.scale.y = 0.16;
      m.scale.z = 1.0;
      m.color.r = 1.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
      arr.markers.push_back(m);
    }
    pub_->publish(arr);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "检测到 %zu 个桩", piles.size());
  }

  nav::PileDetectParams params_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PileDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
