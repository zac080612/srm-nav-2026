#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <nav/crc.hpp>
#include <nav/frame.hpp>

#include <chrono>
#include <cstring>

using std::placeholders::_1;

namespace {

float read_f32_le(const uint8_t* p) {
  const uint32_t u = static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
                     (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
  float v = 0.0f;
  std::memcpy(&v, &u, sizeof(v));
  return v;
}

// 取当前时间（秒）
double steady_now_s() {
  const auto tp = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(tp).count();
}

constexpr double kLostTimeoutS = 0.2;

}  // namespace

// 假下位机：解析 19 字节帧，换算成底盘速度发给 Gazebo
class FakeMcuNode : public rclcpp::Node {
 public:
  FakeMcuNode() : Node("fake_mcu") {
    sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/raw_frame", 10, std::bind(&FakeMcuNode::on_frame, this, _1));
    pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(50),
                               std::bind(&FakeMcuNode::on_timer, this));
    last_frame_s_ = steady_now_s();
    RCLCPP_INFO(get_logger(), "fake_mcu 启动，订阅 /raw_frame ...");
  }

 private:
  // 解析 19 字节帧，换算成底盘速度
  void on_frame(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    const auto& b = msg->data;
    if (b.size() != nav::kUpFrameLen) { ++bad_; return; }
    if (b[0] != 0xA5) { ++bad_; return; }
    if (nav::read_u16_le(&b[1]) != nav::kUpDataLen) { ++bad_; return; }
    if (nav::crc8(b.data(), 4) != b[4]) { ++bad_; return; }
    if (nav::crc16(b.data(), 17) != nav::read_u16_le(&b[17])) { ++bad_; return; }

    const float vx = read_f32_le(&b[5]);
    const float vy = read_f32_le(&b[9]);

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = -static_cast<double>(vy);
    cmd.linear.y = static_cast<double>(vx);
    cmd.angular.z = 0.0;
    pub_->publish(cmd);

    last_frame_s_ = steady_now_s();
    if (lost_) {
      lost_ = false;
      RCLCPP_INFO(get_logger(), "重新收到帧，恢复控制");
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "body=(%.3f, %.3f)  坏帧=%zu",
                         cmd.linear.x, cmd.linear.y, bad_);
  }

  // 超时发零速
  void on_timer() {
    if (steady_now_s() - last_frame_s_ <= kLostTimeoutS) return;

    if (!lost_) {
      lost_ = true;
      RCLCPP_WARN(get_logger(), "超过 %.0f ms 没收到帧 → 发零速（失联保护）",
                  kLostTimeoutS * 1000.0);
    }
    pub_->publish(geometry_msgs::msg::Twist{});
  }

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  double last_frame_s_{0.0};
  bool lost_{false};
  size_t bad_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeMcuNode>());
  rclcpp::shutdown();
  return 0;
}
