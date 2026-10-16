#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <nav/control.hpp>
#include <nav/frame.hpp>
#include <nav/rbc_map.hpp>
#include <nav/safe_cmd.hpp>
#include <nav/state_machine.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

using std::placeholders::_1;

using nav::Mat2;
using nav::OrbitParams;
using nav::ProtoCmd;
using nav::SafeCmd;
using nav::SmInput;
using nav::State;
using nav::StateMachine;
using nav::UpFrame;
using nav::Vec2;
using nav::map_body_to_protocol;
using nav::orbit_step;

namespace {

double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

// 四元数 → yaw（车只在 X-Y 平面运动）
double yaw_of(double qx, double qy, double qz, double qw) {
  return std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
}

}  // namespace

class Figure8Node : public rclcpp::Node {
 public:
  Figure8Node() : Node("figure8") {
    declare_parameter("park_x", -1.5);
    declare_parameter("park_y", -2.5);
    declare_parameter("v_lin", 0.20);
    declare_parameter("a_max", 0.20);

    park_ = {get_parameter("park_x").as_double(),
             get_parameter("park_y").as_double()};
    v_lin_ = get_parameter("v_lin").as_double();
    a_max_ = get_parameter("a_max").as_double();

    down_.ccw = true;
    up_.ccw = false;
    down_.v_t = 0.08;
    up_.v_t = 0.08;

    rbc_.m[0][0] = 0.0; rbc_.m[0][1] = 1.0;
    rbc_.m[1][0] = -1.0; rbc_.m[1][1] = 0.0;

    safe_.set_limit(0.2f);
    safe_.set_watchdog_ms(500);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&Figure8Node::on_odom, this, _1));
    sub_piles_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/piles", 10, std::bind(&Figure8Node::on_piles, this, _1));
    pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/raw_frame", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               std::bind(&Figure8Node::on_timer, this));

    RCLCPP_INFO(get_logger(), "figure8 启动，等待 /odom 与 /piles ...");
  }

 private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    car_x_ = msg->pose.pose.position.x;
    car_y_ = msg->pose.pose.position.y;
    const auto& q = msg->pose.pose.orientation;
    car_yaw_ = yaw_of(q.x, q.y, q.z, q.w);
    have_pose_ = true;
  }

  // 桩位置转世界坐标，按世界 y 区分下桩/上桩
  void on_piles(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    if (msg->markers.size() < 2) {
      have_piles_ = false;
      return;
    }
    const double c = std::cos(car_yaw_);
    const double s = std::sin(car_yaw_);
    std::vector<Vec2> world;
    for (const auto& m : msg->markers) {
      const double bx = m.pose.position.x;
      const double by = m.pose.position.y;
      world.push_back({car_x_ + c * bx - s * by, car_y_ + s * bx + c * by});
    }
    std::sort(world.begin(), world.end(),
              [](const Vec2& a, const Vec2& b) { return a.y < b.y; });
    b_down_ = to_body(world.front().x, world.front().y);
    b_up_ = to_body(world.back().x, world.back().y);
    have_piles_ = true;
  }

  // 世界坐标 → 车体系
  Vec2 to_body(double wx, double wy) const {
    const double c = std::cos(-car_yaw_);
    const double s = std::sin(-car_yaw_);
    const double dx = wx - car_x_;
    const double dy = wy - car_y_;
    return {c * dx - s * dy, s * dx + c * dy};
  }

  Vec2 unit_vec(const Vec2& v) const {
    const double r = std::hypot(v.x, v.y);
    if (r < 1e-6) return {0.0, 0.0};
    return {v.x / r, v.y / r};
  }

  Vec2 scale(const Vec2& v, double k) const { return {v.x * k, v.y * k}; }

  // 累积绕行角
  void update_accum_angle(const Vec2& b_near, bool near_is_up, bool fresh) {
    const double bearing = std::atan2(-b_near.y, -b_near.x);
    if (fresh || near_is_up != last_near_up_) {
      last_near_up_ = near_is_up;
      accum_angle_ = 0.0;
      last_bearing_ = bearing;
      return;
    }
    accum_angle_ += std::fabs(wrap_pi(bearing - last_bearing_));
    last_bearing_ = bearing;
  }

  // 当前状态的目标速度（直行用 v_lin，绕桩用 v_t）
  double target_speed(State s) const {
    switch (s) {
      case State::OrbitDown:
      case State::OrbitUp:      return down_.v_t;
      case State::StraightIn:
      case State::Cross:
      case State::StraightOut:  return v_lin_;
      default:                  return 0.0;
    }
  }

  // 按状态计算目标车体系速度（实际速度经斜坡后传入）
  Vec2 target_velocity(State s, const Vec2& b_near, const Vec2& b_cross,
                       const Vec2& b_park, double v_now) const {
    switch (s) {
      case State::StraightIn:  return scale(unit_vec(b_near), v_now);
      case State::OrbitDown: {
        OrbitParams p = down_;
        p.v_t = v_now;
        return orbit_step(b_near, p);
      }
      case State::Cross:       return scale(unit_vec(b_cross), v_now);
      case State::OrbitUp: {
        OrbitParams p = up_;
        p.v_t = v_now;
        return orbit_step(b_near, p);
      }
      case State::StraightOut: return scale(unit_vec(b_park), v_now);
      default:                 return {0.0, 0.0};
    }
  }

  void on_timer() {
    if (!have_pose_) return;

    const Vec2 b_cross = {(b_down_.x + b_up_.x) * 0.5,
                          (b_down_.y + b_up_.y) * 0.5};
    const Vec2 b_park = to_body(park_.x, park_.y);

    const State cur = sm_.state();
    const bool near_is_up = (cur == State::OrbitUp);
    const Vec2 b_near = near_is_up ? b_up_ : b_down_;

    const bool orbiting = (cur == State::OrbitDown || cur == State::OrbitUp);
    if (orbiting) {
      update_accum_angle(b_near, near_is_up, !was_orbiting_);
    }
    was_orbiting_ = orbiting;

    SmInput in;
    in.started = true;
    in.have_piles = have_piles_;
    in.b_near = b_near;
    in.b_far = near_is_up ? b_down_ : b_up_;
    in.accum_angle = accum_angle_;
    in.dist_to_cross = std::hypot(b_cross.x, b_cross.y);
    in.dist_to_park = std::hypot(b_park.x, b_park.y);
    sm_.step(in);

    // 斜坡：实际速度每帧向目标速度逼近 a_max*dt，避免切换时跳变
    const double v_target = target_speed(sm_.state());
    const auto now = this->now();
    const double dt_raw = last_time_.nanoseconds() > 0
                              ? (now - last_time_).seconds()
                              : 0.0;
    last_time_ = now;
    const double dt = std::min(dt_raw, 0.1);  // 卡顿后最多按 0.1s 推进，防止突变
    if (v_now_ < v_target) {
      v_now_ = std::min(v_target, v_now_ + a_max_ * dt);
    } else if (v_now_ > v_target) {
      v_now_ = std::max(v_target, v_now_ - a_max_ * dt);
    }

    const Vec2 v = target_velocity(sm_.state(), b_near, b_cross, b_park, v_now_);
    const ProtoCmd c = map_body_to_protocol(rbc_, v.x, v.y);
    safe_.update(c.vx, c.vy);
    const UpFrame frame = safe_.tick(seq_++);

    std_msgs::msg::UInt8MultiArray msg;
    msg.data.assign(frame.begin(), frame.end());
    pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "state=%s v_now=%.3f vx=%.3f vy=%.3f",
                         nav::to_string(sm_.state()), v_now_, c.vx, c.vy);
  }

  Vec2 park_;
  double v_lin_{0.20};
  double a_max_{0.20};
  OrbitParams down_, up_;

  double v_now_{0.0};
  rclcpp::Time last_time_;

  double car_x_{0.0}, car_y_{0.0}, car_yaw_{0.0};
  bool have_pose_{false};

  Vec2 b_down_{0.0, 0.0};
  Vec2 b_up_{0.0, 0.0};
  bool have_piles_{false};

  double accum_angle_{0.0};
  double last_bearing_{0.0};
  bool last_near_up_{false};
  bool was_orbiting_{false};

  Mat2 rbc_;
  SafeCmd safe_;
  StateMachine sm_;
  uint8_t seq_{0};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_piles_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Figure8Node>());
  rclcpp::shutdown();
  return 0;
}
