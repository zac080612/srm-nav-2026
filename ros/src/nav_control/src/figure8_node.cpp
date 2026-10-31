#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav/control.hpp>
#include <nav/frame.hpp>
#include <nav/pile_frame.hpp>
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
using nav::car_in_pile_frame;
using nav::map_body_to_protocol;
using nav::orbit_step;
using nav::to_park_body;

namespace {

constexpr double kCarRadius = 0.27;  // 车体半径（URDF）
constexpr double kMargin = 0.20;     // 绕桩安全余量
constexpr double kSafeDist = 0.2;   // 安全净空：车表面到障碍表面留 20cm（不含车/桩半径，已在 clearance 里扣掉）
constexpr double kAvoidRange = 1.2;  // 只避这个距离内的障碍：远处杂物先不管（早避只会左右摆），近了再反应
constexpr double kAvoidHyst = 0.10;     // 退出避障的迟滞带（米）：净空要超过 kSafeDist+kAvoidHyst 才恢复
constexpr double kAvoidHystAng = 0.25;  // 方向搜索时"当前绕行方向"的代价优惠，防止在避人/避桩之间来回跳
constexpr double kPileExclude = 0.35;
constexpr double kAvoidSpeed = 0.5;
constexpr int kStopFrames = 10;
// 贴桩滑行阈值：桩的净空低于它才开始罚，≥ 它时允许沿桩面滑过去（不弹回）
constexpr double kPileBrushDist = 0.08;
// 净空计算的对象集合。人和桩分开算，因为惩罚策略不同：
// 人是硬底线（净空不足就罚），桩允许贴边滑（只有真会撞才重罚）。
enum class ObstacleSet { kHuman, kPile };
constexpr double kPileMatchDist = 0.5;     // 桩匹配阈值：离锁定坐标超过它就当成障碍
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
enum class AvoidState { Normal, Stop, Avoid };
class Figure8Node : public rclcpp::Node {
 public:
  Figure8Node() : Node("figure8") {
    declare_parameter("park_x", -1.5);
    declare_parameter("park_y", -2.5);
    declare_parameter("v_lin", 0.4);
    declare_parameter("a_max", 1.0);
    declare_parameter("v_t", 0.4);
    declare_parameter("k_r", 0.8);
    declare_parameter("v_max_ms", 0.4);
    declare_parameter("auto_start", false);
    declare_parameter("speed_limit", 2.0);
    // 量纲换算：控制律内部按 m/s 规划轨迹，而协议字段是「与底盘速度成正比的数值」，
    // 不是 m/s（协议 3.1/3.3，参考比例 10000:1，满杆约 39600）。
    // speed_scale = 协议数值 ÷ 实际速度(m/s)，实车量出来后填，仿真保持 1.0。
    declare_parameter("speed_scale", 1.0);
    // R_bc 轴映射矩阵（车体系速度 → 协议 vx/vy）。默认是协议字面直译的 90° 交叉，
    // 实车必须按实测标定结果改这四个参数，见计划书问题 1 / 核查文档 4.6。
    declare_parameter("r11", 0.0);
    declare_parameter("r12", 1.0);
    declare_parameter("r21", -1.0);
    declare_parameter("r22", 0.0);

    park_ = {get_parameter("park_x").as_double(),
             get_parameter("park_y").as_double()};
    v_lin_ = get_parameter("v_lin").as_double();
    a_max_ = get_parameter("a_max").as_double();
    auto_start_ = get_parameter("auto_start").as_bool();
    speed_scale_ = get_parameter("speed_scale").as_double();
    v_max_ms_ = get_parameter("v_max_ms").as_double();

    down_.ccw = true;
    up_.ccw = false;
    down_.v_t = get_parameter("v_t").as_double();
    up_.v_t = get_parameter("v_t").as_double();
    down_.k_r = get_parameter("k_r").as_double();
    up_.k_r = get_parameter("k_r").as_double();

    rbc_.m[0][0] = get_parameter("r11").as_double();
    rbc_.m[0][1] = get_parameter("r12").as_double();
    rbc_.m[1][0] = get_parameter("r21").as_double();
    rbc_.m[1][1] = get_parameter("r22").as_double();

    safe_.set_limit(static_cast<float>(get_parameter("speed_limit").as_double()));
    safe_.set_watchdog_ms(500);

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&Figure8Node::on_odom, this, _1));
    sub_piles_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/piles", 10, std::bind(&Figure8Node::on_piles, this, _1));
    sub_progress_ = create_subscription<std_msgs::msg::UInt8>(
        "/game_progress", 10, std::bind(&Figure8Node::on_progress, this, _1));
    sub_scan_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Figure8Node::on_scan, this, _1));
    pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/raw_frame", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(20),
                               std::bind(&Figure8Node::on_timer, this));

    RCLCPP_INFO(get_logger(), "figure8 启动，等待 /odom 与 /piles ...");
    RCLCPP_INFO(get_logger(), "速度：v_lin=%.2f v_t=%.2f 真实上限=%.2f m/s 协议限幅=%.2f 缩放=%.2f",
                v_lin_, down_.v_t, v_max_ms_, get_parameter("speed_limit").as_double(),
                speed_scale_);
  }

 private:
  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
    car_x_ = msg->pose.pose.position.x;
    car_y_ = msg->pose.pose.position.y;
    const auto& q = msg->pose.pose.orientation;
    car_yaw_ = yaw_of(q.x, q.y, q.z, q.w);
    have_pose_ = true;
  }

  // 比赛阶段信号：serial_node 从下行反馈帧解析出 game_progress 发布到这里。
  // 收到非零阶段即认为"比赛开始"，置 started_。具体阶段值待上车确认（计划书 Q6）。
  void on_progress(const std_msgs::msg::UInt8::SharedPtr msg) {
    if (msg->data != 0) {
      started_ = true;
      RCLCPP_INFO(get_logger(), "收到比赛开始信号 game_progress=%u", msg->data);
    }
  }

  // 处理桩检测结果。桩在场地里固定不动，所以锁「世界坐标」最可靠：
  // 处理桩检测结果。检测结果已是车体系坐标（perception 做了 tf 变换），
  // 不依赖 odom：启动时按车体 y 分上下锁定，之后按"短时连续性"匹配
  //（20ms 一帧车最多动几毫米，桩的车体系坐标几乎不变）。
  void on_piles(const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
    struct B { Vec2 pos; double radius; };
    std::vector<B> body;
    for (const auto& m : msg->markers) {
      body.push_back({{m.pose.position.x, m.pose.position.y}, m.scale.x * 0.5});
    }

    // 启动时锁定两个真桩的车体系坐标（按车体 y 分上下）
    if (!piles_locked_) {
      if (body.size() < 2) {
        have_piles_ = false;
        return;
      }
      std::sort(body.begin(), body.end(),
                [](const B& a, const B& b) { return a.pos.y < b.pos.y; });
      b_down_ = body.front().pos;
      b_up_ = body.back().pos;
      down_r_ = body.front().radius;
      up_r_ = body.back().radius;
      piles_locked_ = true;
      RCLCPP_INFO(get_logger(), "已锁定桩（车体系）：下(%.2f,%.2f,r=%.2f) 上(%.2f,%.2f,r=%.2f)",
                  b_down_.x, b_down_.y, down_r_,
                  b_up_.x, b_up_.y, up_r_);
    }

    // 每帧匹配：离锁定位置近的候选是真桩，远的忽略（杂物）
    Vec2 down_b = b_down_;   // 默认锁定值（漏检时补上）
    Vec2 up_b = b_up_;
    for (const auto& b : body) {
      const double dd = std::hypot(b.pos.x - b_down_.x, b.pos.y - b_down_.y);
      const double du = std::hypot(b.pos.x - b_up_.x, b.pos.y - b_up_.y);
      if (dd < kPileMatchDist && dd < du) {
        down_b = b.pos; down_r_ = b.radius;
      } else if (du < kPileMatchDist) {
        up_b = b.pos; up_r_ = b.radius;
      }
      // 离两个锁定位置都远 → 杂物，忽略
    }
    b_down_ = down_b;
    b_up_ = up_b;
    down_.R = down_r_ + kCarRadius + kMargin;
    up_.R = up_r_ + kCarRadius + kMargin;
    have_piles_ = true;

    // 锁定后标定一次停车位桩系坐标（车此时在停车位）
    if (!park_calibrated_) {
      p_park_ = car_in_pile_frame(b_down_, b_up_);
      park_calibrated_ = true;
      RCLCPP_INFO(get_logger(), "已标定停车位（桩系）: (%.3f, %.3f)",
                  p_park_.x, p_park_.y);
    }
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
  // 收到 /scan：点转车体系坐标，排除桩附近的点，剩下的当作障碍（人）。
  // 桩不放进障碍点：正常路线本来就会经过桩附近，桩触发停车会误停。
  // 桩只在"代价法选绕行方向"时单独考虑（见 pick_avoid_direction）。
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    obstacle_pts_.clear();
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (r < msg->range_min || r > msg->range_max) continue;
      const double a = msg->angle_min + i * msg->angle_increment;
      Vec2 p{r * std::cos(a), r * std::sin(a)};
      if (std::hypot(p.x - b_down_.x, p.y - b_down_.y) < kPileExclude) continue;
      if (std::hypot(p.x - b_up_.x, p.y - b_up_.y) < kPileExclude) continue;
      obstacle_pts_.push_back(p);
    }
  }

  // 车沿方向 a（弧度）直线行驶时，到最近障碍表面的"净空距离"。
  // 用几何碰撞检测：障碍圆心到行驶直线的垂直距离 = |p×方向|，
  // 减去车半径和障碍半径就是净空。净空 <= 0 表示会撞。
  // which 决定算谁：kHuman 只看人（判断是否触发避障），kPile 只看桩（方向搜索）。
  double clearance_in_dir(double a, ObstacleSet which) const {
    const double dirx = std::cos(a), diry = std::sin(a);
    double min_clear = 1e9;
    if (which == ObstacleSet::kHuman) {
      // 障碍点（人）：点本身在人表面附近，保守加 0.1 余量
      for (const auto& p : obstacle_pts_) {
        if (std::hypot(p.x, p.y) > kAvoidRange) continue;   // 远障碍不躲
        const double proj = p.x * dirx + p.y * diry;   // 沿方向的投影，<0 在车后方
        if (proj < 0) continue;
        const double lat = std::fabs(p.x * diry - p.y * dirx);  // 到行驶直线的垂直距离
        const double clear = lat - kCarRadius - 0.1;
        min_clear = std::min(min_clear, clear);
      }
    } else {
      const Vec2 piles[2] = {b_down_, b_up_};
      const double pr[2] = {down_r_, up_r_};
      for (int i = 0; i < 2; ++i) {
        const double proj = piles[i].x * dirx + piles[i].y * diry;
        if (proj < 0) continue;
        const double lat = std::fabs(piles[i].x * diry - piles[i].y * dirx);
        const double clear = lat - kCarRadius - pr[i];
        min_clear = std::min(min_clear, clear);
      }
    }
    return min_clear;
  }

  // 避障仲裁：输入原目标速度，输出避障后的速度。
  // 思路：原目标方向被「人」挡时，用方向搜索找一条"净空足够 + 偏离小"的方向，
  // 并对方向做平滑（避免一顿一顿）。直行段和绕桩段共用这套逻辑：
  // 绕圈时 target_ang 取切向（v_orig 方向）。桩允许贴边滑（只罚真撞），
  // 车在人和桩之间会沿桩面滑过去，而不是在两个惩罚之间来回弹。
  Vec2 avoid_arbitrate(const Vec2& v_orig) {
    const State st = sm_.state();

    // 只有移动状态才避障；Idle/Stop 等静止状态原样返回
    const bool moving = (st == State::StraightIn || st == State::Cross ||
                         st == State::StraightOut || st == State::OrbitDown ||
                         st == State::OrbitUp);
    if (!moving) {
      avoid_state_ = AvoidState::Normal;
      avoid_ang_ = 0.0;
      return v_orig;
    }

    // 目标方向（单位向量）及其角度
    const double r = std::hypot(v_orig.x, v_orig.y);
    const Vec2 dir = (r < 1e-6) ? Vec2{1.0, 0.0} : Vec2{v_orig.x / r, v_orig.y / r};
    const double target_ang = std::atan2(dir.y, dir.x);

    // 原目标方向是否畅通（只看人，桩不触发避障）。
    // 退出带迟滞：净空在阈值附近时避免 Normal/Stop 快速切换导致左右摆。
    const double exit_req =
        kSafeDist + ((avoid_state_ == AvoidState::Normal) ? 0.0 : kAvoidHyst);
    if (clearance_in_dir(target_ang, ObstacleSet::kHuman) >= exit_req) {
      avoid_state_ = AvoidState::Normal;
      avoid_ang_ = 0.0;
      return v_orig;
    }

    // 被挡：进入避障，先停车
    if (avoid_state_ == AvoidState::Normal) {
      avoid_state_ = AvoidState::Stop;
      avoid_stop_count_ = 0;
      // 诊断：打印最近障碍的距离和方向（杂物排查用）
      double min_d = 1e9;
      Vec2 min_p{0.0, 0.0};
      for (const auto& p : obstacle_pts_) {
        const double d = std::hypot(p.x, p.y);
        if (d < min_d) { min_d = d; min_p = p; }
      }
      RCLCPP_INFO(get_logger(),
                  "避障触发: 障碍点=%zu 最近(%.2f,%.2f)=%.2fm 目标角=%.0f°",
                  obstacle_pts_.size(), min_p.x, min_p.y, min_d,
                  target_ang * 180.0 / M_PI);
      return {0.0, 0.0};
    }
    if (avoid_state_ == AvoidState::Stop) {
      if (++avoid_stop_count_ >= kStopFrames) {
        avoid_state_ = AvoidState::Avoid;
        avoid_ang_ = target_ang;
      }
      return {0.0, 0.0};
    }

    // Avoid：方向搜索——找"净空足够 + 偏离目标小"的最佳方向
    double best_ang = target_ang;
    double best_cost = 1e9;
    for (int deg : {-90, -60, -30, -15, 0, 15, 30, 60, 90}) {
      const double a = target_ang + deg * M_PI / 180.0;
      // 人：硬底线，净空不足按比例罚
      const double clear_h = clearance_in_dir(a, ObstacleSet::kHuman);
      double cost;
      if (clear_h <= 0.05) {
        cost = 3.0;                                // 会蹭/会撞人，重罚
      } else if (clear_h >= kSafeDist) {
        cost = 0.0;                                // 足够安全
      } else {
        cost = 1.0 - clear_h / kSafeDist;          // 净空不足，按比例罚
      }
      // 桩：贴边不罚，允许沿桩面滑过去；只有真会撞才重罚（滑开，而不是弹回）
      const double clear_p = clearance_in_dir(a, ObstacleSet::kPile);
      if (clear_p <= 0.0) {
        cost += 3.0;
      } else if (clear_p < kPileBrushDist) {
        cost += 1.5;
      }
      const double dev = std::fabs(deg) / 90.0;            // 偏离目标角度（0~1）
      cost += 0.6 * dev;
      // 迟滞：离当前绕行方向越近优惠越多，避免在"避人"和"避桩"两个方向间来回跳
      if (avoid_state_ == AvoidState::Avoid) {
        const double d = std::fabs(wrap_pi(a - avoid_ang_));
        cost -= kAvoidHystAng * std::max(0.0, 1.0 - d / (M_PI / 2.0));
      }
      if (cost < best_cost) {
        best_cost = cost;
        best_ang = a;
      }
    }

    // 方向平滑：avoid_ang_ 向 best_ang 逼近，限制每帧最多转 20°，避免一顿一顿
    double diff = wrap_pi(best_ang - avoid_ang_);
    const double max_step = 20.0 * M_PI / 180.0;
    if (diff > max_step) diff = max_step;
    if (diff < -max_step) diff = -max_step;
    avoid_ang_ += diff;

    return scale({std::cos(avoid_ang_), std::sin(avoid_ang_)}, kAvoidSpeed);
  }
  void on_timer() {
    // 不依赖 odom：桩锁定（车体系）之后就能跑，桩系定位不需要里程计
    if (!have_piles_) return;

    const Vec2 b_cross = {(b_down_.x + b_up_.x) * 0.5,
                          (b_down_.y + b_up_.y) * 0.5};
    // 返程目标：停车位在桩系的坐标（启动时已标定），转成车体系方向。
    // 桩系定位不需要里程计，两桩在就可靠。
    const Vec2 b_park = to_park_body(b_down_, b_up_, p_park_);
    const double dist_to_park = std::hypot(b_park.x, b_park.y);

    const State cur = sm_.state();
    const bool near_is_up = (cur == State::OrbitUp);
    const Vec2 b_near = near_is_up ? b_up_ : b_down_;
    const Vec2 b_far = near_is_up ? b_down_ : b_up_;

    const bool orbiting = (cur == State::OrbitDown || cur == State::OrbitUp);
    const bool avoiding = (avoid_state_ != AvoidState::Normal);
    if (orbiting && !avoiding) {
      // 避障期间先不累计（车偏离圆，bearing 每帧变化不代表纯绕桩进度）；
      // 避障结束恢复累计时，bearing 相对避障前的净变化就是这段的真实绕行角，
      // 不重置——否则障碍堵在绕圈路上时角度永远攒不够，状态机死锁在绕圈。
      update_accum_angle(b_near, near_is_up, !was_orbiting_);
    }
    was_orbiting_ = orbiting;

    SmInput in;
    in.started = auto_start_ || started_;
    in.have_piles = have_piles_;
    in.b_near = b_near;
    in.b_far = b_far;
    in.accum_angle = accum_angle_;
    in.dist_to_cross = std::hypot(b_cross.x, b_cross.y);
    in.dist_to_park = dist_to_park;
    in.orbit_radius = near_is_up ? up_.R : down_.R;
    // 避障激活期间暂停状态机推进，只为保护 StraightOut：绕行路径可能擦过停车位，
    // 若照常推进会误切 Stop 停错位置。其余状态照常推进——障碍挡路时车必须能按
    // 几何条件切状态，否则状态机冻结：StraightIn 永远到不了桩、绕圈绕完也没法切。
    if (avoid_state_ == AvoidState::Normal || sm_.state() != State::StraightOut) {
      sm_.step(in);
    }

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
    const Vec2 v_avoid = avoid_arbitrate(v);
    // 真实速度上限（m/s）：先在这里卡住，保证 v_lin/v_t 要多少就是多少，
    // 不会因为换算成协议值后又撞上 speed_limit 而被打折。
    double vx_ms = v_avoid.x;
    double vy_ms = v_avoid.y;
    const double vmag = std::hypot(vx_ms, vy_ms);
    if (vmag > v_max_ms_ && vmag > 1e-9) {
      const double k = v_max_ms_ / vmag;
      vx_ms *= k;
      vy_ms *= k;
    }
    // m/s → 协议数值：协议 3.1 明确 vx/vy 只是「与底盘速度成正比的指令」
    const ProtoCmd c = map_body_to_protocol(rbc_, vx_ms * speed_scale_,
                                            vy_ms * speed_scale_);
    safe_.update(c.vx, c.vy);
    const UpFrame frame = safe_.tick(seq_++);

    std_msgs::msg::UInt8MultiArray msg;
    msg.data.assign(frame.begin(), frame.end());
    pub_->publish(msg);

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "state=%s v_now=%.3f vx=%.3f vy=%.3f 桩距=%.2f 绕角=%.0f° 避障=%d",
                         nav::to_string(sm_.state()), v_now_, c.vx, c.vy,
                         std::hypot(b_near.x, b_near.y), accum_angle_ * 180.0 / M_PI,
                         static_cast<int>(avoid_state_));
  }

  Vec2 park_;
  Vec2 p_park_{0.0, 0.0};       // 停车位在桩系的坐标（启动时标定）
  bool park_calibrated_{false}; // 是否已标定停车位
  double v_lin_{0.4};
  double a_max_{1.0};
  double v_max_ms_{0.4};
  double speed_scale_{1.0};
  OrbitParams down_, up_;

  double v_now_{0.0};
  rclcpp::Time last_time_;

  double car_x_{0.0}, car_y_{0.0}, car_yaw_{0.0};
  bool have_pose_{false};
  bool started_{false};
  bool auto_start_{false};

  Vec2 b_down_{0.0, 0.0};
  Vec2 b_up_{0.0, 0.0};
  bool have_piles_{false};

  Vec2 pile_down_w_{0.0, 0.0};  // 下桩世界坐标（启动时锁定，固定不动）
  Vec2 pile_up_w_{0.0, 0.0};    // 上桩世界坐标（启动时锁定，固定不动）
  bool piles_locked_{false};    // 是否已锁定桩世界坐标
  double down_r_{0.08};         // 下桩检测半径
  double up_r_{0.08};           // 上桩检测半径

  AvoidState avoid_state_{AvoidState::Normal};
  int avoid_stop_count_{0};
  double avoid_ang_{0.0};        // 当前绕行方向角（平滑后）
  std::vector<Vec2> obstacle_pts_;


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
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr sub_progress_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_scan_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Figure8Node>());
  rclcpp::shutdown();
  return 0;
}
