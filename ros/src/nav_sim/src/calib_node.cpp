// R_bc 轴映射标定节点：发小速度脉冲，用两帧点云质心差测车实际运动方向，反解出协议轴映射矩阵
// 用法：串口场景先起 serial_node（本节点通过 /raw_frame 发帧），再跑本节点；全程车会移动约 25cm，场地要空

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include <nav/calib.hpp>
#include <nav/frame.hpp>
#include <nav/rbc_map.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_set>
#include <vector>

using std::placeholders::_1;
using nav::Vec2;

namespace {

constexpr int kTickMs = 20;            // 定时器周期
constexpr size_t kMinCloudPoints = 30; // 少于此点数认为点云无效
constexpr int kMaxStageMs = 8000;      // 单个阶段的超时保护

// 把角度量化到最近的 90 度倍数
int quantize_quarter(double angle) {
  return static_cast<int>(std::lround(angle / (M_PI / 2.0)));
}

// 度
double to_deg(double rad) { return rad * 180.0 / M_PI; }

// 点集质心
Vec2 centroid_of(const std::vector<Vec2>& pts) {
  if (pts.empty()) return Vec2{0.0, 0.0};
  double sx = 0.0;
  double sy = 0.0;
  for (const auto& p : pts) {
    sx += p.x;
    sy += p.y;
  }
  const double n = static_cast<double>(pts.size());
  return Vec2{sx / n, sy / n};
}

}  // namespace

class CalibNode : public rclcpp::Node {
 public:
  CalibNode() : Node("calib") {
    declare_parameter("cloud_topic", "/livox/lidar");
    declare_parameter("min_height", 0.10);
    declare_parameter("max_height", 1.50);
    declare_parameter("min_range", 0.30);
    declare_parameter("max_range", 6.00);
    declare_parameter("voxel", 0.05);
    declare_parameter("tf_x", 0.0);
    declare_parameter("tf_y", 0.0);
    declare_parameter("tf_z", 0.0);
    declare_parameter("tf_yaw", 0.0);
    declare_parameter("v_test", 0.08);
    declare_parameter("pulse_ms", 800);
    declare_parameter("settle_ms", 400);
    declare_parameter("sample_ms", 300);
    declare_parameter("noise_ms", 1500);
    declare_parameter("min_disp", 0.02);

    min_h_ = get_parameter("min_height").as_double();
    max_h_ = get_parameter("max_height").as_double();
    min_r_ = get_parameter("min_range").as_double();
    max_r_ = get_parameter("max_range").as_double();
    voxel_ = get_parameter("voxel").as_double();
    tf_x_ = get_parameter("tf_x").as_double();
    tf_y_ = get_parameter("tf_y").as_double();
    tf_z_ = get_parameter("tf_z").as_double();
    tf_yaw_ = get_parameter("tf_yaw").as_double();
    v_test_ = get_parameter("v_test").as_double();
    pulse_ms_ = get_parameter("pulse_ms").as_int();
    settle_ms_ = get_parameter("settle_ms").as_int();
    sample_ms_ = get_parameter("sample_ms").as_int();
    noise_ms_ = get_parameter("noise_ms").as_int();
    min_disp_ = get_parameter("min_disp").as_double();

    const std::string topic = get_parameter("cloud_topic").as_string();
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&CalibNode::on_cloud, this, _1));
    raw_pub_ = create_publisher<std_msgs::msg::UInt8MultiArray>("/raw_frame", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(kTickMs),
                               std::bind(&CalibNode::on_tick, this));

    RCLCPP_INFO(get_logger(), "标定开始：车会先往协议 +vx 方向走 %.2f 秒，再回中", pulse_ms_ / 1000.0);
    RCLCPP_INFO(get_logger(), "请确认场地空旷、人在急停旁边。等待点云…");
  }

 private:
  enum class Stage {
    kWaitCloud,
    kNoise,
    kRestVx,
    kBeforeVx,
    kPulseVx,
    kSettleVx,
    kAfterVx,
    kReturnVx,
    kRestVy,
    kBeforeVy,
    kPulseVy,
    kSettleVy,
    kAfterVy,
    kReturnVy,
    kReport,
    kDone,
    kFailed,
  };

  // 点云转到车体系下的 2D 点（高度切片 + 距离过滤 + 体素降采样）
  std::vector<Vec2> to_points(const sensor_msgs::msg::PointCloud2& msg) const {
    std::vector<Vec2> pts;
    const double c = std::cos(tf_yaw_);
    const double s = std::sin(tf_yaw_);
    const double inv_voxel = 1.0 / std::max(voxel_, 1e-3);
    std::unordered_set<int64_t> seen;

    sensor_msgs::PointCloud2ConstIterator<float> ix(msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(msg, "z");
    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      const double z = tf_z_ + static_cast<double>(*iz);
      if (z < min_h_ || z > max_h_) continue;
      const double x = tf_x_ + c * static_cast<double>(*ix) - s * static_cast<double>(*iy);
      const double y = tf_y_ + s * static_cast<double>(*ix) + c * static_cast<double>(*iy);
      const double r = std::hypot(x, y);
      if (r < min_r_ || r > max_r_) continue;
      const int gx = static_cast<int>(std::floor(x * inv_voxel));
      const int gy = static_cast<int>(std::floor(y * inv_voxel));
      const int64_t key = (static_cast<int64_t>(gx) << 32) ^
                          static_cast<int64_t>(static_cast<uint32_t>(gy));
      if (!seen.insert(key).second) continue;
      pts.push_back({x, y});
    }
    return pts;
  }

  // 收点云：采样阶段把点集攒起来，供质心差使用；噪声阶段记录每帧质心
  void on_cloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    const std::vector<Vec2> pts = to_points(*msg);
    if (pts.size() < kMinCloudPoints) return;
    last_points_ = pts;

    if (stage_ == Stage::kNoise) {
      noise_centroids_.push_back(centroid_of(pts));
    }

    const bool sampling = (stage_ == Stage::kBeforeVx || stage_ == Stage::kBeforeVy ||
                           stage_ == Stage::kAfterVx || stage_ == Stage::kAfterVy);
    if (sampling) {
      buffer_.insert(buffer_.end(), pts.begin(), pts.end());
      ++buffer_frames_;
    }
  }

  // 当前阶段应该发出去的速度（车体系）
  void desired_velocity(double& vx, double& vy) const {
    vx = 0.0;
    vy = 0.0;
    switch (stage_) {
      case Stage::kPulseVx: vx = v_test_; break;
      case Stage::kReturnVx: vx = -v_test_; break;
      case Stage::kPulseVy: vy = v_test_; break;
      case Stage::kReturnVy: vy = -v_test_; break;
      default: break;
    }
  }

  // 把速度按当前 R_bc 映射成协议字段并发帧（标定期间 R_bc 用单位矩阵，直接透传）
  void publish_cmd(double vx, double vy) {
    const nav::UpFrame frame = nav::pack_velocity_frame(
        seq_++, static_cast<float>(vx), static_cast<float>(vy), 0.0f);
    std_msgs::msg::UInt8MultiArray m;
    m.data.assign(frame.begin(), frame.end());
    raw_pub_->publish(m);
  }

  // 切到新阶段并清空计时和采样缓冲
  void enter(Stage s) {
    stage_ = s;
    stage_ms_ = 0;
    buffer_.clear();
    buffer_frames_ = 0;
    switch (s) {
      case Stage::kNoise: RCLCPP_INFO(get_logger(), "[0/4] 静止 %.1f 秒测点云噪声底（此时车不应动）", noise_ms_ / 1000.0); break;
      case Stage::kRestVx: RCLCPP_INFO(get_logger(), "[1/4] 静止，准备测协议 vx 方向的真实走向"); break;
      case Stage::kBeforeVx: RCLCPP_INFO(get_logger(), "      采样起点…"); break;
      case Stage::kPulseVx: RCLCPP_INFO(get_logger(), "      发 vx=+%.2f，走 %d ms", v_test_, pulse_ms_); break;
      case Stage::kSettleVx: RCLCPP_INFO(get_logger(), "      停车等待稳定…"); break;
      case Stage::kAfterVx: RCLCPP_INFO(get_logger(), "      采样终点…"); break;
      case Stage::kReturnVx: RCLCPP_INFO(get_logger(), "      回中"); break;
      case Stage::kRestVy: RCLCPP_INFO(get_logger(), "[2/4] 静止，准备测协议 vy 方向的真实走向"); break;
      case Stage::kBeforeVy: RCLCPP_INFO(get_logger(), "      采样起点…"); break;
      case Stage::kPulseVy: RCLCPP_INFO(get_logger(), "      发 vy=+%.2f，走 %d ms", v_test_, pulse_ms_); break;
      case Stage::kSettleVy: RCLCPP_INFO(get_logger(), "      停车等待稳定…"); break;
      case Stage::kAfterVy: RCLCPP_INFO(get_logger(), "      采样终点…"); break;
      case Stage::kReturnVy: RCLCPP_INFO(get_logger(), "      回中"); break;
      case Stage::kReport: RCLCPP_INFO(get_logger(), "[3/4] 计算"); break;
      default: break;
    }
  }

  // 取走采样缓冲的点集
  std::vector<Vec2> take_sample() {
    std::vector<Vec2> out;
    out.swap(buffer_);
    buffer_frames_ = 0;
    return out;
  }

  // 噪声底 = 静止阶段各帧质心间的最大漂移距离（车没动时点云也会抖）
  double noise_floor() const {
    double mx = 0.0;
    for (size_t i = 0; i < noise_centroids_.size(); ++i) {
      for (size_t j = i + 1; j < noise_centroids_.size(); ++j) {
        mx = std::max(mx, std::hypot(noise_centroids_[i].x - noise_centroids_[j].x,
                                     noise_centroids_[i].y - noise_centroids_[j].y));
      }
    }
    return mx;
  }

  // 每 20ms：发当前阶段该发的帧，然后推进状态机
  void on_tick() {
    stage_ms_ += kTickMs;

    double vx = 0.0;
    double vy = 0.0;
    desired_velocity(vx, vy);
    publish_cmd(vx, vy);

    const int limit = std::max(kMaxStageMs, 2 * pulse_ms_ + 2000);
    if (stage_ != Stage::kWaitCloud && stage_ != Stage::kReport &&
        stage_ != Stage::kDone && stage_ != Stage::kFailed &&
        stage_ms_ > limit) {
      fail("阶段超时（点云可能断了）");
      return;
    }

    const int pulse = pulse_ms_;
    const int settle = settle_ms_;
    const int sample = sample_ms_;

    switch (stage_) {
      case Stage::kWaitCloud:
        if (!last_points_.empty()) enter(Stage::kNoise);
        break;
      case Stage::kNoise:
        if (stage_ms_ >= noise_ms_) enter(Stage::kRestVx);
        break;
      case Stage::kRestVx:
        if (stage_ms_ >= settle) enter(Stage::kBeforeVx);
        break;
      case Stage::kBeforeVx:
        if (stage_ms_ >= sample) {
          if (buffer_frames_ < 2) { fail("采样帧数不足，点云太稀"); return; }
          before_vx_ = take_sample();
          enter(Stage::kPulseVx);
        }
        break;
      case Stage::kPulseVx:
        if (stage_ms_ >= pulse) enter(Stage::kSettleVx);
        break;
      case Stage::kSettleVx:
        if (stage_ms_ >= settle) enter(Stage::kAfterVx);
        break;
      case Stage::kAfterVx:
        if (stage_ms_ >= sample) {
          if (buffer_frames_ < 2) { fail("采样帧数不足，点云太稀"); return; }
          after_vx_ = take_sample();
          enter(Stage::kReturnVx);
        }
        break;
      case Stage::kReturnVx:
        if (stage_ms_ >= pulse) enter(Stage::kRestVy);
        break;
      case Stage::kRestVy:
        if (stage_ms_ >= settle) enter(Stage::kBeforeVy);
        break;
      case Stage::kBeforeVy:
        if (stage_ms_ >= sample) {
          if (buffer_frames_ < 2) { fail("采样帧数不足，点云太稀"); return; }
          before_vy_ = take_sample();
          enter(Stage::kPulseVy);
        }
        break;
      case Stage::kPulseVy:
        if (stage_ms_ >= pulse) enter(Stage::kSettleVy);
        break;
      case Stage::kSettleVy:
        if (stage_ms_ >= settle) enter(Stage::kAfterVy);
        break;
      case Stage::kAfterVy:
        if (stage_ms_ >= sample) {
          if (buffer_frames_ < 2) { fail("采样帧数不足，点云太稀"); return; }
          after_vy_ = take_sample();
          enter(Stage::kReturnVy);
        }
        break;
      case Stage::kReturnVy:
        if (stage_ms_ >= pulse) enter(Stage::kReport);
        break;
      case Stage::kReport:
        report();
        if (stage_ != Stage::kFailed) enter(Stage::kDone);
        break;
      case Stage::kDone:
      case Stage::kFailed:
        break;
    }
  }

  // 报错并停在零速
  void fail(const std::string& why) {
    RCLCPP_ERROR(get_logger(), "标定失败：%s", why.c_str());
    enter(Stage::kFailed);
    publish_cmd(0.0, 0.0);
    rclcpp::shutdown();
  }

  // 用 core 的质心差算法解出 R_bc 并打印
  void report() {
    Vec2 d1{0.0, 0.0};
    Vec2 d2{0.0, 0.0};
    const bool ok1 = nav::estimate_translation_direction(before_vx_, after_vx_, d1);
    const bool ok2 = nav::estimate_translation_direction(before_vy_, after_vy_, d2);
    if (!ok1 || !ok2) { fail("质心差算不出方向（车没动或点云没重叠）"); return; }

    const double a1 = std::atan2(d1.y, d1.x);
    const double a2 = std::atan2(d2.y, d2.x);
    const int k1 = quantize_quarter(a1);
    const int k2 = quantize_quarter(a2);
    const double q1 = k1 * (M_PI / 2.0);
    const double q2 = k2 * (M_PI / 2.0);

    // 两个实测方向是 R_bc 逆矩阵的两列：d1 = R_bc^-1·(1,0)，d2 = R_bc^-1·(0,1)。
    // 直接求逆：R_bc = [d1|d2]^-1。det 不假设为 +1——镜像轴布局 det=-1，
    // 忽略符号会让 R_bc 整体反号，车往反方向跑。
    const double c1 = std::cos(q1), s1 = std::sin(q1);
    const double c2 = std::cos(q2), s2 = std::sin(q2);
    const double det = c1 * s2 - s1 * c2;
    if (std::fabs(det) < 1e-3) { fail("两个实测方向平行，解不出 R_bc"); return; }
    const double r11 = s2 / det;
    const double r12 = -c2 / det;
    const double r21 = -s1 / det;
    const double r22 = c1 / det;

    nav::Mat2 rbc;
    rbc.m[0][0] = r11;
    rbc.m[0][1] = r12;
    rbc.m[1][0] = r21;
    rbc.m[1][1] = r22;
    const bool rot = nav::is_rotation_like(rbc);

    const double orth = to_deg(std::acos(std::clamp(d1.x * d2.x + d1.y * d2.y, -1.0, 1.0)));
    const double res1 = std::fabs(to_deg(a1 - q1));
    const double res2 = std::fabs(to_deg(a2 - q2));

    RCLCPP_INFO(get_logger(), " ");

    // 顺手量速度量纲：脉冲期间车移动的距离 ÷ 时长 ≈ 实际速度
    const Vec2 c_before = centroid_of(before_vx_);
    const Vec2 c_after = centroid_of(after_vx_);
    const double disp = std::hypot(c_before.x - c_after.x, c_before.y - c_after.y);
    const double t_pulse = pulse_ms_ / 1000.0;
    const double v_real = disp / t_pulse;
    const double scale = (v_real > 1e-4) ? (v_test_ / v_real) : 0.0;

    // 位移有效性：必须明显大于静止噪声底（车没动时点云质心也会抖出假位移）
    const double noise = noise_floor();
    if (disp < 3.0 * noise && disp < min_disp_) {
      RCLCPP_WARN(get_logger(), " ");
      RCLCPP_WARN(get_logger(), "⚠ 车没有动：位移 %.4f m，噪声底 %.4f m（3倍=%.4f m）",
                  disp, noise, 3.0 * noise);
      RCLCPP_WARN(get_logger(), " 按顺序检查：");
      RCLCPP_WARN(get_logger(), "   1. 急停按钮是否旋起");
      RCLCPP_WARN(get_logger(), "   2. 底盘电源 / 电池");
      RCLCPP_WARN(get_logger(), "   3. 串口是否写失败：grep 写失败 /tmp/srm_calib/serial.log");
      RCLCPP_WARN(get_logger(), "   4. 下位机是否需要裁判系统信号才使能（问赛务）");
      RCLCPP_WARN(get_logger(), " 标定结果无效，未输出。");
      return;
    }
    RCLCPP_INFO(get_logger(), "车动了：位移 %.3f m（噪声底 %.4f m）", disp, noise);

    RCLCPP_INFO(get_logger(), "========== R_bc 标定结果 ==========");
    RCLCPP_INFO(get_logger(), "发 vx=+%.2f → 车实际往 (% .2f, % .2f) 走，角度 % 7.1f°", v_test_, d1.x, d1.y, to_deg(a1));
    RCLCPP_INFO(get_logger(), "发 vy=+%.2f → 车实际往 (% .2f, % .2f) 走，角度 % 7.1f°", v_test_, d2.x, d2.y, to_deg(a2));
    RCLCPP_INFO(get_logger(), "两方向夹角 %.1f°（理想正交，即 90°）", orth);
    RCLCPP_INFO(get_logger(), "量化残差 %.1f° / %.1f°（< 15° 视为可靠）", res1, res2);
    RCLCPP_INFO(get_logger(), "-----------------------------------");
    RCLCPP_INFO(get_logger(), "r11 = % .1f    r12 = % .1f", r11, r12);
    RCLCPP_INFO(get_logger(), "r21 = % .1f    r22 = % .1f", r21, r22);
    RCLCPP_INFO(get_logger(), "自检 |det| = %.3f  %s", std::fabs(r11 * r22 - r12 * r21),
                rot ? "✓ 旋转矩阵" : "✗ 不像旋转，检查雷达安装角/地面干扰");
    RCLCPP_INFO(get_logger(), "-----------------------------------");
    RCLCPP_INFO(get_logger(), "启动参数（直接复制）：");
    RCLCPP_INFO(get_logger(), "  r11:=%.1f r12:=%.1f r21:=%.1f r22:=%.1f", r11, r12, r21, r22);
    RCLCPP_INFO(get_logger(), "===================================");
    RCLCPP_INFO(get_logger(), " ");
    RCLCPP_INFO(get_logger(), "========== 速度量纲 ==========");
    RCLCPP_INFO(get_logger(), "发 vx=%.2f 持续 %.2f 秒，车实际移动 %.3f m", v_test_, t_pulse, disp);
    RCLCPP_INFO(get_logger(), "实测平均速度 %.3f m/s（含加/减速段，与稳态速度有偏差，建议手工复核）", v_real);
    if (scale > 0.0) {
      RCLCPP_INFO(get_logger(), "换算 speed_scale ≈ %.2f", scale);
      RCLCPP_INFO(get_logger(), "  speed_scale:=%.2f", scale);
    } else {
      RCLCPP_WARN(get_logger(), "位移太小（%.4f m）量不出速度：加大 pulse_ms 或 v_test 重测", disp);
    }
    RCLCPP_INFO(get_logger(), "==============================");

    if (res1 > 15.0 || res2 > 15.0) {
      RCLCPP_WARN(get_logger(), "残差偏大：雷达安装角可能没对齐（先标定 tf_yaw），或地面/杂物混进了点云");
    }
    if (std::fabs(orth - 90.0) > 15.0) {
      RCLCPP_WARN(get_logger(), "两方向不正交：车可能带转向、或点云里有动态物体");
    }
  }

  double min_h_{0.10}, max_h_{1.50};
  double min_r_{0.30}, max_r_{6.00};
  double voxel_{0.05};
  double tf_x_{0.0}, tf_y_{0.0}, tf_z_{0.0}, tf_yaw_{0.0};
  double v_test_{0.08};
  int pulse_ms_{800}, settle_ms_{400}, sample_ms_{300};

  Stage stage_{Stage::kWaitCloud};
  int stage_ms_{0};
  uint8_t seq_{0};
  std::vector<Vec2> last_points_;
  std::vector<Vec2> buffer_;
  int buffer_frames_{0};
  std::vector<Vec2> before_vx_, after_vx_, before_vy_, after_vy_;
  std::vector<Vec2> noise_centroids_;
  int noise_ms_{1500};
  double min_disp_{0.02};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr raw_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CalibNode>());
  rclcpp::shutdown();
  return 0;
}
