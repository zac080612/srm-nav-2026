// 串口后端：19 字节帧写串口发给下位机，读反馈解析比赛阶段（默认不编译）

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include <nav/frame.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <mutex>
#include <thread>
#include <vector>

using std::placeholders::_1;

// 打开串口并配置 115200 8N1
static int open_serial(const std::string& device, int baud) {
  const int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) return -1;

  struct termios tty;
  std::memset(&tty, 0, sizeof(tty));
  if (tcgetattr(fd, &tty) != 0) { ::close(fd); return -1; }

  cfsetospeed(&tty, static_cast<speed_t>(baud));
  cfsetispeed(&tty, static_cast<speed_t>(baud));
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;
  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd, TCSANOW, &tty) != 0) { ::close(fd); return -1; }
  return fd;
}

class SerialNode : public rclcpp::Node {
 public:
  SerialNode() : Node("serial_protocol") {
    declare_parameter("device", "/dev/ttyACM0");
    declare_parameter("baud", 115200);
    const std::string device = get_parameter("device").as_string();
    const int baud = get_parameter("baud").as_int();

    fd_ = open_serial(device, baud);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "打不开串口 %s（检查设备、是否加入 dialout 组）", device.c_str());
      return;
    }
    RCLCPP_INFO(get_logger(), "串口 %s 已打开", device.c_str());

    sub_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/raw_frame", 10, std::bind(&SerialNode::on_frame, this, _1));
    pub_progress_ = create_publisher<std_msgs::msg::UInt8>("/game_progress", 10);
    send_timer_ = create_wall_timer(std::chrono::milliseconds(10),
                                    std::bind(&SerialNode::on_send_timer, this));
    recv_thread_ = std::thread(&SerialNode::recv_loop, this);
  }

  ~SerialNode() {
    running_ = false;
    if (recv_thread_.joinable()) recv_thread_.join();
    if (fd_ >= 0) ::close(fd_);
  }

 private:
  void on_frame(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_frame_ = msg->data;
    last_rx_ = std::chrono::steady_clock::now();
  }

  // 定时发帧；超过 500ms 没收到新帧就发零速，防止上游节点崩溃后车一直跑
  void on_send_timer() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fd_ < 0) return;

    const double age = std::chrono::duration<double>(
                           std::chrono::steady_clock::now() - last_rx_).count();
    const bool fresh = latest_frame_.size() == nav::kUpFrameLen && age < 0.5;

    std::vector<uint8_t> frame;
    if (fresh) {
      frame = latest_frame_;
    } else {
      const nav::UpFrame zero = nav::pack_velocity_frame(seq_++, 0.0f, 0.0f, 0.0f);
      frame.assign(zero.begin(), zero.end());
    }

    if (::write(fd_, frame.data(), frame.size()) < 0)
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "串口写失败");
  }

  // 读串口并解析比赛阶段
  void recv_loop() {
    uint8_t buf[256];
    while (running_) {
      const ssize_t n = ::read(fd_, buf, sizeof(buf));
      if (n <= 0) continue;
      const auto frames = parser_.feed(buf, static_cast<size_t>(n));
      recv_bytes_ += static_cast<size_t>(n);
      recv_frames_ += frames.size();
      const auto now = std::chrono::steady_clock::now();
      if (now - last_recv_log_ > std::chrono::seconds(2)) {
        last_recv_log_ = now;
        RCLCPP_INFO(get_logger(),
                    "串口接收：累计 %zu 字节 %zu 帧 crc8错=%zu crc16错=%zu",
                    recv_bytes_, recv_frames_,
                    parser_.crc8_errors(), parser_.crc16_errors());
      }
      for (const auto& f : frames) {
        if (f.cmd_id == nav::kCmdGameStatus && f.data.size() >= nav::kGameStatusLen) {
          std_msgs::msg::UInt8 m;
          m.data = nav::game_progress_of(f.data[0]);
          pub_progress_->publish(m);
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                               "发布 game_progress=%u", m.data);
        }
      }
    }
  }

  int fd_{-1};
  nav::DownParser parser_;
  std::vector<uint8_t> latest_frame_;
  std::chrono::steady_clock::time_point last_rx_{};
  std::chrono::steady_clock::time_point last_recv_log_{};
  size_t recv_bytes_{0};
  size_t recv_frames_{0};
  uint8_t seq_{0};
  std::mutex mutex_;
  std::thread recv_thread_;
  bool running_{true};

  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_progress_;
  rclcpp::TimerBase::SharedPtr send_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialNode>());
  rclcpp::shutdown();
  return 0;
}
