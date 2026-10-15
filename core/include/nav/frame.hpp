#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace nav {

// 上→下：固定 19 字节，没有 cmd_id
// 偏移：0=0xA5, 1-2=长度12, 3=seq, 4=crc8(覆盖0-3), 5-16=3个float32, 17-18=crc16(覆盖0-16)
inline constexpr size_t kUpFrameLen = 19;
inline constexpr size_t kUpDataLen = 12;
using UpFrame = std::array<uint8_t, kUpFrameLen>;

UpFrame pack_velocity_frame(uint8_t seq, float vx, float vy, float wz);

// 下→上：9+data_length 字节，含 2 字节 cmd_id（偏移 5-6），crc16 覆盖 7+len（含 cmd_id）
struct DownFrame {
  uint16_t cmd_id{0};
  std::vector<uint8_t> data;
};

// 流式解析器（一次 read 可能不是一帧，校验失败丢 1 字节重同步）
class DownParser {
 public:
  std::vector<DownFrame> feed(const uint8_t* buf, size_t n);

  size_t crc16_errors() const { return crc16_errors_; }
  size_t crc8_errors() const { return crc8_errors_; }
  size_t resync_drops() const { return resync_drops_; }

 private:
  void drop_front(size_t n);

  std::vector<uint8_t> buf_;
  size_t crc16_errors_{0};
  size_t crc8_errors_{0};
  size_t resync_drops_{0};
};

// 下位机当前发送的两个数据包
inline constexpr uint16_t kCmdGameStatus = 0x0001;   // data_length = 11
inline constexpr uint16_t kCmdRobotStatus = 0x000B;  // data_length = 13
inline constexpr size_t kGameStatusLen = 11;
inline constexpr size_t kRobotStatusLen = 13;

// cmd_id=0x0001 的 data[0]：低 4 位比赛类型、高 4 位比赛阶段
inline uint8_t game_type_of(uint8_t b) { return static_cast<uint8_t>(b & 0x0F); }
inline uint8_t game_progress_of(uint8_t b) { return static_cast<uint8_t>(b >> 4); }

inline uint16_t read_u16_le(const uint8_t* p) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}

inline uint64_t read_u64_le(const uint8_t* p) {
  uint64_t v = 0;
  for (int i = 7; i >= 0; --i) v = (v << 8) | static_cast<uint64_t>(p[i]);
  return v;
}

}  // namespace nav
