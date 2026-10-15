#include "nav/frame.hpp"

#include <cstring>

#include "nav/crc.hpp"

namespace nav {

namespace {

// float 按小端写进 4 字节
void write_f32_le(uint8_t* dst, float v) {
  uint32_t u = 0;
  std::memcpy(&u, &v, sizeof(u));
  dst[0] = static_cast<uint8_t>(u & 0xFF);
  dst[1] = static_cast<uint8_t>((u >> 8) & 0xFF);
  dst[2] = static_cast<uint8_t>((u >> 16) & 0xFF);
  dst[3] = static_cast<uint8_t>((u >> 24) & 0xFF);
}

}  // namespace

// 打包 19 字节上行帧
UpFrame pack_velocity_frame(uint8_t seq, float vx, float vy, float wz) {
  UpFrame f{};
  f[0] = 0xA5;
  f[1] = 0x0C;
  f[2] = 0x00;
  f[3] = seq;
  f[4] = crc8(f.data(), 4);
  write_f32_le(f.data() + 5, vx);
  write_f32_le(f.data() + 9, vy);
  write_f32_le(f.data() + 13, wz);
  const uint16_t c = crc16(f.data(), 17);
  f[17] = static_cast<uint8_t>(c & 0xFF);
  f[18] = static_cast<uint8_t>(c >> 8);
  return f;
}

// 下行流式组帧
std::vector<DownFrame> DownParser::feed(const uint8_t* buf, size_t n) {
  buf_.insert(buf_.end(), buf, buf + n);
  std::vector<DownFrame> out;

  while (true) {
    if (buf_.size() < 5) break;
    if (buf_[0] != 0xA5) { drop_front(1); continue; }

    const uint16_t len = read_u16_le(&buf_[1]);
    if (len > 4096) { drop_front(1); continue; }
    const size_t total = 9 + len;
    if (buf_.size() < total) break;

    if (crc8(buf_.data(), 4) != buf_[4]) {
      ++crc8_errors_;
      drop_front(1);
      continue;
    }

    const uint16_t expect = read_u16_le(&buf_[total - 2]);
    if (crc16(buf_.data(), 7 + len) != expect) {
      ++crc16_errors_;
      drop_front(1);
      continue;
    }

    DownFrame fr;
    fr.cmd_id = read_u16_le(&buf_[5]);
    fr.data.assign(buf_.begin() + 7, buf_.begin() + 7 + static_cast<long>(len));
    out.push_back(fr);
    drop_front(total);
  }
  return out;
}

void DownParser::drop_front(size_t n) {
  buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(n));
  resync_drops_ += n;
}

}  // namespace nav
