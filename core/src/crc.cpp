#include "nav/crc.hpp"

#include <cstddef>
#include <cstdint>

namespace nav {

namespace {

// 查表在编译期生成
struct Crc8Table {
  uint8_t t[256]{};
  constexpr Crc8Table() {
    for (int i = 0; i < 256; ++i) {
      uint8_t c = static_cast<uint8_t>(i);
      for (int b = 0; b < 8; ++b)
        c = (c & 1u) ? static_cast<uint8_t>((c >> 1) ^ 0x8C)
                     : static_cast<uint8_t>(c >> 1);
      t[i] = c;
    }
  }
};

struct Crc16Table {
  uint16_t t[256]{};
  constexpr Crc16Table() {
    for (int i = 0; i < 256; ++i) {
      uint16_t c = static_cast<uint16_t>(i);
      for (int b = 0; b < 8; ++b)
        c = (c & 1u) ? static_cast<uint16_t>((c >> 1) ^ 0x8408)
                     : static_cast<uint16_t>(c >> 1);
      t[i] = c;
    }
  }
};

constexpr Crc8Table kT8{};
constexpr Crc16Table kT16{};

}  // namespace

// CRC8 查表计算
uint8_t crc8(const uint8_t* data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) crc = kT8.t[crc ^ data[i]];
  return crc;
}

// CRC16 查表计算
uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i)
    crc = static_cast<uint16_t>((crc >> 8) ^ kT16.t[(crc ^ data[i]) & 0xFF]);
  return crc;
}

}  // namespace nav
