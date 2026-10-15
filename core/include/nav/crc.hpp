#pragma once
#include <cstddef>
#include <cstdint>

namespace nav {

// CRC8(初值0xFF)/CRC16(初值0xFFFF)，反射、无最终异或
// 自检：crc8("123456789")=0x0B，crc16=0x6F91
uint8_t crc8(const uint8_t* data, size_t len);
uint16_t crc16(const uint8_t* data, size_t len);

}  // namespace nav
