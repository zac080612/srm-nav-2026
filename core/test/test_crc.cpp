// T1 的一部分：CRC 自检值
// 判据来源：协议 2.2 节的 CRC 自检向量（已独立复算确认）
#include "nav/crc.hpp"

#include "nav/test_util.hpp"

using namespace nav;

int main() {
  const char* s = "123456789";
  const auto* p = reinterpret_cast<const uint8_t*>(s);

  // 自检向量：对 ASCII "123456789"
  CHECK_EQ(crc8(p, 9), 0x0B);     // 协议给的值
  CHECK_EQ(crc16(p, 9), 0x6F91);  // 协议给的值（等价 CRC-16/MCRF4XX）

  // 边界：空输入应等于初值本身（初值 0xFF / 0xFFFF，无最终异或）
  CHECK_EQ(crc8(p, 0), 0xFF);
  CHECK_EQ(crc16(p, 0), 0xFFFF);

  // 再压两个"单元测试友好"的短输入，防止只对最长的自检向量碰巧正确
  const uint8_t a5[] = {0xA5};
  CHECK_EQ(crc8(a5, 1), static_cast<uint8_t>(0xA5));  // 0xFF ^ 0xA5 = 0x5A，移位 8 次后回到 0xA5
  const uint8_t four[] = {0xA5, 0x0C, 0x00, 0x01};
  CHECK_EQ(crc8(four, 4), 0x26);  // 与参考帧第 5 字节一致

  return navtest::summary("test_crc");
}
