// 用 cat 抓到的下位机真实帧验证 CRC 参数
#include <nav/crc.hpp>
#include <cstdio>
#include <cstdint>

static uint16_t le16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0] | (p[1] << 8));
}

int main() {
  // 下位机实际发出的两帧（用户 cat 抓包，逐字节转录）
  // 比赛状态：data_length 字段 = 0x0B(11)，但实际总长只有 18 字节 → 数据只有 9 字节
  const uint8_t f_status[] = {0xa5, 0x0b, 0x00, 0x86, 0x53, 0x01, 0x00,
                              0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                              0x30, 0x9b};
  // 机器人状态：data_length = 0x0D(13)，总长 22 = 9+13 ✓ 与文档一致
  const uint8_t f_robot[] = {0xa5, 0x0d, 0x00, 0x87, 0xdc, 0x0b, 0x00,
                             0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                             0x00, 0x00, 0x00, 0x00,
                             0xc6, 0xf8};

  const size_t n1 = sizeof(f_status);
  const size_t n2 = sizeof(f_robot);

  printf("=== 比赛状态帧（总长 %zu，字段声称 11 数据，实际 9）===\n", n1);
  printf("  crc8  覆盖0-3: 计算=%02X 帧内=%02X  %s\n",
         nav::crc8(f_status, 4), f_status[4],
         nav::crc8(f_status, 4) == f_status[4] ? "PASS" : "FAIL");
  printf("  crc16 覆盖0-%zu: 计算=%04X 帧内=%04X  %s\n",
         n1 - 3, nav::crc16(f_status, n1 - 2), le16(&f_status[n1 - 2]),
         nav::crc16(f_status, n1 - 2) == le16(&f_status[n1 - 2]) ? "PASS" : "FAIL");

  printf("=== 机器人状态帧（总长 %zu，字段 13 数据）===\n", n2);
  printf("  crc8  覆盖0-3: 计算=%02X 帧内=%02X  %s\n",
         nav::crc8(f_robot, 4), f_robot[4],
         nav::crc8(f_robot, 4) == f_robot[4] ? "PASS" : "FAIL");
  printf("  crc16 覆盖0-%zu: 计算=%04X 帧内=%04X  %s\n",
         n2 - 3, nav::crc16(f_robot, n2 - 2), le16(&f_robot[n2 - 2]),
         nav::crc16(f_robot, n2 - 2) == le16(&f_robot[n2 - 2]) ? "PASS" : "FAIL");

  return 0;
}
