// T1 的主体：19 字节上行帧逐字节比对 + 下行流式组帧
// 所有十六进制真值都来自协议文档（第 4.3、5 节），不要手改。
#include "nav/frame.hpp"

#include "nav/test_util.hpp"

namespace {

// ---- 协议第 5 节的三个参考帧（seq = 1）----
const uint8_t kRefVx1[19] = {
    0xA5, 0x0C, 0x00, 0x01, 0x26, 0x00, 0x00, 0x80, 0x3F, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6B, 0xA3};

const uint8_t kRefStop[19] = {
    0xA5, 0x0C, 0x00, 0x01, 0x26, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x9F};

const uint8_t kRefMix[19] = {
    0xA5, 0x0C, 0x00, 0x01, 0x26, 0x00, 0x00, 0x80, 0x3F, 0x00,
    0x00, 0x80, 0xBF, 0x00, 0x00, 0x40, 0x40, 0xD3, 0x96};

// ---- 协议第 4.3 节的反馈示例帧（cmd_id = 0x0001，20 字节）----
const uint8_t kRefFeedback[20] = {
    0xA5, 0x0B, 0x00, 0x01, 0x5C, 0x01, 0x00, 0xAA, 0x2C, 0x01,
    0x88, 0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0xEF, 0x74};

}  // namespace

int main() {
  // ================= 上行：三个参考帧 =================
  {
    const auto f = nav::pack_velocity_frame(1, 1.0f, 0.0f, 0.0f);
    CHECK_BYTES(f.data(), kRefVx1, 19);
  }
  {
    const auto f = nav::pack_velocity_frame(1, 0.0f, 0.0f, 0.0f);
    CHECK_BYTES(f.data(), kRefStop, 19);
  }
  {
    const auto f = nav::pack_velocity_frame(1, 1.0f, -1.0f, 3.0f);
    CHECK_BYTES(f.data(), kRefMix, 19);
  }

  // ================= 下行：整体喂入 =================
  {
    nav::DownParser p;
    const auto frames = p.feed(kRefFeedback, sizeof(kRefFeedback));
    CHECK_EQ(frames.size(), static_cast<size_t>(1));
    if (frames.size() == 1) {
      const auto& fr = frames[0];
      CHECK_EQ(fr.cmd_id, nav::kCmdGameStatus);
      CHECK_EQ(fr.data.size(), nav::kGameStatusLen);

      if (fr.data.size() == nav::kGameStatusLen) {
        // data[0]：低 4 位比赛类型、高 4 位当前阶段
        CHECK_EQ(nav::game_type_of(fr.data[0]), static_cast<uint8_t>(0x0A));
        CHECK_EQ(nav::game_progress_of(fr.data[0]), static_cast<uint8_t>(0x0A));
        // data[1..2]：stage_remain_time = 300（小端 2C 01）
        CHECK_EQ(nav::read_u16_le(&fr.data[1]), static_cast<uint16_t>(300));
        // data[3..10]：sync_time_stamp = 0x1122334455667788
        CHECK_EQ(nav::read_u64_le(&fr.data[3]), 0x1122334455667788ULL);
      }
    }
    CHECK_EQ(p.crc16_errors(), static_cast<size_t>(0));
    CHECK_EQ(p.crc8_errors(), static_cast<size_t>(0));
  }

  // ================= 下行：1 字节一片（这是最关键的一条）=================
  {
    nav::DownParser p;
    size_t total = 0;
    for (size_t i = 0; i < sizeof(kRefFeedback); ++i) {
      const auto frames = p.feed(kRefFeedback + i, 1);
      total += frames.size();
    }
    CHECK_EQ(total, static_cast<size_t>(1));  // 切片喂入也必须正好解析出 1 帧
  }

  // ================= 下行：两帧粘连一次喂入 =================
  {
    nav::DownParser p;
    uint8_t two[40];
    for (size_t i = 0; i < 20; ++i) {
      two[i] = kRefFeedback[i];
      two[20 + i] = kRefFeedback[i];
    }
    const auto frames = p.feed(two, 40);
    CHECK_EQ(frames.size(), static_cast<size_t>(2));
  }

  // ================= 下行：坏一字节 → 计入错误、不产出帧 =================
  {
    nav::DownParser p;
    uint8_t bad[20];
    for (size_t i = 0; i < 20; ++i) bad[i] = kRefFeedback[i];
    bad[9] ^= 0xFF;  // 破坏 data 里的一个字节，crc16 必然不符
    const auto frames = p.feed(bad, sizeof(bad));
    CHECK_EQ(frames.size(), static_cast<size_t>(0));
    CHECK_TRUE(p.crc16_errors() + p.crc8_errors() + p.resync_drops() > 0);
  }

  return navtest::summary("test_frame");
}
