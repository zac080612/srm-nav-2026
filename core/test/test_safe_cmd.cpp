// T1 的一部分：SafeCmd —— 限幅 / NaN 兜底 / 看门狗
// 判据来源：协议第 6 节（不要发 NaN/Inf、自己做限幅、500 ms 看门狗、无超时保护）
#include <cmath>
#include <cstring>
#include <limits>

#include "nav/safe_cmd.hpp"

#include "nav/test_util.hpp"

namespace {

// 从帧里按小端读出 float32（不依赖主机字节序）
float read_f32_le(const nav::UpFrame& f, size_t off) {
  const uint32_t u = static_cast<uint32_t>(f[off]) |
                     (static_cast<uint32_t>(f[off + 1]) << 8) |
                     (static_cast<uint32_t>(f[off + 2]) << 16) |
                     (static_cast<uint32_t>(f[off + 3]) << 24);
  float v = 0.0f;
  std::memcpy(&v, &u, sizeof(v));
  return v;
}

float f32_at_vx(const nav::UpFrame& f) { return read_f32_le(f, 5); }
float f32_at_vy(const nav::UpFrame& f) { return read_f32_le(f, 9); }

}  // namespace

int main() {
  double fake_now = 100.0;  // 注入时钟，单测不真的 sleep

  nav::SafeCmd cmd;
  cmd.set_limit(0.2f);
  cmd.set_watchdog_ms(500);
  cmd.set_now_fn([&fake_now]() { return fake_now; });

  // ---- 1) 还没收到任何指令 → 必须发零速帧 ----
  {
    const auto f = cmd.tick(1);
    CHECK_NEAR(f32_at_vx(f), 0.0, 1e-9);
    CHECK_NEAR(f32_at_vy(f), 0.0, 1e-9);
  }

  // ---- 2) NaN / Inf 必须当作 0（注意：clamp 不会拦住 NaN）----
  {
    cmd.update(std::nanf(""), 0.1f);
    const auto f = cmd.tick(2);
    CHECK_NEAR(f32_at_vx(f), 0.0, 1e-9);
    CHECK_NEAR(f32_at_vy(f), 0.1, 1e-6);  // 另一个轴不受影响（float 0.1 vs double 0.1 差 ~1.5e-9，用 1e-6）

    cmd.update(std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity());
    const auto g = cmd.tick(3);
    CHECK_NEAR(f32_at_vx(g), 0.0, 1e-9);
    CHECK_NEAR(f32_at_vy(g), 0.0, 1e-9);
  }

  // ---- 3) 超限必须被限幅到 ±limit ----
  {
    cmd.update(5.0f, -9.0f);
    const auto f = cmd.tick(4);
    CHECK_NEAR(f32_at_vx(f), 0.2, 1e-6);
    CHECK_NEAR(f32_at_vy(f), -0.2, 1e-6);
  }

  // ---- 4) 看门狗：600 ms 没有新指令 → 零速帧 ----
  {
    cmd.update(0.1f, 0.1f);
    fake_now += 0.4;  // 还在 500 ms 之内
    const auto ok = cmd.tick(5);
    CHECK_NEAR(f32_at_vx(ok), 0.1, 1e-6);

    fake_now += 0.3;  // 累计 700 ms，超时
    const auto stale = cmd.tick(6);
    CHECK_NEAR(f32_at_vx(stale), 0.0, 1e-9);
    CHECK_NEAR(f32_at_vy(stale), 0.0, 1e-9);

    // 收到新指令后应立刻恢复
    cmd.update(0.05f, 0.0f);
    const auto back = cmd.tick(7);
    CHECK_NEAR(f32_at_vx(back), 0.05, 1e-6);
  }

  // ---- 5) seq 必须原样进帧，且帧结构合法（sof / data_length）----
  {
    const auto f = cmd.tick(0xAB);
    CHECK_EQ(f[0], static_cast<uint8_t>(0xA5));
    CHECK_EQ(f[1], static_cast<uint8_t>(12));
    CHECK_EQ(f[2], static_cast<uint8_t>(0));
    CHECK_EQ(f[3], static_cast<uint8_t>(0xAB));
  }

  return navtest::summary("test_safe_cmd");
}
