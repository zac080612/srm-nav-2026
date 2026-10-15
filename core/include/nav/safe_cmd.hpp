#pragma once
#include <functional>

#include "nav/frame.hpp"

namespace nav {

// 安全指令包装：NaN/Inf 归零、限幅、500ms 看门狗超时发零速帧
class SafeCmd {
 public:
  // 外部喂入新指令（每帧控制输出调一次）
  void update(float vx, float vy);

  // 每个发送周期调一次，返回本周期要发出的 19 字节帧
  UpFrame tick(uint8_t seq);

  void set_limit(float lim) { limit_ = lim; }
  void set_watchdog_ms(int ms) { watchdog_ms_ = ms; }

  // 注入时钟（单位：秒）。留空则用 std::chrono::steady_clock。
  // 单测靠它推进时间，不需要真的 sleep。
  void set_now_fn(std::function<double()> fn) { now_fn_ = std::move(fn); }

  float last_vx() const { return vx_; }
  float last_vy() const { return vy_; }

 private:
  double now_s() const;
  float clamp_safe(float v) const;

  float limit_{0.2f};
  int watchdog_ms_{500};
  float vx_{0.0f};
  float vy_{0.0f};
  double last_update_s_{-1.0};   // <0 表示还没收到过有效指令
  std::function<double()> now_fn_;
};

}  // namespace nav
