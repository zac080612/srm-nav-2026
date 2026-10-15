#include "nav/safe_cmd.hpp"

#include <chrono>
#include <cmath>

namespace nav {

// 取当前时间（秒）
double SafeCmd::now_s() const {
  if (now_fn_) return now_fn_();
  const auto tp = std::chrono::steady_clock::now().time_since_epoch();
  return std::chrono::duration<double>(tp).count();
}

// NaN/Inf 归零后限幅
float SafeCmd::clamp_safe(float v) const {
  if (!std::isfinite(v)) return 0.0f;
  if (v > limit_) return limit_;
  if (v < -limit_) return -limit_;
  return v;
}

// 保存限幅后的速度
void SafeCmd::update(float vx, float vy) {
  vx_ = clamp_safe(vx);
  vy_ = clamp_safe(vy);
  last_update_s_ = now_s();
}

// 打包一帧（超时或未收到指令就发零速）
UpFrame SafeCmd::tick(uint8_t seq) {
  if (last_update_s_ < 0.0) {
    return pack_velocity_frame(seq, 0.0f, 0.0f, 0.0f);
  }
  const double age = now_s() - last_update_s_;
  if (age > static_cast<double>(watchdog_ms_) / 1000.0) {
    return pack_velocity_frame(seq, 0.0f, 0.0f, 0.0f);
  }
  return pack_velocity_frame(seq, vx_, vy_, 0.0f);
}

}  // namespace nav
