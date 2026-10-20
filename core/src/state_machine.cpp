#include "nav/state_machine.hpp"

#include <cmath>

namespace nav {

namespace {

constexpr double kArriveDist = 0.15;
constexpr int kMissingLimit = 15;
constexpr double kExitCos = -0.5;   // 出点判据：夹角 > 120° 时认为"车已绕到桩的交叉点侧"
constexpr double kCrossNearDist = 0.30;  // 车离交叉点比这个近时，方向无法可靠判断

// 判断车相对桩的位置状态：
//   返回  1 = 出点（车→桩 与 车→交叉点 反向，车在桩的交叉点侧）
//   返回  0 = 外侧（同向，车在桩远离交叉点的一侧）
//   返回 -1 = 无法判断（车太靠近交叉点，b_cross 太短，方向不可靠）
int cross_side_status(const Vec2& b_near, const Vec2& b_far) {
  const double cx = (b_near.x + b_far.x) * 0.5;   // 交叉点 = 两桩中点
  const double cy = (b_near.y + b_far.y) * 0.5;
  const double nr = std::hypot(b_near.x, b_near.y);
  const double cr = std::hypot(cx, cy);
  if (nr < 1e-9) return -1;
  if (cr < kCrossNearDist) return -1;   // 车离交叉点太近，判据失效
  const double cos_a = (b_near.x * cx + b_near.y * cy) / (nr * cr);
  return cos_a < kExitCos ? 1 : 0;
}

}  // namespace

// 状态名转字符串
const char* to_string(State s) {
  switch (s) {
    case State::Idle: return "Idle";
    case State::StraightIn: return "StraightIn";
    case State::OrbitDown: return "OrbitDown";
    case State::Cross: return "Cross";
    case State::OrbitUp: return "OrbitUp";
    case State::StraightOut: return "StraightOut";
    case State::Stop: return "Stop";
  }
  return "?";
}

// 复位到 Idle
void StateMachine::reset() {
  state_ = State::Idle;
  passed_outside_ = false;
  missing_frames_ = 0;
}

// 状态机前进一步
StepResult StateMachine::step(const SmInput& in) {
  StepResult r;
  r.next = state_;

  if (!in.have_piles) {
    ++missing_frames_;
    if (missing_frames_ > kMissingLimit) r.degraded = true;
    return r;
  }
  missing_frames_ = 0;

  bool advanced = true;
  while (advanced) {
    advanced = false;
    switch (state_) {
      case State::Idle:
        if (in.started) { state_ = State::StraightIn; advanced = true; }
        break;
      case State::StraightIn:
        if (std::hypot(in.b_near.x, in.b_near.y) <= in.orbit_radius * 1.05) {
          state_ = State::OrbitDown;
          passed_outside_ = false;
          advanced = true;
        }
        break;
      case State::OrbitDown: {
        const int s = cross_side_status(in.b_near, in.b_far);
        if (s == 0) passed_outside_ = true;    // 在外侧：记录"已绕过外侧"
        if (s == 1 && passed_outside_) {       // 回到交叉点侧，且绕过外侧 → 切出
          passed_outside_ = false;
          state_ = State::Cross;
          advanced = true;
        }
        break;
      }
      case State::Cross:
        if (in.dist_to_cross < kArriveDist) {
          state_ = State::OrbitUp;
          passed_outside_ = false;
          advanced = true;
        }
        break;
      case State::OrbitUp: {
        const int s = cross_side_status(in.b_near, in.b_far);
        if (s == 0) passed_outside_ = true;
        if (s == 1 && passed_outside_) {
          passed_outside_ = false;
          state_ = State::StraightOut;
          advanced = true;
        }
        break;
      }
      case State::StraightOut:
        if (in.dist_to_park < kArriveDist) {
          state_ = State::Stop;
          advanced = true;
        }
        break;
      case State::Stop:
        break;
    }
  }

  r.next = state_;
  return r;
}

}  // namespace nav
