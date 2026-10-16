#include "nav/state_machine.hpp"

#include <cmath>

namespace nav {

namespace {

constexpr double kOneTurnRad = 270.0 * 3.14159265358979323846 / 180.0;  // 270 度
constexpr double kResetAngleRad = 0.2;
constexpr double kArriveDist = 0.15;
constexpr int kMissingLimit = 15;

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
  angle_consumed_ = false;
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
          angle_consumed_ = false;
          advanced = true;
        }
        break;
      case State::OrbitDown:
        if (in.accum_angle > kOneTurnRad && !angle_consumed_) {
          angle_consumed_ = true;
          state_ = State::Cross;
          advanced = true;
        } else if (in.accum_angle < kResetAngleRad) {
          angle_consumed_ = false;
        }
        break;
      case State::Cross:
        if (in.dist_to_cross < kArriveDist) {
          state_ = State::OrbitUp;
          advanced = true;
        }
        break;
      case State::OrbitUp:
        if (in.accum_angle > kOneTurnRad && !angle_consumed_) {
          angle_consumed_ = true;
          state_ = State::StraightOut;
          advanced = true;
        } else if (in.accum_angle < kResetAngleRad) {
          angle_consumed_ = false;
        }
        break;
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
