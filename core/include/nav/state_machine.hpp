#pragma once

#include "nav/control.hpp"

namespace nav {

enum class State {
  Idle,          // 启动前，只发零速帧
  StraightIn,    // 朝下桩直线驶入，直到进入圆上入点
  OrbitDown,     // 绕下桩（逆时针）
  Cross,         // 朝交叉点（两桩连线中点）走
  OrbitUp,       // 绕上桩（顺时针）
  StraightOut,   // 朝停车位走
  Stop           // 到停车位，发零速帧并保持
};

const char* to_string(State s);

// 状态机一帧的输入
struct SmInput {
  bool started{false};       // 外部发令：true 表示可以开始
  bool have_piles{false};    // 本帧是否同时看到两桩（桩系定位的前提）
  Vec2 b_near{0.0, 0.0};     // 近桩在车体系的位置
  Vec2 b_far{0.0, 0.0};      // 远桩在车体系的位置
  double accum_angle{0.0};   // 当前桩的累积绕行角（弧度）
  double dist_to_cross{0.0}; // 到交叉点的距离（米）
  double dist_to_park{0.0};  // 到停车位的距离（米）
};

// 切换判据：Idle→StraightIn(started)→OrbitDown(|b_near|≤R*1.05)→Cross(绕330°)
//          →OrbitUp(到交叉点<0.15)→StraightOut(绕330°)→Stop(到停车位<0.15)
// 累积角要滞回（触发一次后等回零），桩丢失超过阈值降级
struct StepResult {
  State next{State::Idle};
  bool degraded{false};   // true = 本帧处于降级（桩丢失）
};

class StateMachine {
 public:
  State state() const { return state_; }
  StepResult step(const SmInput& in);
  void reset();

 private:
  State state_{State::Idle};
  bool angle_consumed_{false};   // 累积角是否已经用掉（滞回用）
  int missing_frames_{0};        // 连续丢桩帧数（降级判定用）
};

}  // namespace nav
