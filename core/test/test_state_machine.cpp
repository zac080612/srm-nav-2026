// T5：状态机离线测试 —— 绕桩出点 = 车、桩、交叉点三点共线（夹角>148°）几何判据
#include <cmath>

#include "nav/state_machine.hpp"

#include "nav/test_util.hpp"

namespace {

// 构造一帧输入。默认：已发令、看到两桩、还没到交叉点/停车位。
nav::SmInput base_input() {
  nav::SmInput in;
  in.started = true;
  in.have_piles = true;
  in.b_near = nav::Vec2{0.0, 0.55};   // 近桩在车上方 0.55m
  in.b_far = nav::Vec2{0.0, 2.55};    // 远桩在更上方（桩间距 2m）
  in.accum_angle = 0.0;
  in.dist_to_cross = 5.0;
  in.dist_to_park = 5.0;
  return in;
}

}  // namespace

int main() {
  // ============ 1) 绕下桩：外侧不切，绕到三点共线才切 Cross ============
  {
    nav::StateMachine sm;
    nav::SmInput in = base_input();
    sm.step(in);                       // Idle -> StraightIn -> OrbitDown（连跳）
    CHECK_TRUE(sm.state() == nav::State::OrbitDown);

    // 入点：车在下桩下方，车→下桩 与 车→交叉点 同向（外侧）→ 记录 passed_outside，不切
    in.b_near = nav::Vec2{0.0, 0.55};   // 下桩在车上方
    in.b_far = nav::Vec2{0.0, 2.55};    // 上桩在更上方
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::OrbitDown);

    // 出点：车绕到下桩上方，车→下桩 与 车→交叉点 反向（夹角180°>148°）→ 切 Cross
    in.b_near = nav::Vec2{0.0, -0.55};  // 下桩在车下方
    in.b_far = nav::Vec2{0.0, 1.45};    // 上桩在车上方
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::Cross);
  }

  // ============ 2) 绕上桩：入点就在交叉点附近（无法判断），不能误切 ============
  {
    nav::StateMachine sm;
    nav::SmInput in = base_input();
    sm.step(in);                       // -> OrbitDown
    in.b_near = nav::Vec2{0.0, -0.55};  // 绕到下桩上方
    in.b_far = nav::Vec2{0.0, 1.45};
    sm.step(in);                       // -> Cross
    in.dist_to_cross = 0.10;            // 到交叉点
    sm.step(in);                       // -> OrbitUp
    CHECK_TRUE(sm.state() == nav::State::OrbitUp);

    // 入点：车就在交叉点（两桩中点），车→交叉点 长度≈0，判据失效（-1），不能切
    in.b_near = nav::Vec2{0.0, 1.00};   // 上桩在车上方 1m
    in.b_far = nav::Vec2{0.0, -1.00};   // 下桩在车下方 1m（交叉点=车位置）
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::OrbitUp);

    // 绕到上桩上方（外侧）：车→上桩 与 车→交叉点 同向 → passed_outside_ = true
    in.b_near = nav::Vec2{0.0, -0.55};  // 上桩在车下方
    in.b_far = nav::Vec2{0.0, -2.55};   // 下桩在更下方
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::OrbitUp);

    // 绕回上桩下方（交叉点侧）：反向，且已绕过外侧 → 切 StraightOut
    in.b_near = nav::Vec2{0.0, 0.55};   // 上桩在车上方
    in.b_far = nav::Vec2{0.0, -1.45};   // 下桩在车下方
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::StraightOut);
  }

  // ============ 3) 完整序列到 Stop ============
  {
    nav::StateMachine sm;
    CHECK_TRUE(sm.state() == nav::State::Idle);

    nav::SmInput in = base_input();
    in.started = false;
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::Idle);   // 未发令保持 Idle

    in.started = true;
    in.b_near = nav::Vec2{3.0, 0.0};              // 桩还远
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::StraightIn);

    in.b_near = nav::Vec2{0.0, 0.55};             // 到入点
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::OrbitDown);

    in.b_near = nav::Vec2{0.0, -0.55};            // 绕到下桩上方
    in.b_far = nav::Vec2{0.0, 1.45};
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::Cross);

    in.dist_to_cross = 0.10;                      // 到交叉点
    in.b_near = nav::Vec2{0.0, 1.00};
    in.b_far = nav::Vec2{0.0, -1.00};
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::OrbitUp);

    in.b_near = nav::Vec2{0.0, -0.55};            // 绕到上桩上方
    in.b_far = nav::Vec2{0.0, -2.55};
    sm.step(in);
    in.b_near = nav::Vec2{0.0, 0.55};             // 绕回上桩下方
    in.b_far = nav::Vec2{0.0, -1.45};
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::StraightOut);

    in.dist_to_park = 0.10;                       // 到停车位
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::Stop);

    in.dist_to_park = 0.05;                       // Stop 是稳态
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::Stop);
  }

  // ============ 4) 桩丢失 → 降级 ============
  {
    nav::StateMachine sm;
    nav::SmInput in = base_input();
    sm.step(in);                       // OrbitDown
    in.have_piles = false;

    bool degraded_seen = false;
    for (int i = 0; i < 30; ++i) {
      const nav::StepResult r = sm.step(in);
      if (r.degraded) degraded_seen = true;
    }
    CHECK_TRUE(degraded_seen);
    CHECK_TRUE(sm.state() == nav::State::OrbitDown || sm.state() == nav::State::Cross);
  }

  // ============ 5) reset 回到初始状态 ============
  {
    nav::StateMachine sm;
    nav::SmInput in = base_input();
    sm.step(in);
    sm.step(in);
    CHECK_TRUE(sm.state() != nav::State::Idle);
    sm.reset();
    CHECK_TRUE(sm.state() == nav::State::Idle);
  }

  return navtest::summary("test_state_machine");
}
