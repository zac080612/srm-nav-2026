// T5：状态机离线测试 —— 不需要 Gazebo、不需要实车
// 阈值依据：state_machine.hpp 顶部注释（270 度 ≈ 4.71 rad；交叉点/停车位判定 0.15 m）
#include <cmath>

#include "nav/state_machine.hpp"

#include "nav/test_util.hpp"

namespace {

// 越过"绕满一圈"的累积角（弧度）。270 度 ≈ 4.71 rad，这里给到 6.0 rad 留余量，
// 避免实现里用 >/>= 的细微差别把测试搞红。
constexpr double kPastOneTurn = 6.0;

nav::SmInput base_input() {
  nav::SmInput in;
  in.started = true;
  in.have_piles = true;
  in.b_near = nav::Vec2{0.55, 0.0};   // 正好贴在目标半径上
  in.b_far = nav::Vec2{-0.55, 0.0};
  in.accum_angle = 0.0;
  in.dist_to_cross = 5.0;
  in.dist_to_park = 5.0;
  return in;
}

}  // namespace

int main() {
  // ================= 1) 完整一圈的状态序列 =================
  {
    nav::StateMachine sm;
    CHECK_TRUE(sm.state() == nav::State::Idle);

    // 未发令 → 保持 Idle
    {
      nav::SmInput in = base_input();
      in.started = false;
      sm.step(in);
      CHECK_TRUE(sm.state() == nav::State::Idle);
    }

    // 发令 → StraightIn
    {
      nav::SmInput in = base_input();
      in.b_near = nav::Vec2{3.0, 0.0};   // 还离得很远
      sm.step(in);
      CHECK_TRUE(sm.state() == nav::State::StraightIn);
    }

    // 进入圆上入点 → OrbitDown
    {
      nav::SmInput in = base_input();
      sm.step(in);
      CHECK_TRUE(sm.state() == nav::State::OrbitDown);
    }

    // 累积角过阈值 → Cross
    {
      nav::SmInput in = base_input();
      in.accum_angle = kPastOneTurn;
      sm.step(in);
      CHECK_TRUE(sm.state() == nav::State::Cross);
    }

    // 到交叉点 → OrbitUp
    {
      nav::SmInput in = base_input();
      in.dist_to_cross = 0.10;
      sm.step(in);
      CHECK_TRUE(sm.state() == nav::State::OrbitUp);
    }

    // 上瓣也绕满一圈 → StraightOut
    {
      nav::SmInput in = base_input();
      in.accum_angle = kPastOneTurn;
      sm.step(in);
      CHECK_TRUE(sm.state() == nav::State::StraightOut);
    }

    // 回到停车位 → Stop
    {
      nav::SmInput in = base_input();
      in.dist_to_park = 0.10;
      sm.step(in);
      CHECK_TRUE(sm.state() == nav::State::Stop);
    }

    // Stop 是稳态：继续喂输入不应跳回别的状态
    {
      nav::SmInput in = base_input();
      in.accum_angle = 0.0;
      in.dist_to_park = 0.05;
      sm.step(in);
      CHECK_TRUE(sm.state() == nav::State::Stop);
    }
  }

  // ================= 2) "恰好触发一次"：累积角在阈值附近抖动不能来回跳 =================
  {
    nav::StateMachine sm;
    nav::SmInput in = base_input();
    sm.step(in);                      // -> OrbitDown
    in.b_near = nav::Vec2{3.0, 0.0};
    sm.step(in);                      // 保持（已经不在 StraightIn 的判据里）

    // 第一次越过阈值：进入 Cross
    in.accum_angle = kPastOneTurn;
    sm.step(in);
    const nav::State s1 = sm.state();

    // 之后即使累积角继续略高于阈值，也不应再触发一次（应停在 Cross 或已推进）
    for (int i = 0; i < 5; ++i) {
      in.accum_angle = kPastOneTurn + 0.1 * (i + 1);
      sm.step(in);
    }
    const nav::State s2 = sm.state();
    // 允许推进到 Cross（尚未到交叉点），但**不允许**因为抖动回到 OrbitDown
    CHECK_TRUE(s1 == nav::State::Cross);
    CHECK_TRUE(s2 == nav::State::Cross || s2 == nav::State::OrbitUp);
  }

  // ================= 3) 桩丢失 → 降级 =================
  {
    nav::StateMachine sm;
    nav::SmInput in = base_input();
    sm.step(in);                      // OrbitDown
    in.have_piles = false;

    bool degraded_seen = false;
    for (int i = 0; i < 30; ++i) {    // 30 帧（约 1 s @30Hz）足够超过降级阈值
      const nav::StepResult r = sm.step(in);
      if (r.degraded) degraded_seen = true;
    }
    CHECK_TRUE(degraded_seen);
    // 降级期间不应自己跳到后面的状态
    CHECK_TRUE(sm.state() == nav::State::OrbitDown || sm.state() == nav::State::Cross);
  }

  // ================= 4) reset 能回到初始状态 =================
  {
    nav::StateMachine sm;
    nav::SmInput in = base_input();
    sm.step(in);
    sm.step(in);
    CHECK_TRUE(sm.state() != nav::State::Idle);
    sm.reset();
    CHECK_TRUE(sm.state() == nav::State::Idle);
  }

  // ================= 5) 换桩连跳不误触发（回归）=================
  // 绕完下桩的同一帧恰好到交叉点：OrbitDown -> Cross -> OrbitUp 连跳，
  // 但 OrbitUp 不能沿用下桩的旧 accum_angle 直接跳到 StraightOut。
  {
    nav::StateMachine sm;
    nav::SmInput in = base_input();
    sm.step(in);   // -> OrbitDown

    in.accum_angle = kPastOneTurn;   // 绕下桩满一圈
    in.dist_to_cross = 0.10;         // 已到交叉点
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::OrbitUp);   // 必须停在 OrbitUp

    in.accum_angle = 0.0;   // figure8 换桩后重置累积角
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::OrbitUp);   // 仍 OrbitUp，重新累积

    in.accum_angle = kPastOneTurn;   // 绕上桩也满一圈
    sm.step(in);
    CHECK_TRUE(sm.state() == nav::State::StraightOut);  // 这时才回去
  }

  return navtest::summary("test_state_machine");
}
