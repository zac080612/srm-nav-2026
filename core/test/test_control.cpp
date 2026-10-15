// T4：绕桩控制律（车体系直连版）—— 6 个用例
// 手算依据见 control.hpp / control.cpp 顶部注释
#include <cmath>
#include <limits>

#include "nav/control.hpp"

#include "nav/test_util.hpp"

int main() {
  nav::OrbitParams p;
  p.R = 0.5;
  p.v_t = 0.1;
  p.k_r = 0.2;
  p.ccw = true;

  // ---- 用例 1：r == R，桩在车体系 +x 方向 ----
  //    u = (1,0)，t = (0,-1)（逆时针，向下），e_r = 0
  //    v_body = 0.1 * (0,-1) = (0, -0.1)
  {
    const nav::Vec2 v = nav::orbit_step(nav::Vec2{0.5, 0.0}, p);
    CHECK_NEAR(v.x, 0.0, 1e-9);
    CHECK_NEAR(v.y, -0.1, 1e-9);
  }

  // ---- 用例 1b：顺时针时切向反号 ----
  {
    nav::OrbitParams q = p;
    q.ccw = false;
    const nav::Vec2 v = nav::orbit_step(nav::Vec2{0.5, 0.0}, q);
    CHECK_NEAR(v.x, 0.0, 1e-9);
    CHECK_NEAR(v.y, 0.1, 1e-9);
  }

  // ---- 用例 2：r > R（太远）→ 速度里必须有**指向桩**的分量 ----
  //    b = (1.0, 0)：u = (1,0)，e_r = +0.5
  //    v_body = 0.1*(0,-1) + 0.2*0.5*(1,0) = (0.1, -0.1)
  {
    const nav::Vec2 v = nav::orbit_step(nav::Vec2{1.0, 0.0}, p);
    CHECK_NEAR(v.x, 0.1, 1e-9);
    CHECK_NEAR(v.y, -0.1, 1e-9);
    CHECK_TRUE(v.x > 0.0);  // 关键性质：指向桩（+x 方向就是朝桩）
  }

  // ---- 用例 3：r < R（太近）→ 速度里必须有**背离桩**的分量 ----
  //    b = (0.25, 0)：e_r = -0.25
  //    v_body = 0.1*(0,-1) + 0.2*(-0.25)*(1,0) = (-0.05, -0.1)
  {
    const nav::Vec2 v = nav::orbit_step(nav::Vec2{0.25, 0.0}, p);
    CHECK_NEAR(v.x, -0.05, 1e-9);
    CHECK_NEAR(v.y, -0.1, 1e-9);
    CHECK_TRUE(v.x < 0.0);  // 关键性质：背离桩
  }

  // ---- 用例 4：桩恰好在车正上方（b 只有一个分量）----
  //    b = (0, 0.5)：u = (0,1)，t = (1,0)（逆时针，向右），e_r = 0
  //    v_body = 0.1 * (1,0) = (0.1, 0)
  {
    const nav::Vec2 v = nav::orbit_step(nav::Vec2{0.0, 0.5}, p);
    CHECK_NEAR(v.x, 0.1, 1e-9);
    CHECK_NEAR(v.y, 0.0, 1e-9);
  }

  // ---- 用例 5：桩几乎在车中心 → 必须返回 0，绝不能出 inf/nan ----
  {
    const nav::Vec2 v = nav::orbit_step(nav::Vec2{0.0, 0.0}, p);
    CHECK_NEAR(v.x, 0.0, 1e-12);
    CHECK_NEAR(v.y, 0.0, 1e-12);
    CHECK_TRUE(std::isfinite(v.x) && std::isfinite(v.y));

    const nav::Vec2 tiny = nav::orbit_step(nav::Vec2{1e-9, 0.0}, p);
    CHECK_TRUE(std::isfinite(tiny.x) && std::isfinite(tiny.y));
  }

  // ---- 用例 6：NaN / Inf 输入 → 返回 0 ----
  {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const nav::Vec2 a = nav::orbit_step(nav::Vec2{nan, 0.5}, p);
    const nav::Vec2 b = nav::orbit_step(nav::Vec2{0.5, inf}, p);
    CHECK_TRUE(std::isfinite(a.x) && std::isfinite(a.y));
    CHECK_TRUE(std::isfinite(b.x) && std::isfinite(b.y));
    CHECK_NEAR(a.x, 0.0, 1e-12);
    CHECK_NEAR(b.y, 0.0, 1e-12);
  }

  // ---- 附：参数合法性 ----
  {
    CHECK_TRUE(nav::valid(p));
    nav::OrbitParams bad = p;
    bad.R = -1.0;
    CHECK_TRUE(!nav::valid(bad));
    bad = p;
    bad.v_t = std::numeric_limits<double>::quiet_NaN();
    CHECK_TRUE(!nav::valid(bad));
  }

  return navtest::summary("test_control");
}
