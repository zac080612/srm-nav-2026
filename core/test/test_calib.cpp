// 两帧点集估计车运动方向 + 由样本解 R_bc 的单测
#include <cmath>
#include <vector>

#include "nav/calib.hpp"

#include "nav/test_util.hpp"

namespace {

std::vector<nav::Vec2> square() {
  return {nav::Vec2{0.0, 0.0}, nav::Vec2{1.0, 0.0}, nav::Vec2{0.0, 1.0}, nav::Vec2{1.0, 1.0}};
}

std::vector<nav::Vec2> shifted(const std::vector<nav::Vec2>& src, double dx, double dy) {
  std::vector<nav::Vec2> out;
  out.reserve(src.size());
  for (const auto& p : src) out.push_back(nav::Vec2{p.x + dx, p.y + dy});
  return out;
}

}  // namespace

int main() {
  const std::vector<nav::Vec2> A = square();

  // B 比 A 往 +x 移 → dir 指向 -x
  {
    const auto B = shifted(A, 0.05, 0.0);
    nav::Vec2 dir{0.0, 0.0};
    const bool ok = nav::estimate_translation_direction(A, B, dir);
    CHECK_TRUE(ok);
    if (ok) {
      CHECK_NEAR(dir.x, -1.0, 1e-6);
      CHECK_NEAR(dir.y, 0.0, 1e-6);
    }
  }

  // B 比 A 往 -y 移 → dir 指向 +y
  {
    const auto B = shifted(A, 0.0, -0.08);
    nav::Vec2 dir{0.0, 0.0};
    const bool ok = nav::estimate_translation_direction(A, B, dir);
    CHECK_TRUE(ok);
    if (ok) {
      CHECK_NEAR(dir.x, 0.0, 1e-6);
      CHECK_NEAR(dir.y, 1.0, 1e-6);
    }
  }

  // ================= 3) 斜向位移，只关心方向（长度必须归一化）=================
  {
    const auto B = shifted(A, 0.03, 0.03);
    nav::Vec2 dir{0.0, 0.0};
    const bool ok = nav::estimate_translation_direction(A, B, dir);
    CHECK_TRUE(ok);
    if (ok) {
      const double len = std::hypot(dir.x, dir.y);
      CHECK_NEAR(len, 1.0, 1e-9);              // 必须是单位向量
      CHECK_NEAR(dir.x, -std::sqrt(0.5), 1e-6);
      CHECK_NEAR(dir.y, -std::sqrt(0.5), 1e-6);
    }
  }

  // ================= 4) 退化：点数不足 =================
  {
    const std::vector<nav::Vec2> few = {nav::Vec2{0, 0}, nav::Vec2{1, 1}};
    nav::Vec2 dir{0.0, 0.0};
    CHECK_TRUE(!nav::estimate_translation_direction(few, few, dir));
  }

  // ================= 5) 退化：两帧完全相同（没动，测不出方向）=================
  {
    nav::Vec2 dir{0.0, 0.0};
    CHECK_TRUE(!nav::estimate_translation_direction(A, A, dir));
  }

  // ================= 6) 退化：含 NaN =================
  {
    auto bad = A;
    bad[0].x = std::nan("");
    nav::Vec2 dir{0.0, 0.0};
    CHECK_TRUE(!nav::estimate_translation_direction(bad, A, dir));
  }

  // ================= 7) 由样本解 R_bc =================
  // 假设标定得到：发 +x（车体系）时车实际往 +y 走；发 +y 时车实际往 -x 走。
  // 那么 R_bc 的两列分别是 (0,1) 和 (-1,0)，即 90 度旋转。
  {
    std::vector<nav::CalibSample> s;
    s.push_back({nav::Vec2{1.0, 0.0}, nav::Vec2{0.0, 1.0}});
    s.push_back({nav::Vec2{0.0, 1.0}, nav::Vec2{-1.0, 0.0}});
    nav::Mat2 rbc;
    const bool ok = nav::solve_rbc_from_samples(s, rbc);
    CHECK_TRUE(ok);
    if (ok) {
      CHECK_NEAR(rbc.m[0][0], 0.0, 1e-6);
      CHECK_NEAR(rbc.m[1][0], 1.0, 1e-6);
      CHECK_NEAR(rbc.m[0][1], -1.0, 1e-6);
      CHECK_NEAR(rbc.m[1][1], 0.0, 1e-6);
      CHECK_TRUE(nav::is_rotation_like(rbc));

      // 解出的矩阵必须能把 v_body 映射到当时观测到的方向
      const auto c = nav::map_body_to_protocol(rbc, 1.0, 0.0);
      CHECK_NEAR(c.vx, 0.0, 1e-6);
      CHECK_NEAR(c.vy, 1.0, 1e-6);
    }
  }

  // ================= 8) 退化：样本不足 / 两列共线 =================
  {
    nav::Mat2 rbc;
    std::vector<nav::CalibSample> one = {{nav::Vec2{1.0, 0.0}, nav::Vec2{0.0, 1.0}}};
    CHECK_TRUE(!nav::solve_rbc_from_samples(one, rbc));

    std::vector<nav::CalibSample> collinear = {{nav::Vec2{1.0, 0.0}, nav::Vec2{0.0, 1.0}},
                                               {nav::Vec2{1.0, 0.0}, nav::Vec2{0.0, 1.0}}};
    CHECK_TRUE(!nav::solve_rbc_from_samples(collinear, rbc));
  }

  return navtest::summary("test_calib");
}
