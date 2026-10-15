// 桩检测单测：合成桩表面圆弧，验证能找回圆心
#include <cmath>
#include <vector>

#include "nav/pile_detect.hpp"
#include "nav/test_util.hpp"

namespace {

// 生成圆心 center、半径 r 的圆弧点（角度 [a0, a1]，共 n+1 个点）
std::vector<nav::Vec2> arc(const nav::Vec2& center, double r,
                           double a0, double a1, int n) {
  std::vector<nav::Vec2> pts;
  for (int i = 0; i <= n; ++i) {
    const double a = a0 + (a1 - a0) * i / n;
    pts.push_back({center.x + r * std::cos(a), center.y + r * std::sin(a)});
  }
  return pts;
}

}  // namespace

int main() {
  nav::PileDetectParams p;
  p.pile_radius = 0.08;
  p.cluster_dist = 0.15;
  p.min_points = 5;
  p.fit_tol = 0.02;

  // ---- 两个桩：下桩 (0,-1)、上桩 (0,1)，车在原点 ----
  // 车在原点，看下桩的上半圆弧（朝车），看上桩的下半圆弧（朝车）
  std::vector<nav::Vec2> pts;
  const auto down = arc({0.0, -1.0}, 0.08, 0.0, M_PI, 16);    // 上半圆
  const auto up = arc({0.0, 1.0}, 0.08, M_PI, 2.0 * M_PI, 16);  // 下半圆
  pts.insert(pts.end(), down.begin(), down.end());
  pts.insert(pts.end(), up.begin(), up.end());

  const auto piles = nav::detect_piles(pts, p);

  CHECK_EQ(piles.size(), static_cast<size_t>(2));

  bool found_down = false;
  bool found_up = false;
  for (const auto& c : piles) {
    if (std::hypot(c.x, c.y + 1.0) < 0.02) found_down = true;
    if (std::hypot(c.x, c.y - 1.0) < 0.02) found_up = true;
  }
  CHECK_TRUE(found_down);
  CHECK_TRUE(found_up);

  // ---- 边界：空点集 → 无桩 ----
  CHECK_EQ(nav::detect_piles({}, p).size(), static_cast<size_t>(0));

  // ---- 边界：点数不足 → 无桩 ----
  {
    std::vector<nav::Vec2> few = {{0.1, 0.0}, {0.2, 0.0}};
    CHECK_EQ(nav::detect_piles(few, p).size(), static_cast<size_t>(0));
  }

  return navtest::summary("test_pile_detect");
}
