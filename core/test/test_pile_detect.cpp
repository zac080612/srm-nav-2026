// 桩检测单测：合成圆弧点，验证任意半径圆拟合能找回圆心和半径
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
  p.cluster_dist = 0.15;
  p.min_points = 5;
  p.min_radius = 0.03;
  p.max_radius = 0.30;

  // ---- 完整圆：验证圆心和半径都准 ----
  {
    std::vector<nav::Vec2> pts = arc({0.5, -1.0}, 0.08, 0.0, 2.0 * M_PI, 60);
    const auto piles = nav::detect_piles(pts, p);
    CHECK_EQ(piles.size(), static_cast<size_t>(1));
    if (piles.size() == 1) {
      CHECK_NEAR(piles[0].center.x, 0.5, 0.01);
      CHECK_NEAR(piles[0].center.y, -1.0, 0.01);
      CHECK_NEAR(piles[0].radius, 0.08, 0.01);
    }
  }

  // ---- 两个桩：下桩 (0,-1)、上桩 (0,1)，半径 0.08，半圆 ----
  {
    std::vector<nav::Vec2> pts;
    const auto down = arc({0.0, -1.0}, 0.08, 0.0, M_PI, 24);      // 上半圆
    const auto up = arc({0.0, 1.0}, 0.08, M_PI, 2.0 * M_PI, 24);   // 下半圆
    pts.insert(pts.end(), down.begin(), down.end());
    pts.insert(pts.end(), up.begin(), up.end());

    const auto piles = nav::detect_piles(pts, p);
    CHECK_EQ(piles.size(), static_cast<size_t>(2));

    bool found_down = false;
    bool found_up = false;
    for (const auto& pl : piles) {
      if (std::hypot(pl.center.x, pl.center.y + 1.0) < 0.05) found_down = true;
      if (std::hypot(pl.center.x, pl.center.y - 1.0) < 0.05) found_up = true;
    }
    CHECK_TRUE(found_down);
    CHECK_TRUE(found_up);
  }

  // ---- 半径超范围 → 不算桩 ----
  {
    std::vector<nav::Vec2> pts = arc({0.5, -1.0}, 0.5, 0.0, 2.0 * M_PI, 60);  // 半径 0.5 > 0.3
    CHECK_EQ(nav::detect_piles(pts, p).size(), static_cast<size_t>(0));
  }

  // ---- 空点集 → 无桩 ----
  CHECK_EQ(nav::detect_piles({}, p).size(), static_cast<size_t>(0));

  // ---- 点数不足 → 无桩 ----
  {
    std::vector<nav::Vec2> few = {{0.1, 0.0}, {0.2, 0.0}};
    CHECK_EQ(nav::detect_piles(few, p).size(), static_cast<size_t>(0));
  }

  return navtest::summary("test_pile_detect");
}
