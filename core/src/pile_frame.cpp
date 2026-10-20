#include "nav/pile_frame.hpp"

#include <cmath>

namespace nav {

// 车在桩系的位置（桩系：下桩原点，x 轴沿下桩→上桩，y 轴右手系）
Vec2 car_in_pile_frame(const Vec2& b_down, const Vec2& b_up) {
  const double dx = b_up.x - b_down.x;
  const double dy = b_up.y - b_down.y;
  const double D = std::hypot(dx, dy);
  if (D < 1e-9) return Vec2{0.0, 0.0};   // 两桩重合，无法建桩系
  // 车相对下桩 = -b_down，投影到桩系两轴
  const double cx = -(b_down.x * dx + b_down.y * dy) / D;
  const double cy = (b_down.x * dy - b_down.y * dx) / D;
  return Vec2{cx, cy};
}

// 桩系向量 → 车体系向量
Vec2 pile_to_body(const Vec2& v_pile, const Vec2& b_down, const Vec2& b_up) {
  const double dx = b_up.x - b_down.x;
  const double dy = b_up.y - b_down.y;
  const double D = std::hypot(dx, dy);
  if (D < 1e-9) return Vec2{0.0, 0.0};
  const double ex = dx / D, ey = dy / D;   // 桩系 x 轴（车体系单位向量）
  // 桩系 y 轴 = x 轴逆时针 90° = (-ey, ex)
  return Vec2{v_pile.x * ex - v_pile.y * ey,
              v_pile.x * ey + v_pile.y * ex};
}

// 车→停车位 的车体系方向
Vec2 to_park_body(const Vec2& b_down, const Vec2& b_up, const Vec2& p_park) {
  const Vec2 c = car_in_pile_frame(b_down, b_up);
  return pile_to_body(Vec2{p_park.x - c.x, p_park.y - c.y}, b_down, b_up);
}

}  // namespace nav