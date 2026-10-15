#include "nav/control.hpp"

#include <cmath>

namespace nav {

// 绕桩控制律：v = v_t*t + k_r*(r-R)*u
Vec2 orbit_step(const Vec2& b_body, const OrbitParams& p) {
  if (!std::isfinite(b_body.x) || !std::isfinite(b_body.y)) {
    return Vec2{0.0, 0.0};
  }

  const double r = std::hypot(b_body.x, b_body.y);
  if (r < 1e-6) {
    return Vec2{0.0, 0.0};
  }

  const double ux = b_body.x / r;
  const double uy = b_body.y / r;
  const double tx = p.ccw ? uy : -uy;
  const double ty = p.ccw ? -ux : ux;

  const double radial = p.k_r * (r - p.R);
  return Vec2{p.v_t * tx + radial * ux, p.v_t * ty + radial * uy};
}

// 参数合法性检查
bool valid(const OrbitParams& p) {
  return std::isfinite(p.R) && std::isfinite(p.v_t) && std::isfinite(p.k_r) &&
         p.R > 0.0 && p.v_t >= 0.0 && p.k_r >= 0.0;
}

}  // namespace nav
