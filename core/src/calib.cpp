#include "nav/calib.hpp"

#include <cmath>

namespace nav {

namespace {

bool finite(const Vec2& v) { return std::isfinite(v.x) && std::isfinite(v.y); }

Vec2 mean_of(const std::vector<Vec2>& pts) {
  Vec2 s{0.0, 0.0};
  for (const auto& p : pts) {
    s.x += p.x;
    s.y += p.y;
  }
  const double n = static_cast<double>(pts.size());
  return Vec2{s.x / n, s.y / n};
}

}  // namespace

// 两帧点集估计车运动方向
bool estimate_translation_direction(const std::vector<Vec2>& before,
                                    const std::vector<Vec2>& after,
                                    Vec2& dir_out) {
  if (before.size() < 3 || after.size() < 3) return false;

  for (const auto& p : before) if (!finite(p)) return false;
  for (const auto& p : after) if (!finite(p)) return false;

  const Vec2 mb = mean_of(before);
  const Vec2 ma = mean_of(after);
  const double dx = mb.x - ma.x;
  const double dy = mb.y - ma.y;
  const double len = std::hypot(dx, dy);
  if (len < 1e-9) return false;

  dir_out = Vec2{dx / len, dy / len};
  return true;
}

// 由样本解 R_bc
bool solve_rbc_from_samples(const std::vector<CalibSample>& samples, Mat2& rbc_out) {
  const Vec2* col1 = nullptr;
  const Vec2* col2 = nullptr;

  for (const auto& s : samples) {
    if (std::fabs(s.v_body.x - 1.0) < 0.5 && std::fabs(s.v_body.y) < 0.5) {
      if (col1 == nullptr) col1 = &s.v_measured;
    } else if (std::fabs(s.v_body.x) < 0.5 && std::fabs(s.v_body.y - 1.0) < 0.5) {
      if (col2 == nullptr) col2 = &s.v_measured;
    }
  }

  if (col1 == nullptr || col2 == nullptr) return false;

  const double det = col1->x * col2->y - col1->y * col2->x;
  if (std::fabs(det) < 1e-3) return false;

  rbc_out.m[0][0] = col1->x;
  rbc_out.m[1][0] = col1->y;
  rbc_out.m[0][1] = col2->x;
  rbc_out.m[1][1] = col2->y;
  return true;
}

}  // namespace nav
