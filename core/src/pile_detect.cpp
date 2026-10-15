#include "nav/pile_detect.hpp"

#include <cmath>

namespace nav {

namespace {

// 贪心聚类
std::vector<std::vector<Vec2>> cluster_points(const std::vector<Vec2>& pts,
                                              double dist) {
  std::vector<std::vector<Vec2>> out;
  std::vector<bool> used(pts.size(), false);
  for (size_t i = 0; i < pts.size(); ++i) {
    if (used[i]) continue;
    std::vector<Vec2> c{pts[i]};
    used[i] = true;
    for (size_t j = 0; j < pts.size(); ++j) {
      if (used[j]) continue;
      for (const auto& q : c) {
        if (std::hypot(pts[j].x - q.x, pts[j].y - q.y) < dist) {
          c.push_back(pts[j]);
          used[j] = true;
          break;
        }
      }
    }
    out.push_back(std::move(c));
  }
  return out;
}

// 由两点和半径求圆心
void circle_centers(const Vec2& a, const Vec2& b, double r,
                    Vec2& c1, Vec2& c2, bool& ok) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double d = std::hypot(dx, dy);
  if (d < 1e-6 || d > 2.0 * r) {
    ok = false;
    return;
  }
  const double mx = (a.x + b.x) * 0.5;
  const double my = (a.y + b.y) * 0.5;
  const double h = std::sqrt(r * r - (d * 0.5) * (d * 0.5));
  const double nx = -dy / d;
  const double ny = dx / d;
  c1 = Vec2{mx + h * nx, my + h * ny};
  c2 = Vec2{mx - h * nx, my - h * ny};
  ok = true;
}

// 投票拟合圆心
bool fit_circle(const std::vector<Vec2>& pts, double r, double tol, Vec2& center) {
  int best_votes = 0;
  Vec2 best{0.0, 0.0};
  const size_t n = pts.size();
  for (size_t i = 0; i < n; ++i) {
    for (size_t j = i + 1; j < n; ++j) {
      Vec2 c1, c2;
      bool ok = false;
      circle_centers(pts[i], pts[j], r, c1, c2, ok);
      if (!ok) continue;
      const Vec2 cand[2] = {c1, c2};
      for (const auto& c : cand) {
        int votes = 0;
        for (const auto& q : pts) {
          if (std::fabs(std::hypot(q.x - c.x, q.y - c.y) - r) < tol) ++votes;
        }
        if (votes > best_votes) {
          best_votes = votes;
          best = c;
        }
      }
    }
  }
  if (best_votes < 3) return false;
  center = best;
  return true;
}

}  // namespace

// 从点集检测桩圆心
std::vector<Vec2> detect_piles(const std::vector<Vec2>& points,
                               const PileDetectParams& p) {
  std::vector<Vec2> out;
  const auto clusters = cluster_points(points, p.cluster_dist);
  for (const auto& c : clusters) {
    if (c.size() < p.min_points) continue;
    Vec2 center;
    if (fit_circle(c, p.pile_radius, p.fit_tol, center)) {
      out.push_back(center);
    }
  }
  return out;
}

}  // namespace nav
