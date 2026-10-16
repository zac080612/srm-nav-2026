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

// Kasa 圆拟合：最小二乘拟合 x²+y²+Dx+Ey+F=0，得圆心和半径
bool fit_circle(const std::vector<Vec2>& pts, Vec2& center, double& radius) {
  const size_t n = pts.size();
  if (n < 3) return false;

  double sxx = 0, sxy = 0, syy = 0, sx = 0, sy = 0;
  double sxr = 0, syr = 0, sr = 0;
  for (const auto& p : pts) {
    const double r2 = p.x * p.x + p.y * p.y;
    sxx += p.x * p.x;
    sxy += p.x * p.y;
    syy += p.y * p.y;
    sx += p.x;
    sy += p.y;
    sxr += p.x * r2;
    syr += p.y * r2;
    sr += r2;
  }
  const double N = static_cast<double>(n);

  double a[3][3] = {{sxx, sxy, sx}, {sxy, syy, sy}, {sx, sy, N}};
  double b[3] = {-sxr, -syr, -sr};

  for (int k = 0; k < 3; ++k) {
    int piv = k;
    for (int i = k + 1; i < 3; ++i)
      if (std::fabs(a[i][k]) > std::fabs(a[piv][k])) piv = i;
    if (std::fabs(a[piv][k]) < 1e-12) return false;
    if (piv != k) {
      for (int j = 0; j < 3; ++j) std::swap(a[k][j], a[piv][j]);
      std::swap(b[k], b[piv]);
    }
    for (int i = k + 1; i < 3; ++i) {
      const double f = a[i][k] / a[k][k];
      for (int j = k; j < 3; ++j) a[i][j] -= f * a[k][j];
      b[i] -= f * b[k];
    }
  }
  double sol[3] = {0, 0, 0};
  for (int i = 2; i >= 0; --i) {
    double s = b[i];
    for (int j = i + 1; j < 3; ++j) s -= a[i][j] * sol[j];
    sol[i] = s / a[i][i];
  }

  const double D = sol[0], E = sol[1], F = sol[2];
  center = Vec2{-D * 0.5, -E * 0.5};
  const double r2 = D * D * 0.25 + E * E * 0.25 - F;
  if (r2 <= 0) return false;
  radius = std::sqrt(r2);
  return true;
}

}  // namespace

// 从点集检测桩（半径未知，拟合后按半径范围筛）
std::vector<Pile> detect_piles(const std::vector<Vec2>& points,
                               const PileDetectParams& p) {
  std::vector<Pile> out;
  const auto clusters = cluster_points(points, p.cluster_dist);
  for (const auto& c : clusters) {
    if (c.size() < p.min_points) continue;
    Vec2 center;
    double radius = 0;
    if (fit_circle(c, center, radius) &&
        radius >= p.min_radius && radius <= p.max_radius) {
      out.push_back({center, radius});
    }
  }
  return out;
}

}  // namespace nav
