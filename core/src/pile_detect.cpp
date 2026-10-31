#include "nav/pile_detect.hpp"

#include <algorithm>
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

// 圆形检查：点到拟合圆的残差要小，且点在圆上要铺开足够角度
// （排除墙、桌沿这类"一段弧也能拟合出圆"的杂物）
bool pile_like(const std::vector<Vec2>& pts, const Vec2& c, double r,
               double max_rms, double min_arc_deg) {
  if (pts.size() < 3) return false;

  double se = 0.0;
  for (const auto& p : pts) {
    const double d = std::hypot(p.x - c.x, p.y - c.y) - r;
    se += d * d;
  }
  if (std::sqrt(se / static_cast<double>(pts.size())) > max_rms) return false;

  std::vector<double> angs;
  angs.reserve(pts.size());
  for (const auto& p : pts) angs.push_back(std::atan2(p.y - c.y, p.x - c.x));
  std::sort(angs.begin(), angs.end());

  double gap = angs.front() + 2.0 * M_PI - angs.back();   // 首尾相接那个间隙
  for (size_t i = 1; i < angs.size(); ++i) {
    gap = std::max(gap, angs[i] - angs[i - 1]);
  }
  return (360.0 - gap * 180.0 / M_PI) >= min_arc_deg;
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
        radius >= p.min_radius && radius <= p.max_radius &&
        pile_like(c, center, radius, p.max_fit_rms, p.min_arc_deg)) {
      out.push_back({center, radius});
    }
  }
  return out;
}

// 结构化桩检测：高带细杆点定圆心，低带圆环验证半径。
// 为什么不用「低带弧直接拟合圆心」：单层激光只看到桩一小段弧，短弧拟合
// 圆心会沿法线漂移几十厘米（半径反而准）。细杆点稀疏但投影就是圆心，
// 用它定位、用低带环验证半径，两个带互相兜底：
//   矮圆柱/半球无杆 → 高带无点；粗柱/细柱 → 高带散布 = 柱半径，太大。
std::vector<Pile> detect_piles_structured(const std::vector<Vec2>& low_points,
                                          const std::vector<Vec2>& high_points,
                                          const PileDetectParams& p,
                                          const PileStructureParams& sp) {
  std::vector<Pile> out;
  const auto clusters = cluster_points(high_points, p.cluster_dist);
  for (const auto& cl : clusters) {
    const int n_high = static_cast<int>(cl.size());
    if (n_high < sp.high_min_pts) continue;

    // 细杆簇质心 = 候选圆心（细杆投影准，短弧拟合圆心会漂，不能用它定位）
    Vec2 c_h{0.0, 0.0};
    for (const auto& q : cl) {
      c_h.x += q.x;
      c_h.y += q.y;
    }
    c_h.x /= n_high;
    c_h.y /= n_high;

    // 细杆细：簇的散布（点到质心最大距离）必须小。
    // 上下一样粗的柱子散布 = 柱半径（≥5cm），真桩细杆散布 ≈1cm。
    double spread = 0.0;
    for (const auto& q : cl) {
      spread = std::max(spread, std::hypot(q.x - c_h.x, q.y - c_h.y));
    }
    if (spread > sp.high_spread_max) continue;

    // 圆心周围低带环：收集「像半圆」的点，拟合验证半径
    const double r_lo = 0.4 * p.min_radius;
    const double r_hi = 1.6 * p.max_radius;
    std::vector<Vec2> ring;
    for (const auto& q : low_points) {
      const double d = std::hypot(q.x - c_h.x, q.y - c_h.y);
      if (d >= r_lo && d <= r_hi) ring.push_back(q);
    }
    if (ring.size() < p.min_points) {
      // 近距离豁免：半球在雷达俯角盲区（雷达高 0.2m，桩近于 0.8m 时打不到
      // 半球），环空是正常现象。只对 1m 内的簇豁免——绕桩半径 0.57m 的近桩
      // 必然在这个范围；远处的箱子棱角（竖直棱投影成点、散布≈0）不会豁免。
      if (n_high >= 4 && std::hypot(c_h.x, c_h.y) < 1.0) {
        out.push_back({c_h, 0.10});
      }
      continue;
    }

    Vec2 c_fit;
    double r_fit = 0.0;
    if (!fit_circle(ring, c_fit, r_fit)) continue;
    if (r_fit < p.min_radius || r_fit > p.max_radius) continue;
    if (!pile_like(ring, c_fit, r_fit, p.max_fit_rms, 0.0)) continue;
    // 拟合圆心必须与细杆质心一致（排除「细杆旁边恰有别的圆」）
    if (std::hypot(c_fit.x - c_h.x, c_fit.y - c_h.y) > sp.high_match_r) continue;
    out.push_back({c_fit, r_fit});
  }
  return out;
}

}  // namespace nav
