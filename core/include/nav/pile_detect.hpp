#pragma once
#include <cstddef>
#include <vector>

#include "nav/control.hpp"  // Vec2

namespace nav {

// 桩检测：从 2D 点集检测桩圆心（桩截面是圆；实车 3D 点云投影到水平面复用同一套）
struct PileDetectParams {
  double pile_radius{0.08};    // 桩半径（米）
  double cluster_dist{0.15};   // 聚类距离阈值（米）
  size_t min_points{5};        // 一个桩至少这么多扫描点
  double fit_tol{0.03};        // 圆拟合容差（米）
};

// 输入 2D 点集（车体系），输出检测到的桩圆心。
std::vector<Vec2> detect_piles(const std::vector<Vec2>& points,
                               const PileDetectParams& p);

}  // namespace nav
