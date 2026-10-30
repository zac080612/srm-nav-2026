#pragma once
#include <cstddef>
#include <vector>

#include "nav/control.hpp"  // Vec2

namespace nav {

// 桩检测：从 2D 点集检测桩（截面是圆）。桩半径未知，用一般圆拟合。
struct PileDetectParams {
  double cluster_dist{0.15};   // 聚类距离阈值（米）
  size_t min_points{5};        // 一个桩至少这么多扫描点
  double min_radius{0.03};     // 桩半径下限（米）
  double max_radius{0.30};     // 桩半径上限（米）
  double max_fit_rms{0.02};    // 点到拟合圆的 RMS 残差上限（米），非圆形杂物靠它排除
  double min_arc_deg{60.0};    // 点覆盖的最小圆心角（度），直线/墙面靠它排除
};

// 检测到的桩：圆心 + 半径
struct Pile {
  Vec2 center;
  double radius{0.0};
};

// 输入 2D 点集（车体系），输出检测到的桩。
std::vector<Pile> detect_piles(const std::vector<Vec2>& points,
                               const PileDetectParams& p);

}  // namespace nav
