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

// 结构验证参数：真桩 = 低带（下半圆）+ 上方稀疏细杆点
struct PileStructureParams {
  double high_match_r{0.10};   // 细杆质心与拟合圆心允许偏差（米）
  int high_min_pts{2};         // 细杆簇最少点数：上方 0 点 = 矮杂物
  double high_spread_max{0.03};  // 细杆簇散布上限（米）：细杆 r=1cm，打点散布<2cm；
                                 // 上下一样粗的柱子（r≥5cm）散布远超此值，靠它排除。
                                 // 不用点数比例：俯角几何下杆/半球的可见线数不同，
                                 // 点数比例随距离和俯角覆盖变化，不稳定。
};

// 输入 2D 点集（车体系），输出检测到的桩。
std::vector<Pile> detect_piles(const std::vector<Vec2>& points,
                               const PileDetectParams& p);

// 结构化桩检测：高带（细杆）点定圆心，低带（半圆）环验证半径。
// 为什么不用「低带弧直接拟合圆心」：单层激光只看到桩一小段弧，短弧拟合
// 圆心会沿法线漂移几十厘米（半径反而准）。细杆点稀疏但投影就是圆心，
// 用它定位、用低带环验证半径，两个带互相兜底，对杂物免疫：
//   矮圆柱/半球无杆 → 高带无点；粗柱/细柱 → 高带点太多或粗细比例≈1。
std::vector<Pile> detect_piles_structured(const std::vector<Vec2>& low_points,
                                          const std::vector<Vec2>& high_points,
                                          const PileDetectParams& p,
                                          const PileStructureParams& sp);

}  // namespace nav
