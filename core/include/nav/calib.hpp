#pragma once
#include <vector>

#include "nav/control.hpp"    // Vec2
#include "nav/rbc_map.hpp"    // Mat2

namespace nav {

// R_bc 标定：点云静止参照物法，返回车运动方向（两帧质心差）
bool estimate_translation_direction(const std::vector<Vec2>& before,
                                    const std::vector<Vec2>& after,
                                    Vec2& dir_out);

// 一个标定样本：发的方向 + 测得的实际运动方向
struct CalibSample {
  Vec2 v_body;      // 期望的车体系运动方向（单位向量）
  Vec2 v_measured;  // estimate 测得的车运动方向（单位向量）
};

// 由样本解 R_bc：R_bc * v_body ≈ v_measured（样本不足或共线返回 false）
bool solve_rbc_from_samples(const std::vector<CalibSample>& samples, Mat2& rbc_out);

}  // namespace nav
