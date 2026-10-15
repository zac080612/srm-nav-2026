#pragma once

namespace nav {

struct Vec2 {
  double x{0.0};
  double y{0.0};
};

struct OrbitParams {
  double R{0.55};    // 目标绕行半径 = 桩半径 + 车体半径 0.27 + 余量（10/14 实测后定）
  double v_t{0.08};  // 切向速度（起步 0.05–0.1）
  double k_r{0.20};  // 径向 P 增益
  bool ccw{true};    // true = 逆时针（下瓣），false = 顺时针（上瓣）
};

// 绕桩控制律：输入桩在车体系的位置，输出车体系期望速度
// v = v_t*t + k_r*(r-R)*u（t 切向，u 朝桩，k_r*(r-R) 径向修正）
Vec2 orbit_step(const Vec2& b_body, const OrbitParams& p);

// 参数合法性：R > 0、v_t >= 0、k_r >= 0，且三者都有限
bool valid(const OrbitParams& p);

}  // namespace nav
