#pragma once
#include "nav/control.hpp"  // Vec2

namespace nav {

// 桩系定位：以下桩为原点、下桩→上桩方向为 x 轴的坐标系。
// 只用两桩在车体系的位置，不需要里程计、不需要 yaw。
// 两桩是场地里固定不动的实物，所以这个坐标系稳定、无累积误差。

// 车在桩系的位置
Vec2 car_in_pile_frame(const Vec2& b_down, const Vec2& b_up);

// 桩系向量 → 车体系向量
Vec2 pile_to_body(const Vec2& v_pile, const Vec2& b_down, const Vec2& b_up);

// 车→停车位 的车体系方向（StraightOut 用）。
// p_park 是停车位在桩系的坐标（启动时车在停车位标定一次）。
Vec2 to_park_body(const Vec2& b_down, const Vec2& b_up, const Vec2& p_park);

}  // namespace nav