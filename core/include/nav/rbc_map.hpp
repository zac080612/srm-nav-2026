#pragma once
#include <string>

namespace nav {

// R_bc：把车体系速度映射成协议字段 (vx,vy) 的 2x2 旋转（就是 90° 轴交叉）
// 字面直译 [[0,1],[-1,0]]，实车必须标定确认
struct Mat2 {
  double m[2][2]{{0.0, 1.0}, {-1.0, 0.0}};
};

struct ProtoCmd {
  float vx{0.0f};
  float vy{0.0f};
};

// 从文本文件读 4 个数，两种写法都接受：
//     r11=0 r12=1 r21=-1 r22=0        （键值，顺序任意）
//     0 1 -1 0                        （按 r11 r12 r21 r22 顺序）
// 以 '#' 开头的行视为注释。失败时 err 写明原因（打不开 / 个数不够 / 非法数字）。
bool load_rbc(const std::string& path, Mat2& out, std::string& err);

// 核心映射：一行矩阵乘法
ProtoCmd map_body_to_protocol(const Mat2& rbc, double v_body_x, double v_body_y);

// 自检：|det(R_bc)| 应接近 1（旋转，或带一次反射）。
// 标定文件里数字写错（比如少了符号）时用它报警，比让车乱跑便宜。
bool is_rotation_like(const Mat2& rbc, double tol = 0.05);

}  // namespace nav
