// T1 的一部分：R_bc 映射（那个 90 度轴交叉）与自检
#include <cstdio>
#include <fstream>
#include <string>

#include "nav/rbc_map.hpp"

#include "nav/test_util.hpp"

namespace {

nav::Mat2 make(double r11, double r12, double r21, double r22) {
  nav::Mat2 m;
  m.m[0][0] = r11;
  m.m[0][1] = r12;
  m.m[1][0] = r21;
  m.m[1][1] = r22;
  return m;
}

}  // namespace

int main() {
  // ================= 1) 四种候选旋转各自的映射结果 =================
  // v_body = (1, 0)：希望看到 vx/vy 分别等于该候选矩阵的第一列
  {
    const nav::Mat2 ident = make(1, 0, 0, 1);   // 0 度
    const auto c = nav::map_body_to_protocol(ident, 1.0, 0.0);
    CHECK_NEAR(c.vx, 1.0, 1e-6);
    CHECK_NEAR(c.vy, 0.0, 1e-6);
  }
  {
    const nav::Mat2 rot90 = make(0, -1, 1, 0);  // 90 度
    const auto c = nav::map_body_to_protocol(rot90, 1.0, 0.0);
    CHECK_NEAR(c.vx, 0.0, 1e-6);
    CHECK_NEAR(c.vy, 1.0, 1e-6);
  }
  {
    const nav::Mat2 rot180 = make(-1, 0, 0, -1);
    const auto c = nav::map_body_to_protocol(rot180, 1.0, 0.0);
    CHECK_NEAR(c.vx, -1.0, 1e-6);
    CHECK_NEAR(c.vy, 0.0, 1e-6);
  }
  {
    // 协议字面直译：vx = +v_body_y、vy = -v_body_x
    // 这是**起点参考**，不是结论 —— 真值必须靠 10/11 的实车标定。
    const nav::Mat2 literal = make(0, 1, -1, 0);
    const auto cx = nav::map_body_to_protocol(literal, 1.0, 0.0);
    CHECK_NEAR(cx.vx, 0.0, 1e-6);
    CHECK_NEAR(cx.vy, -1.0, 1e-6);
    const auto cy = nav::map_body_to_protocol(literal, 0.0, 1.0);
    CHECK_NEAR(cy.vx, 1.0, 1e-6);
    CHECK_NEAR(cy.vy, 0.0, 1e-6);
  }

  // ================= 2) 正交性自检 =================
  {
    CHECK_TRUE(nav::is_rotation_like(make(0, 1, -1, 0)));       // 合法旋转
    CHECK_TRUE(nav::is_rotation_like(make(1, 0, 0, 1)));
    CHECK_TRUE(!nav::is_rotation_like(make(0.5, 0, 0, 1)));     // det = 0.5，缩放不是旋转
    CHECK_TRUE(!nav::is_rotation_like(make(0, 0, 0, 0)));       // 全零，典型"文件没读到"
  }

  // ================= 3) load_rbc 两种写法 =================
  {
    const std::string path = "rbc_test_tmp.txt";
    {
      std::ofstream os(path);
      os << "# 标定结果 2026-10-11\n";
      os << "r11=0 r12=1\n";
      os << "r21=-1 r22=0\n";
    }
    nav::Mat2 m;
    std::string err;
    const bool ok = nav::load_rbc(path, m, err);
    CHECK_TRUE(ok);
    if (ok) {
      CHECK_NEAR(m.m[0][0], 0.0, 1e-9);
      CHECK_NEAR(m.m[0][1], 1.0, 1e-9);
      CHECK_NEAR(m.m[1][0], -1.0, 1e-9);
      CHECK_NEAR(m.m[1][1], 0.0, 1e-9);
    } else {
      std::printf("    load_rbc 失败原因: %s\n", err.c_str());
    }
    std::remove(path.c_str());
  }
  {
    const std::string path = "rbc_test_tmp2.txt";
    {
      std::ofstream os(path);
      os << "0 -1 1 0\n";  // 90 度，四个数按 r11 r12 r21 r22
    }
    nav::Mat2 m;
    std::string err;
    const bool ok = nav::load_rbc(path, m, err);
    CHECK_TRUE(ok);
    if (ok) {
      CHECK_NEAR(m.m[0][1], -1.0, 1e-9);
      CHECK_NEAR(m.m[1][0], 1.0, 1e-9);
    }
    std::remove(path.c_str());
  }
  {
    // 文件不存在时必须**返回 false**（不要静默用默认值，那会让车按错的映射跑）
    nav::Mat2 m;
    std::string err;
    CHECK_TRUE(!nav::load_rbc("no_such_file_rbc.txt", m, err));
    CHECK_TRUE(!err.empty());
  }

  return navtest::summary("test_rbc");
}
