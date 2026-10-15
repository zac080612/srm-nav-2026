#include "nav/rbc_map.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace nav {

namespace {
const char* kKeys[4] = {"r11", "r12", "r21", "r22"};
}  // namespace

// 从文件读 R_bc（支持键值和四个数两种写法）
bool load_rbc(const std::string& path, Mat2& out, std::string& err) {
  std::ifstream is(path);
  if (!is) {
    err = "打不开文件: " + path;
    return false;
  }

  double v[4] = {0.0, 0.0, 0.0, 0.0};
  bool got[4] = {false, false, false, false};
  int positional = 0;

  std::string line;
  while (std::getline(is, line)) {
    const size_t hash = line.find('#');
    if (hash != std::string::npos) line = line.substr(0, hash);

    if (line.find('=') != std::string::npos) {
      std::istringstream ls(line);
      std::string token;
      while (ls >> token) {
        const size_t eq = token.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = token.substr(0, eq);
        const std::string valstr = token.substr(eq + 1);
        char* endp = nullptr;
        const double val = std::strtod(valstr.c_str(), &endp);
        if (endp == valstr.c_str()) {
          err = "非法数字: " + token;
          return false;
        }
        for (int i = 0; i < 4; ++i) {
          if (key == kKeys[i]) { v[i] = val; got[i] = true; }
        }
      }
    } else {
      std::istringstream ls(line);
      double val = 0.0;
      while (positional < 4 && (ls >> val)) {
        v[positional] = val;
        got[positional] = true;
        ++positional;
      }
    }
  }

  for (int i = 0; i < 4; ++i) {
    if (!got[i]) {
      err = "缺少 " + std::string(kKeys[i]);
      return false;
    }
  }

  out.m[0][0] = v[0];
  out.m[0][1] = v[1];
  out.m[1][0] = v[2];
  out.m[1][1] = v[3];
  return true;
}

// 矩阵乘法：车体系 → 协议字段
ProtoCmd map_body_to_protocol(const Mat2& rbc, double v_body_x, double v_body_y) {
  ProtoCmd c;
  c.vx = static_cast<float>(rbc.m[0][0] * v_body_x + rbc.m[0][1] * v_body_y);
  c.vy = static_cast<float>(rbc.m[1][0] * v_body_x + rbc.m[1][1] * v_body_y);
  return c;
}

// 自检：|det| 接近 1 才是旋转
bool is_rotation_like(const Mat2& rbc, double tol) {
  const double det = rbc.m[0][0] * rbc.m[1][1] - rbc.m[0][1] * rbc.m[1][0];
  return std::fabs(std::fabs(det) - 1.0) <= tol;
}

}  // namespace nav
