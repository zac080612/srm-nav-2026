#pragma once
// ============================================================================
//  零依赖测试宏（约 40 行）
// ----------------------------------------------------------------------------
//  为什么不直接用 gtest：
//    现在这台机器没有 C++ 工具链、也没 ROS 环境，用自带的宏可以做到
//    "装好编译器就能跑"，不需要联网拉 googletest。
//    以后在 Ubuntu/ROS 环境里想换成 gtest，只需把本文件的宏替换掉即可，
//    测试文件的写法（CHECK_* 宏 + main 里 return summary）可以保持不变。
// ============================================================================

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <sstream>
#include <string>

namespace navtest {

inline int g_checks = 0;
inline int g_failures = 0;

template <typename T>
inline std::string to_str(const T& v) {
  std::ostringstream os;
  os << v;
  return os.str();
}

inline std::string to_str(const uint8_t& v) {
  char buf[16];
  std::snprintf(buf, sizeof(buf), "%u(0x%02X)", static_cast<unsigned>(v), static_cast<unsigned>(v));
  return buf;
}

inline void fail(const char* file, int line, const std::string& what) {
  ++g_failures;
  std::printf("[FAIL] %s:%d  %s\n", file, line, what.c_str());
}

template <typename A, typename B>
inline void check_eq(const A& a, const B& b, const char* ea, const char* eb, const char* file, int line) {
  ++g_checks;
  if (!(a == b)) {
    fail(file, line, std::string(ea) + " == " + eb +
                        "   实际=" + to_str(a) + "  期望=" + to_str(b));
  }
}

inline void check_true(bool c, const char* expr, const char* file, int line) {
  ++g_checks;
  if (!c) fail(file, line, std::string(expr) + "  应为 true，实际 false");
}

inline void check_near(double a, double b, double tol, const char* ea, const char* eb, const char* file, int line) {
  ++g_checks;
  if (!(std::fabs(a - b) <= tol)) {
    fail(file, line, std::string("|") + ea + " - " + eb + "| <= " + navtest::to_str(tol) +
                        "   实际 " + navtest::to_str(a) + " vs " + navtest::to_str(b));
  }
}

// 逐字节比较：失败时直接指出**第几个偏移不同**，这是调协议帧最省时间的报错形式
inline void check_bytes(const uint8_t* actual, const uint8_t* expected, size_t n,
                        const char* what, const char* file, int line) {
  ++g_checks;
  for (size_t i = 0; i < n; ++i) {
    if (actual[i] != expected[i]) {
      char buf[160];
      std::snprintf(buf, sizeof(buf), "%s  偏移 %zu 不同：实际 %02X，期望 %02X",
                    what, i, static_cast<unsigned>(actual[i]), static_cast<unsigned>(expected[i]));
      fail(file, line, buf);
      return;
    }
  }
}

inline int summary(const char* name) {
  std::printf("%-22s 检查 %d 项，失败 %d 项\n", name, g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

}  // namespace navtest

#define CHECK_TRUE(cond) ::navtest::check_true((cond), #cond, __FILE__, __LINE__)
#define CHECK_EQ(a, b)   ::navtest::check_eq((a), (b), #a, #b, __FILE__, __LINE__)
#define CHECK_NEAR(a, b, tol) ::navtest::check_near((a), (b), (tol), #a, #b, __FILE__, __LINE__)
#define CHECK_BYTES(actual, expected, n) \
  ::navtest::check_bytes((actual), (expected), (n), #actual, __FILE__, __LINE__)
